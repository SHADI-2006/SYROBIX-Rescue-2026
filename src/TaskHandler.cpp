/**
 * ============================================================================
 *  @file       TaskHandler.cpp
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Manipulator, Indicator, the five FreeRTOS task bodies, and the
 *              mission state machine implementing the SYROBIX rescue rules.
 *
 *  @dependency adafruit/Adafruit PWM Servo Driver Library @ ^3.0.2
 *              (pulls in adafruit/Adafruit BusIO)
 *
 *  ---------------------------------------------------------------------------
 *  REVISION 3 — PCA9685 PERIPHERAL BUS
 *  ---------------------------------------------------------------------------
 *   ESP32Servo and every LEDC servo/buzzer call are gone. Both servos, the
 *   status LED and the ACTIVE buzzer are PCA9685 channels (PcaChannels:: in
 *   Config.h) on the global `pca` driver. TaskHandler::begin() runs
 *   pca.begin() + pca.setPWMFreq(50); servo angles are written as
 *   pca.setPWM(ch, 0, map(angle, 0, 180, 102, 491)); the LED and buzzer use
 *   the full-ON (4096, 0) / full-OFF (0, 4096) codes. PcaBus (below):
 *     - checks every setPWM() acknowledgement and retries failed writes,
 *     - reads MODE1/PRESCALE back once a second and re-initialises a chip
 *       that a servo spike has reset, then re-sends every output.
 *   The Manipulator slew limiter is unchanged. Indicator::tone(Hz, ms) became
 *   beep(ms): an active buzzer has one pitch.
 *
 *  ---------------------------------------------------------------------------
 *  AUDIT REVISION 2 — what changed in this file, keyed to the audit
 *  ---------------------------------------------------------------------------
 *   [A-C1] DRIVE MAILBOX. Only the Core-1 control task writes the motors now.
 *          The mission task posts a single packed 32-bit command (atomic on
 *          Xtensa); the control task applies it whenever line following is
 *          off. Previously the mission called tank()/brake() from Core 0 while
 *          followLine() ran on Core 1, and an in-flight followLine() could
 *          overwrite a brake — e.g. rolling the robot off a mine mid-dwell.
 *          Ordering rule: post the command FIRST, then clear line following.
 *
 *   [A-C2] Line PID is reset on the rising edge of line following and on
 *          every line re-acquisition (no stale-derivative kick).
 *
 *   [A-C3] Victim scan is STOP-AND-GO at SPEED_PIVOT, and every sample must be
 *          a ToF reading that started after the chassis settled. Align is
 *          IMU-closed with brake-lead and pulse nudges, then range-verified.
 *
 *   [A-C4] A branch turn must LEAVE the line it stands on (or rotate >= 60°)
 *          before it may accept a re-acquisition. At a 4-way it used to
 *          "succeed" on tick one and drive straight.
 *
 *   [A-C5] Markers accumulate over a distance window and are decided AT the
 *          bar: both sides -> U-turn, one side -> that side, none -> straight.
 *          Markers seen after a bar (they belong to other approaches) are
 *          ignored. Green-tile confirmation is by distance.
 *
 *   [A-C6] An object ahead before the victim is aboard is VERIFIED first: the
 *          robot creeps forward looking for the green safe tile, because the
 *          victim itself trips obstacleAhead before green can confirm.
 *
 *   [A-C7] Obstacle bypass is IMU-angle + encoder-distance based and returns
 *          TOWARD the line (the old arc curved away from it).
 *
 *   [A-C8] EXIT_SEEK uses full navigation. Red is classified by run LENGTH:
 *          a short run is the evacuation stripe, a long run a mine tile
 *          (defused, then the exit search continues).
 *
 *   [A-C9] CALIBRATING teaches WHITE/BLACK/GREEN/RED on the field (button per
 *          colour, beep count = which colour), via posted requests.
 *
 *   [A-H3] Victim sequence retries are bounded (MAX_VICTIM_ATTEMPTS).
 *   [A-H4] Mine: park the chassis centre on the tile centre, then dwell; red
 *          is ignored for MINE_REARM_MM afterwards (no same-mine livelock).
 *   [A-H5] Arm is lowered only at the end of the approach; after the lift the
 *          robot U-turns and re-finds the line before seeking the exit.
 *   [A-V1] CALIBRATING is entered BEFORE any task exists.
 *
 *  Earlier deviations still in force: the victim scan rotates the chassis
 *  (no pan servo); the 5-second mine dwell is a phase timer, not vTaskDelay.
 * ============================================================================
 */

#include "TaskHandler.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdlib.h>


// ============================================================================
//  File-local state
// ============================================================================

namespace {

// ============================================================================
//  [Rev 3] PCA9685 driver instance and its health state (mission task only)
// ============================================================================
} // anonymous namespace (reopened below)

/**
 * The PCA9685 driver: address 0x40 (Hw::PCA9685_ADDR) on the default Wire
 * bus — the same bus SensorsSystem::begin() starts on GPIO9/10. Global so
 * TaskHandler::begin(), Manipulator and Indicator all write through it.
 */
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(Hw::PCA9685_ADDR);
static_assert(Hw::PCA9685_ADDR == 0x40, "Board jumpers set for 0x40.");
static_assert(Hw::SERVO_FREQ_HZ == 50.0f,
              "TaskHandler::begin() calls pca.setPWMFreq(50) literally.");

namespace {

bool     g_pcaUp           = false;
uint8_t  g_pcaPrescale     = 0;     // 0 == never initialised
uint32_t g_pcaGeneration   = 0;
uint8_t  g_pcaFailStreak   = 0;
uint32_t g_pcaLastCheckMs  = 0;

constexpr uint8_t  PCA_REG_MODE1     = 0x00;
constexpr uint8_t  PCA_REG_PRESCALE  = 0xFE;
constexpr uint8_t  PCA_MODE1_SLEEP   = 0x10;
constexpr uint16_t PCA_FULL_BIT      = 4096;   // bit 12: full ON / full OFF

/**
 * Servo pulse endpoints in 12-bit PCA ticks for the 50 Hz frame.
 * With the oscillator calibrated (Hw::PCA_OSC_HZ) one tick = 20 ms / 4096
 * = 4.88 us, so 102 ticks = ~500 us (0 deg) and 491 ticks = ~2400 us
 * (180 deg) — the same Hw::SERVO_MIN_US / SERVO_MAX_US range ESP32Servo
 * used, and therefore the same pulse for every taught pose.
 */
constexpr long SERVO_MIN_TICKS = 102;
constexpr long SERVO_MAX_TICKS = 491;

/** One register read with a real error path (Adafruit's read8() has none). */
bool pcaReadReg(uint8_t reg, uint8_t& out) {
    Wire.beginTransmission(Hw::PCA9685_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(Hw::PCA9685_ADDR, static_cast<uint8_t>(1)) != 1) return false;
    out = static_cast<uint8_t>(Wire.read());
    return true;
}

// ============================================================================
//  Cross-core flags (single-word volatile stores; no locks, never block Core 1)
// ============================================================================
volatile bool    g_lineFollowEnabled   = false;
volatile int16_t g_commandedSpeed      = 0;
/** Line task is gated off while the mission task owns LineArray (CALIBRATING). */
volatile bool    g_lineSamplingEnabled = false;
/** Latched by emergencyStop(), which may be called from an ISR. */
volatile bool    g_emergencyStop       = false;
/** FINISHED / FAULT: the control task drops STBY and stops writing PWM. */
volatile bool    g_driveKill           = false;

// ============================================================================
//  [A-C1] Drive mailbox: mission (Core 0) -> control (Core 1)
// ============================================================================
//  One 32-bit word: [31:22] command, [21:11] left+1024, [10:0] right+1024.
//  A single aligned 32-bit store/load is atomic on Xtensa, so the control task
//  can never see a left value from one command and a right from another.
enum : uint32_t { CMD_BRAKE = 0, CMD_TANK = 1, CMD_COAST = 2 };

constexpr uint32_t packDrive(uint32_t cmd, int16_t l, int16_t r) {
    return (cmd << 22) |
           ((static_cast<uint32_t>(l + 1024) & 0x7FFu) << 11) |
           ( static_cast<uint32_t>(r + 1024) & 0x7FFu);
}

volatile uint32_t g_driveCmd = packDrive(CMD_BRAKE, 0, 0);

inline int16_t clampCmd(int v) {
    return static_cast<int16_t>(v > Hw::PWM_MAX ? Hw::PWM_MAX
                              : (v < -Hw::PWM_MAX ? -Hw::PWM_MAX : v));
}
inline void postDrive(uint32_t cmd, int l = 0, int r = 0) {
    g_driveCmd = packDrive(cmd, clampCmd(l), clampCmd(r));
}
inline void driveTank(int l, int r) { postDrive(CMD_TANK, l, r); }
inline void driveBrake()            { postDrive(CMD_BRAKE); }

/** Zero-radius pivot. LEFT == counter-clockwise == positive yaw. */
inline void drivePivot(int effort, TurnDirection dir) {
    const int m = abs(effort);
    if (dir == TurnDirection::LEFT) {
        driveTank(-m, m);
    } else if (dir == TurnDirection::RIGHT || dir == TurnDirection::U_TURN) {
        driveTank(m, -m);
    } else {
        driveBrake();
    }
}

/** Post BRAKE first, THEN stop line following: the control task's next tick
 *  (flag false) applies the brake; a followLine() already in flight can
 *  write at most one 5 ms tick before being overridden. */
inline void stopDriving() {
    postDrive(CMD_BRAKE);
    g_lineFollowEnabled = false;
}
inline void startLineFollow(int16_t speed) {
    g_commandedSpeed    = speed;    // speed first, then enable
    g_lineFollowEnabled = true;
}

/** Control task only. */
void applyDriveCommand(RobotDrivetrain& drive, uint32_t c) {
    const int16_t l = static_cast<int16_t>(static_cast<int32_t>((c >> 11) & 0x7FFu) - 1024);
    const int16_t r = static_cast<int16_t>(static_cast<int32_t>(c & 0x7FFu) - 1024);
    switch (c >> 22) {
        case CMD_TANK:  drive.tank(l, r); break;
        case CMD_COAST: drive.coast();    break;
        case CMD_BRAKE:
        default:        drive.brake();    break;
    }
}

// ============================================================================
//  Sub-phase state — reset by transitionTo() unless marked PERSISTENT
// ============================================================================
uint8_t  g_phase        = 0;
uint32_t g_phaseStartMs = 0;
uint8_t  g_blinkCount   = 0;
uint8_t  g_aux          = 0;      // nudge counter, etc.
float    g_refYaw       = 0.0f;
float    g_refMm        = 0.0f;
float    g_targetMm     = 0.0f;
float    g_stepTarget   = 0.0f;
float    g_prevYaw      = 0.0f;
float    g_turnAccum    = 0.0f;
uint32_t g_brakeMs      = 0;
int8_t   g_alignSign    = 0;
bool     g_pulse        = false;
bool     g_sawLost      = false;
TurnDirection g_avoidDir = TurnDirection::RIGHT;
uint16_t g_rangeAtLower = 0;

// --- Victim scan working set ------------------------------------------------
float    g_scanStartYaw   = 0.0f;
uint16_t g_scanBestRange  = Tune::TOF_INVALID;
float    g_scanBestYaw    = 0.0f;

// --- Navigation latches (reset per state) -----------------------------------
bool     g_mkL = false, g_mkR = false;
float    g_mkStartMm      = -1.0f;
float    g_barMm          = -1.0f;
float    g_greenStartMm   = -1.0f;
float    g_redStartMm     = -1.0f;
uint8_t  g_redFrames      = 0;
uint32_t g_lastColorStamp = 0;
uint32_t g_lostSinceMs    = 0;
float    g_lostSinceMm    = 0.0f;

// --- PERSISTENT across transitions (set before a transition, read after) ----
float    g_redLockoutUntilMm   = -1.0e9f;
float    g_greenIgnoreUntilMm  = -1.0e9f;
float    g_turnAdvanceMm       = Hw::LINE_ARRAY_TO_PIVOT_MM;
float    g_redRunAtConfirm     = 0.0f;
uint8_t  g_calColorIdx         = 0;

constexpr TileColor CAL_SEQUENCE[4] = {
    TileColor::WHITE, TileColor::BLACK, TileColor::GREEN, TileColor::RED
};
constexpr uint8_t CAL_COLORS = 4;

// --- Sequence watchdogs -----------------------------------------------------
constexpr uint32_t WD_TURNING_MS   = 4000;
constexpr uint32_t WD_UTURN_MS     = 8000;   // 180 + line search
constexpr uint32_t WD_SCAN_MS      = 15000;  // stop-and-go, ~20 steps
constexpr uint32_t WD_ALIGN_MS     = 6000;
constexpr uint32_t WD_APPROACH_MS  = 8000;
constexpr uint32_t WD_OBSTACLE_MS  = 10000;

// --- Motion tuning ----------------------------------------------------------
constexpr uint16_t GRIP_TOLERANCE_MM     = 12;
constexpr float    U_TURN_DONE_DEG       = 176.0f;
constexpr uint32_t DEAD_END_PAUSE_MS     = 300;
constexpr float    UTURN_SEARCH_DEG      = 60.0f;
constexpr float    TURN_MIN_DEG          = 30.0f;  // never accept a branch sooner
constexpr float    TURN_ASSUME_LEFT_DEG  = 60.0f;  // rotated this far == left the line
constexpr float    APPROACH_OVERSHOOT_MM = 60.0f;
constexpr uint32_t GRIP_SETTLE_TIMEOUT_MS = 2500;

// --- Calibration UI -----------------------------------------------------------
constexpr uint32_t CAL_SWEEP_MS      = 4000;
constexpr uint32_t CAL_SETTLE_MS     = 1500;   // operator sets the robot down
constexpr uint32_t CAL_BEEP_PERIOD   = 300;
constexpr uint16_t CAL_BEEP_MS       = 100;    // prompt: short beeps
constexpr uint16_t CAL_ACK_MS        = 400;    // ack: one long beep (active
                                               // buzzer: duration is the only
                                               // thing that can differ)

/** Servo slew rate. Slamming an MG996R across its travel browns out the rail. */
constexpr float    SERVO_DEG_PER_SEC   = 140.0f;

inline float wrapDeg(float d) {
    while (d >= 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

inline bool rangeValid(uint16_t r) { return r < Tune::TOF_INVALID; }

/** True once the FRONT range in @p snap was collected at least settle+budget
 *  after @p sinceMs, i.e. the measurement itself started after the robot
 *  stopped and stopped rocking. */
inline bool freshFrontSample(const SensorSnapshot& snap, uint32_t sinceMs) {
    return snap.rangeFrontStampMs != 0 &&
           static_cast<int32_t>(snap.rangeFrontStampMs -
               (sinceMs + Tune::SCAN_SETTLE_MS + Tof::FRONT_BUDGET_MS)) >= 0;
}

} // anonymous namespace


// ============================================================================
//  PcaBus  [Rev 3]
// ============================================================================

namespace PcaBus {

bool adoptConfiguration() {
    // Called right after pca.begin() + pca.setPWMFreq(50). Reads the result
    // back over I2C with a real error path: the chip must be awake and its
    // prescaler programmed. The prescale is cached so service() can detect a
    // chip that a servo spike has reset to its power-on defaults.
    uint8_t prescale = 0;
    uint8_t mode1    = 0;
    if (!pcaReadReg(PCA_REG_PRESCALE, prescale) ||
        !pcaReadReg(PCA_REG_MODE1, mode1) ||
        (mode1 & PCA_MODE1_SLEEP) != 0 ||
        prescale == 0) {
        g_pcaUp = false;
        return false;
    }

    g_pcaPrescale   = prescale;
    g_pcaFailStreak = 0;
    ++g_pcaGeneration;            // owners re-send their full output state
    g_pcaUp = true;
    return true;
}

void service(uint32_t nowMs) {
    if ((nowMs - g_pcaLastCheckMs) < Hw::PCA_HEALTH_PERIOD_MS) {
        return;
    }
    g_pcaLastCheckMs = nowMs;

    uint8_t mode1 = 0, prescale = 0;
    const bool readOk = pcaReadReg(PCA_REG_MODE1, mode1) &&
                        pcaReadReg(PCA_REG_PRESCALE, prescale);

    if (readOk && g_pcaUp &&
        (mode1 & PCA_MODE1_SLEEP) == 0 && prescale == g_pcaPrescale) {
        g_pcaFailStreak = 0;
        return;                                   // healthy
    }

    if (!readOk) {
        // Bus glitch or chip gone. Tolerate a short streak before declaring
        // it down; a single NACK during a servo spike is not a dead chip.
        if (g_pcaFailStreak < 255) ++g_pcaFailStreak;
        if (g_pcaFailStreak < Hw::PCA_FAIL_STREAK) return;
        g_pcaUp = false;
    }

    // Either it answered but has been RESET (SLEEP set or prescaler back at
    // the power-on default — every output is dead), or it has been
    // unreachable for a full streak. Re-run the same init as
    // TaskHandler::begin(); success bumps the generation and
    // Manipulator/Indicator re-send their state next tick.
    // Costs ~15 ms of blocking delay inside the Adafruit library, once.
    if (!pca.begin()) {
        g_pcaUp = false;
        return;
    }
    pca.setOscillatorFrequency(Hw::PCA_OSC_HZ);
    pca.setPWMFreq(50);
    adoptConfiguration();
}

bool isUp()             { return g_pcaUp; }
void markDown()         { g_pcaUp = false; }
uint32_t generation()   { return g_pcaGeneration; }

bool servoAngle(uint8_t channel, uint8_t angleDeg) {
    if (!g_pcaUp) return false;                     // no bus traffic
    if (angleDeg > Hw::SERVO_MAX_ANGLE) angleDeg = Hw::SERVO_MAX_ANGLE;
    const long ticks = map(static_cast<long>(angleDeg), 0L, 180L,
                           SERVO_MIN_TICKS, SERVO_MAX_TICKS);
    return pca.setPWM(channel, 0, static_cast<uint16_t>(ticks)) == 0;
}

bool digital(uint8_t channel, bool on) {
    if (!g_pcaUp) return false;
    // Dedicated full-ON / full-OFF codes (bit 12), not 4095/4096 duty.
    const uint8_t rc = on ? pca.setPWM(channel, PCA_FULL_BIT, 0)
                          : pca.setPWM(channel, 0, PCA_FULL_BIT);
    return rc == 0;
}

} // namespace PcaBus


// ============================================================================
//  Manipulator
// ============================================================================

Manipulator::Manipulator()
    : _armTarget(Hw::ARM_ANGLE_CARRY), _armCurrent(Hw::ARM_ANGLE_CARRY),
      _gripperTarget(Hw::GRIPPER_OPEN), _gripperCurrent(Hw::GRIPPER_OPEN),
      _slewAccumulator(0.0f), _carrying(false),
      _armWritten(0), _gripperWritten(0),
      _armValid(false), _gripperValid(false),
      _outGeneration(0) {}

void Manipulator::begin() {
    // Start stowed and open: an arm that boots at an unknown angle can drive
    // itself into the chassis before the first update() tick. (The PCA9685
    // powers up with every output OFF, so the servos are limp until this
    // first write — no pulse, no uncommanded twitch.)
    _armCurrent = _armTarget = Hw::ARM_ANGLE_CARRY;
    _gripperCurrent = _gripperTarget = Hw::GRIPPER_OPEN;

    _outGeneration = PcaBus::generation();
    writeServo(PcaChannels::SERVO_ARM,     _armCurrent,     _armWritten,     _armValid);
    writeServo(PcaChannels::SERVO_GRIPPER, _gripperCurrent, _gripperWritten, _gripperValid);
}

void Manipulator::openGripper()  { _gripperTarget = Hw::GRIPPER_OPEN;   _carrying = false; }
void Manipulator::closeGripper() { _gripperTarget = Hw::GRIPPER_CLOSED; }
void Manipulator::armDown()      { _armTarget     = Hw::ARM_ANGLE_DOWN;  }
void Manipulator::armToCarry()   { _armTarget     = Hw::ARM_ANGLE_CARRY; _carrying = true; }

// static
void Manipulator::writeServo(uint8_t channel, uint8_t angle,
                             uint8_t& written, bool& valid) {
    if (PcaBus::servoAngle(channel, angle)) {
        written = angle;
        valid   = true;
    } else {
        valid   = false;   // retried by refreshOutputs() on the next tick
    }
}

void Manipulator::refreshOutputs() {
    const uint32_t gen = PcaBus::generation();
    if (gen != _outGeneration) {
        // The PCA9685 was (re)initialised: its output registers may be blank.
        _outGeneration = gen;
        _armValid      = false;
        _gripperValid  = false;
    }
    if (!_armValid || _armWritten != _armCurrent) {
        writeServo(PcaChannels::SERVO_ARM, _armCurrent, _armWritten, _armValid);
    }
    if (!_gripperValid || _gripperWritten != _gripperCurrent) {
        writeServo(PcaChannels::SERVO_GRIPPER, _gripperCurrent,
                   _gripperWritten, _gripperValid);
    }
}

void Manipulator::update(float dtSec) {
    // [Rev 3] Resend anything a failed write or a PCA re-init left stale.
    // Done FIRST because the slew step below returns early on most ticks.
    refreshOutputs();

    // --- Slew limiter: unchanged from Rev 2 --------------------------------
    // Fractional degrees accumulate here so a slow slew rate is not quantised
    // away by integer servo angles at a 50 Hz tick.
    _slewAccumulator += SERVO_DEG_PER_SEC * dtSec;
    if (_slewAccumulator < 1.0f) {
        return;
    }

    const int16_t step = static_cast<int16_t>(_slewAccumulator);
    _slewAccumulator -= static_cast<float>(step);

    if (_armCurrent != _armTarget) {
        const int16_t delta = static_cast<int16_t>(_armTarget) - _armCurrent;
        const int16_t move  = (abs(delta) < step) ? delta
                                                  : (delta > 0 ? step : -step);
        _armCurrent = static_cast<uint8_t>(_armCurrent + move);
        writeServo(PcaChannels::SERVO_ARM, _armCurrent, _armWritten, _armValid);
    }

    if (_gripperCurrent != _gripperTarget) {
        const int16_t delta = static_cast<int16_t>(_gripperTarget) - _gripperCurrent;
        const int16_t move  = (abs(delta) < step) ? delta
                                                  : (delta > 0 ? step : -step);
        _gripperCurrent = static_cast<uint8_t>(_gripperCurrent + move);
        writeServo(PcaChannels::SERVO_GRIPPER, _gripperCurrent,
                   _gripperWritten, _gripperValid);
    }
}

bool Manipulator::isSettled() const {
    return (_armCurrent == _armTarget) && (_gripperCurrent == _gripperTarget);
}

bool Manipulator::verifyGrasp(uint16_t frontRangeMm) const {
    // Inferred, not sensed: the jaw reached its commanded close, and the
    // ranger still sees something at grip distance. Both are required.
    // A jaw contact switch on one of the GPIOs Rev 3 freed should replace this.
    const bool jawClosed = (_gripperCurrent <= Hw::GRIPPER_CLOSED + 3);
    const bool objectPresent = (frontRangeMm < Tune::TOF_GRIP_MM + 25) &&
                               (frontRangeMm > 5);
    return jawClosed && objectPresent;
}

uint8_t Manipulator::armAngle()     const { return _armCurrent; }
uint8_t Manipulator::gripperAngle() const { return _gripperCurrent; }


// ============================================================================
//  Indicator
// ============================================================================

Indicator::Indicator()
    : _status(StatusColor::OFF),
      _manualOverride(false),
      _patternStartMs(0),
      _lastButtonRaw(true),
      _debouncedState(true),
      _edgePrev(true),
      _lastDebounceMs(0),
      _toneEndMs(0),
      _ledWant(false),    _ledWritten(false),    _ledValid(false),
      _buzzerWant(false), _buzzerWritten(false), _buzzerValid(false),
      _outGeneration(0) {}

void Indicator::begin() {
    pinMode(Pins::START_BUTTON, INPUT_PULLUP);

    _outGeneration = PcaBus::generation();
    writeBuzzer(false);
    writeLed(false);

    setStatus(StatusColor::BOOTING);
}

// static
bool Indicator::patternIsOn(StatusColor status, uint32_t elapsedMs) {
    switch (status) {
        case StatusColor::OFF:
            return false;

        case StatusColor::BOOTING:
            return (elapsedMs % Tune::LED_PATTERN_BOOT_PERIOD_MS) < Tune::LED_PATTERN_BOOT_ON_MS;

        case StatusColor::CALIBRATING:
            return (elapsedMs % Tune::LED_PATTERN_CALIB_PERIOD_MS) < Tune::LED_PATTERN_CALIB_ON_MS;

        case StatusColor::READY:
            return true;   // solid — armed and waiting on the start button

        case StatusColor::RUNNING:
            // Short heartbeat: mostly off, brief blip, so "nominal" doesn't
            // read as visually busy as one of the alert patterns below.
            return (elapsedMs % Tune::LED_PATTERN_RUN_PERIOD_MS) < Tune::LED_PATTERN_RUN_ON_MS;

        case StatusColor::ZONE:
            // [FIX 2] Long slow pulse — never the scored 250/250 mine cadence.
            // Config.h static_asserts that it never becomes one again.
            return (elapsedMs % Tune::LED_PATTERN_ZONE_PERIOD_MS) < Tune::LED_PATTERN_ZONE_ON_MS;

        case StatusColor::CARRYING: {
            // Double-blip then a long pause — distinct from every duty-cycle
            // pattern above, so "victim aboard" reads at a glance.
            const uint32_t phase         = elapsedMs % Tune::LED_PATTERN_CARRY_PERIOD_MS;
            const uint32_t firstOnEnd    = Tune::LED_PATTERN_CARRY_PULSE_MS;
            const uint32_t secondOnStart = firstOnEnd + Tune::LED_PATTERN_CARRY_GAP_MS;
            const uint32_t secondOnEnd   = secondOnStart + Tune::LED_PATTERN_CARRY_PULSE_MS;
            return (phase < firstOnEnd) || (phase >= secondOnStart && phase < secondOnEnd);
        }

        case StatusColor::ERROR:
        default:
            return (elapsedMs % Tune::LED_PATTERN_ERROR_PERIOD_MS) < Tune::LED_PATTERN_ERROR_ON_MS;
    }
}

void Indicator::writeLed(bool on) {
    // Change-driven: the pattern engine calls this at 50 Hz, but the bus is
    // only touched when the commanded state differs from the ACKed one.
    _ledWant = on;
    if (_ledValid && _ledWritten == on) return;
    if (PcaBus::digital(PcaChannels::STATUS_LED, on)) {
        _ledWritten = on;
        _ledValid   = true;
    } else {
        _ledValid   = false;    // retried by refreshOutputs()
    }
}

void Indicator::writeBuzzer(bool on) {
    _buzzerWant = on;
    if (_buzzerValid && _buzzerWritten == on) return;
    if (PcaBus::digital(PcaChannels::BUZZER, on)) {
        _buzzerWritten = on;
        _buzzerValid   = true;
    } else {
        _buzzerValid   = false;
    }
}

void Indicator::refreshOutputs() {
    const uint32_t gen = PcaBus::generation();
    if (gen != _outGeneration) {
        _outGeneration = gen;
        _ledValid      = false;
        _buzzerValid   = false;
    }
    if (!_ledValid)    writeLed(_ledWant);
    if (!_buzzerValid) writeBuzzer(_buzzerWant);
}

void Indicator::setStatus(StatusColor color) {
    _status         = color;
    _manualOverride = false;              // hand control back to the pattern engine
    _patternStartMs = millis();           // every status change starts its pattern at phase 0
    writeLed(patternIsOn(color, 0));      // apply immediately, don't wait for the next tick
}

void Indicator::ledOn() {
    // Latches manual mode — see the class doc in TaskHandler.h.
    _manualOverride = true;
    writeLed(true);
}

void Indicator::ledOff() {
    _manualOverride = true;
    writeLed(false);
}

void Indicator::beep(uint16_t durationMs) {
    writeBuzzer(true);
    _toneEndMs = millis() + durationMs;
    if (_toneEndMs == 0) _toneEndMs = 1;  // 0 is the "no beep" sentinel
}

void Indicator::beepOff() {
    writeBuzzer(false);
    _toneEndMs = 0;
}

bool Indicator::buttonPressed() {
    return !_debouncedState;   // active low
}

bool Indicator::buttonRisingEdge() {
    const bool now  = _debouncedState;
    const bool edge = (_edgePrev && !now);   // released -> pressed
    _edgePrev = now;
    return edge;
}

void Indicator::update() {
    // --- Debounce ----------------------------------------------------------
    const bool raw = digitalRead(Pins::START_BUTTON);
    const uint32_t nowMs = millis();

    if (raw != _lastButtonRaw) {
        _lastDebounceMs = nowMs;
        _lastButtonRaw  = raw;
    } else if ((nowMs - _lastDebounceMs) > 30) {
        _debouncedState = raw;
    }

    // --- Beep timeout --------------------------------------------------------
    if (_toneEndMs != 0 && nowMs >= _toneEndMs) {
        beepOff();
    }

    // --- [Rev 3] Retry failed writes / resync after a PCA re-init -----------
    refreshOutputs();

    // --- LED pattern ---------------------------------------------------------
    if (_manualOverride) {
        // A caller owns the LED right now — the mine sequence during the dwell
        // and the scored blinks, or stepLackOfProgress(). Leave it exactly as
        // commanded until the next setStatus() call.
        return;
    }
    writeLed(patternIsOn(_status, nowMs - _patternStartMs));
}


// ============================================================================
//  TaskHandler — construction and bring-up
// ============================================================================

TaskHandler::TaskHandler(RobotDrivetrain& drive, SensorsSystem& sensors)
    : _drive(drive), _sensors(sensors) {
    _ctx = MissionContext();
    _ctx.state          = RobotState::BOOT;
    _ctx.previousState  = RobotState::BOOT;
    _ctx.stateEnteredMs = 0;
    _ctx.matchStartMs   = 0;
    _ctx.pendingTurn    = TurnDirection::NONE;
}

bool TaskHandler::begin() {
    // [Rev 3] PCA9685 before anything that writes to it. Wire is already up:
    // SensorsSystem::begin() ran Wire.begin(SDA, SCL, clock) — see main.cpp.
    // pca.begin() calls Wire.begin() with no pins, which is a no-op on a bus
    // that is already running, so it stays on GPIO9/10.
    if (!pca.begin()) {
        PcaBus::markDown();      // main.cpp reports it via peripheralsUp()
        return false;
    }
    // Calibrated oscillator first, so setPWMFreq(50) programs a true 50 Hz
    // frame and the 102..491 tick servo range lands on 500..2400 us.
    pca.setOscillatorFrequency(Hw::PCA_OSC_HZ);
    pca.setPWMFreq(50);
    if (!PcaBus::adoptConfiguration()) {
        return false;
    }
    _manipulator.begin();
    _indicator.begin();

    // [A-V1] Enter CALIBRATING BEFORE any task exists. It used to run after
    // the mission task was spawned, so the mission could tick CALIBRATING on
    // Core 0 while this call was still resetting the line extrema on Core 1.
    g_lineSamplingEnabled = false;
    transitionTo(RobotState::CALIBRATING);

    // ---- Core 1: hard real time. Neither of these may ever touch I2C. -----
    BaseType_t ok = xTaskCreatePinnedToCore(
        controlTaskEntry, "control", Rtos::STACK_CONTROL, this,
        Rtos::PRIO_CONTROL, nullptr, Rtos::CORE_CONTROL);
    if (ok != pdPASS) return false;

    ok = xTaskCreatePinnedToCore(
        lineTaskEntry, "line", Rtos::STACK_LINE, this,
        Rtos::PRIO_LINE, nullptr, Rtos::CORE_CONTROL);
    if (ok != pdPASS) return false;

    // ---- Core 0: latency tolerant -----------------------------------------
    ok = xTaskCreatePinnedToCore(
        sensorTaskEntry, "sensors", Rtos::STACK_SENSORS, this,
        Rtos::PRIO_SENSORS, nullptr, Rtos::CORE_SENSORS);
    if (ok != pdPASS) return false;

    ok = xTaskCreatePinnedToCore(
        missionTaskEntry, "mission", Rtos::STACK_MISSION, this,
        Rtos::PRIO_MISSION, nullptr, Rtos::CORE_SENSORS);
    if (ok != pdPASS) return false;

#if SYROBIX_ENABLE_TELEMETRY
    xTaskCreatePinnedToCore(
        telemetryTaskEntry, "telemetry", Rtos::STACK_TELEMETRY, this,
        Rtos::PRIO_TELEMETRY, nullptr, Rtos::CORE_SENSORS);
#endif
    return true;
}

void TaskHandler::emergencyStop() {
    // ISR-safe: volatile stores and one GPIO write. It no longer calls
    // RobotDrivetrain::emergencyStop(), whose ledcWrite() takes a mutex on
    // Arduino-ESP32 2.x and would assert inside an ISR. The control task
    // completes the stop (coast + PID reset) on its next tick.
    g_emergencyStop     = true;
    g_lineFollowEnabled = false;
    digitalWrite(Pins::MOTOR_STBY, LOW);
}

const MissionContext& TaskHandler::context() const { return _ctx; }

bool TaskHandler::peripheralsUp() const { return PcaBus::isUp(); }

bool TaskHandler::_vision_isRed(const SensorSnapshot& snap) const {
    // Both heads must agree: a single-head red is far more likely to be an
    // edge artefact than a mine tile or the full-width evacuation stripe.
    return (snap.colorLeft == TileColor::RED) && (snap.colorRight == TileColor::RED);
}

RobotState TaskHandler::navState() const {
    return _ctx.victimAcquired ? RobotState::EXIT_SEEK : RobotState::LINE_FOLLOW;
}

float TaskHandler::yawNow(const SensorSnapshot& snap) const {
    // [A-H1] Degraded fallback. Encoder heading under-reads rotation by
    // 30-50 % on a skid-steer, so turns overshoot on it — but they terminate,
    // which is better than every IMU-closed rotation hitting its watchdog.
    return snap.imuHealthy ? snap.yawDeg : _drive.headingDeg();
}

float TaskHandler::odoMm() const {
    return _drive.distanceTravelledMm();
}


// ============================================================================
//  Task trampolines
// ============================================================================

void TaskHandler::controlTaskEntry(void* pv)   { static_cast<TaskHandler*>(pv)->controlLoop();   }
void TaskHandler::lineTaskEntry(void* pv)      { static_cast<TaskHandler*>(pv)->lineLoop();      }
void TaskHandler::missionTaskEntry(void* pv)   { static_cast<TaskHandler*>(pv)->missionLoop();   }
void TaskHandler::sensorTaskEntry(void* pv)    { static_cast<TaskHandler*>(pv)->sensorLoop();    }
void TaskHandler::telemetryTaskEntry(void* pv) { static_cast<TaskHandler*>(pv)->telemetryLoop(); }


// ============================================================================
//  Core 1 — control loop, 200 Hz. THE ONLY TASK THAT WRITES THE MOTORS.
// ============================================================================

void TaskHandler::controlLoop() {
    TickType_t wake = xTaskGetTickCount();
    SensorSnapshot snap = SensorSnapshot();
    uint32_t lastMs       = millis();
    bool     wasFollowing = false;
    bool     wasLost      = true;
    uint32_t lastApplied  = 0xFFFFFFFFu;

    for (;;) {
        const uint32_t nowMs = millis();
        float dtSec = (nowMs - lastMs) / 1000.0f;
        lastMs = nowMs;
        if (dtSec <= 0.0f)  dtSec = 0.001f;
        if (dtSec > 0.100f) dtSec = 0.100f;

        if (g_emergencyStop || g_driveKill) {
            _drive.emergencyStop();
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(Rtos::PERIOD_CONTROL_MS));
            continue;
        }

        // Non-blocking; on failure reuse the previous copy.
        _sensors.getSnapshot(snap, Rtos::MUTEX_TIMEOUT_MS);

        const bool following = g_lineFollowEnabled;

        // [A-C2] Reset the line PID on the rising edge of line following and
        // on every lost->found transition. Either one otherwise pairs a stale
        // _prevError with a fresh position and kicks the derivative.
        if (following && (!wasFollowing || (wasLost && !snap.lineLost))) {
            _drive.resetLinePid();
        }
        wasFollowing = following;
        wasLost      = snap.lineLost;

        if (following) {
            _drive.followLine(snap.linePosition, g_commandedSpeed);
            lastApplied = 0xFFFFFFFFu;   // motors now differ from the mailbox
        } else {
            // [A-C1] Apply the mission's latest command. Re-applied only when
            // it changes (or after line following wrote the motors).
            const uint32_t cmd = g_driveCmd;
            if (cmd != lastApplied) {
                applyDriveCommand(_drive, cmd);
                lastApplied = cmd;
            }
        }

        _drive.update(dtSec);   // odometry integration
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(Rtos::PERIOD_CONTROL_MS));
    }
}


// ============================================================================
//  Core 1 — line array sweep, 200 Hz
// ============================================================================

void TaskHandler::lineLoop() {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        // During CALIBRATING the mission task owns LineArray (ADC + extrema).
        if (g_lineSamplingEnabled) {
            _sensors.updateLineArray();
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(Rtos::PERIOD_LINE_MS));
    }
}


// ============================================================================
//  Core 0 — slow sensors, ~33 Hz (also executes posted calibration requests)
// ============================================================================

void TaskHandler::sensorLoop() {
    TickType_t wake = xTaskGetTickCount();
    uint32_t lastMs = millis();

    for (;;) {
        const uint32_t nowMs = millis();
        float dtSec = (nowMs - lastMs) / 1000.0f;
        lastMs = nowMs;
        if (dtSec <= 0.0f)  dtSec = 0.001f;
        if (dtSec > 0.100f) dtSec = 0.100f;   // after a blocking IMU bias run

        _sensors.updateSlowSensors(dtSec);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(Rtos::PERIOD_SENSORS_MS));
    }
}


// ============================================================================
//  Core 0 — mission state machine, 50 Hz
// ============================================================================

void TaskHandler::missionLoop() {
    TickType_t wake = xTaskGetTickCount();
    SensorSnapshot snap = SensorSnapshot();
    uint32_t lastMs = millis();

    for (;;) {
        const uint32_t nowMs = millis();
        float dtSec = (nowMs - lastMs) / 1000.0f;
        lastMs = nowMs;
        if (dtSec <= 0.0f) dtSec = 0.001f;

        // [Rev 3] PCA9685 health / re-init, then its two owners. All PCA
        // traffic happens on this task (TwoWire serialises it against the
        // sensor task's IMU/ToF transactions).
        PcaBus::service(nowMs);
        _indicator.update();
        _manipulator.update(dtSec);

        if (_sensors.getSnapshot(snap, 5)) {
            tickStateMachine(snap, dtSec);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(Rtos::PERIOD_MISSION_MS));
    }
}


// ============================================================================
//  Core 0 — telemetry, 5 Hz (Rev 3: UART0 pins are free, works in match builds)
// ============================================================================

void TaskHandler::telemetryLoop() {
#if SYROBIX_ENABLE_TELEMETRY
    TickType_t wake = xTaskGetTickCount();
    SensorSnapshot snap = SensorSnapshot();

    for (;;) {
        if (_sensors.getSnapshot(snap, 10)) {
            Serial.printf(
                "[%6lu] st=%2u ph=%u line=%+.2f lost=%u fr=%4u yaw=%+7.1f imu=%u "
                "odo=%7.1f col=%u/%u turn=%u mines=%u vic=%u try=%u pca=%u score=%u\n",
                static_cast<unsigned long>(millis()),
                static_cast<unsigned>(_ctx.state),
                static_cast<unsigned>(g_phase),
                snap.linePosition, snap.lineLost ? 1u : 0u,
                snap.rangeFrontMm, snap.yawDeg, snap.imuHealthy ? 1u : 0u,
                odoMm(),
                static_cast<unsigned>(snap.colorLeft),
                static_cast<unsigned>(snap.colorRight),
                static_cast<unsigned>(_ctx.pendingTurn),
                _ctx.minesDefused, _ctx.victimAcquired ? 1u : 0u,
                _ctx.victimAttempts, PcaBus::isUp() ? 1u : 0u,
                _ctx.estimatedScore);
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(Rtos::PERIOD_TELEMETRY_MS));
    }
#else
    vTaskDelete(nullptr);
#endif
}


// ============================================================================
//  State machine plumbing
// ============================================================================

void TaskHandler::transitionTo(RobotState next) {
    if (next == _ctx.state) return;

    _ctx.previousState  = _ctx.state;
    _ctx.state          = next;
    _ctx.stateEnteredMs = millis();

    // Every per-state latch starts clean, so a re-entered sequence restarts
    // from phase 0 instead of resuming mid-way. PERSISTENT values (lockouts,
    // the red run at confirmation, the turn advance) are deliberately NOT
    // cleared: they are set immediately before a transition and read after.
    g_phase        = 0;
    g_phaseStartMs = _ctx.stateEnteredMs;
    g_blinkCount   = 0;
    g_aux          = 0;
    g_sawLost      = false;
    g_pulse        = false;
    g_mkL = g_mkR  = false;
    g_mkStartMm    = -1.0f;
    g_barMm        = -1.0f;
    g_greenStartMm = -1.0f;
    g_redStartMm   = -1.0f;
    g_redFrames    = 0;
    g_lostSinceMs  = 0;

    g_lineSamplingEnabled = (next != RobotState::CALIBRATING);
    _sensors.lidar().clearFocus();

    switch (next) {
        case RobotState::CALIBRATING:
            _indicator.setStatus(StatusColor::CALIBRATING);
            stopDriving();
            _sensors.beginLineCalibration();
            g_calColorIdx = 0;
            break;

        case RobotState::IDLE_ARMED:
            _indicator.setStatus(StatusColor::READY);
            stopDriving();
            break;

        case RobotState::LINE_FOLLOW:
            _indicator.setStatus(StatusColor::RUNNING);
            startLineFollow(Tune::SPEED_CRUISE);
            break;

        case RobotState::EXIT_SEEK:
            _indicator.setStatus(StatusColor::CARRYING);
            startLineFollow(Tune::SPEED_SLOW);
            break;

        case RobotState::RAMP_TRANSIT:
            startLineFollow(Tune::SPEED_SLOW);
            break;

        case RobotState::MINE_DEFUSE:
            // Keep moving: phase 0 line-follows onto the tile centre. The LED
            // is blanked NOW so no status pattern can masquerade as the
            // scored blink (see Config.h [FIX 2]).
            _indicator.ledOff();
            _indicator.beepOff();
            break;

        case RobotState::ZONE_ENTERED:
            _indicator.setStatus(StatusColor::ZONE);
            stopDriving();
            break;

        case RobotState::VICTIM_SCAN:
        case RobotState::VICTIM_ALIGN:
        case RobotState::VICTIM_APPROACH:
        case RobotState::VICTIM_GRIP:
            stopDriving();
            _sensors.lidar().setFocus(LidarId::FRONT);
            break;

        case RobotState::VICTIM_LIFT:
        case RobotState::TURNING:
        case RobotState::OBSTACLE_AVOID:
        case RobotState::EXIT_HOLD:
            stopDriving();
            break;

        case RobotState::LACK_OF_PROGRESS:
            _indicator.setStatus(StatusColor::ERROR);
            stopDriving();
            if (_ctx.lackOfProgressCount < 255) ++_ctx.lackOfProgressCount;
            break;

        case RobotState::FINISHED:
            _indicator.beepOff();
            _indicator.setStatus(StatusColor::OFF);
            stopDriving();
            g_driveKill = true;
            break;

        case RobotState::FAULT:
            _indicator.beepOff();
            _indicator.setStatus(StatusColor::ERROR);
            stopDriving();
            g_driveKill = true;
            break;

        default:
            break;
    }
}

uint32_t TaskHandler::timeInState() const {
    return millis() - _ctx.stateEnteredMs;
}

uint32_t TaskHandler::timeRemaining() const {
    if (_ctx.matchStartMs == 0) return Mission::MATCH_DURATION_MS;
    const uint32_t elapsed = millis() - _ctx.matchStartMs;
    return (elapsed >= Mission::MATCH_DURATION_MS)
               ? 0
               : (Mission::MATCH_DURATION_MS - elapsed);
}

void TaskHandler::retryVictim() {
    _manipulator.armToCarry();
    _manipulator.openGripper();
    if (++_ctx.victimAttempts >= Mission::MAX_VICTIM_ATTEMPTS) {
        // [A-H3] Bounded. The old SCAN <-> ALIGN / APPROACH / GRIP cycle had
        // no counter and could loop until the match clock ran out.
        transitionTo(RobotState::LACK_OF_PROGRESS);
    } else {
        transitionTo(RobotState::VICTIM_SCAN);
    }
}

void TaskHandler::tickStateMachine(const SensorSnapshot& snap, float dtSec) {
    (void)dtSec;

    if (g_emergencyStop && _ctx.state != RobotState::FINISHED) {
        _ctx.emergencyStopped = true;
        transitionTo(RobotState::FINISHED);
        return;
    }

    if (_ctx.state != RobotState::CALIBRATING &&
        _ctx.state != RobotState::IDLE_ARMED  &&
        _ctx.state != RobotState::FINISHED    &&
        _ctx.state != RobotState::FAULT       &&
        timeRemaining() == 0) {
        transitionTo(RobotState::FINISHED);
        return;
    }

    SequenceResult r = SequenceResult::RUNNING;

    switch (_ctx.state) {
        case RobotState::CALIBRATING:
            r = stepCalibrating(snap);
            if (r == SequenceResult::SUCCESS)     transitionTo(RobotState::IDLE_ARMED);
            else if (r == SequenceResult::FAILED) transitionTo(RobotState::FAULT);
            break;

        case RobotState::IDLE_ARMED:
            if (stepIdleArmed(snap) == SequenceResult::SUCCESS) {
                _ctx.matchStartMs = millis();
                transitionTo(RobotState::LINE_FOLLOW);
            }
            break;

        case RobotState::LINE_FOLLOW:
        case RobotState::EXIT_SEEK:
            stepLineFollow(snap);            // performs its own transitions
            break;

        case RobotState::TURNING:
            r = stepTurning(snap);
            if (r == SequenceResult::SUCCESS) {
                _ctx.pendingTurn = TurnDirection::NONE;
                transitionTo(navState());
            } else if (r != SequenceResult::RUNNING) {
                transitionTo(RobotState::LACK_OF_PROGRESS);
            }
            break;

        case RobotState::OBSTACLE_AVOID:
            r = stepObstacleAvoid(snap);     // may divert to ZONE_ENTERED itself
            if (r == SequenceResult::SUCCESS)      transitionTo(navState());
            else if (r != SequenceResult::RUNNING) transitionTo(RobotState::LACK_OF_PROGRESS);
            break;

        case RobotState::RAMP_TRANSIT:
            if (stepRampTransit(snap) == SequenceResult::SUCCESS) {
                transitionTo(navState());
            }
            break;

        case RobotState::ZONE_ENTERED:
            // Reached only via the safe tile (green). Arm stays at CARRY for
            // the scan [A-H5]; only the jaw opens.
            _manipulator.armToCarry();
            _manipulator.openGripper();
            _ctx.victimAttempts = 0;
            transitionTo(RobotState::VICTIM_SCAN);
            break;

        case RobotState::MINE_DEFUSE:
            r = stepDefuseMine(snap);
            if (r == SequenceResult::SUCCESS) {
                if (_ctx.minesDefused < 255) ++_ctx.minesDefused;
                addScore(5);
                if (_ctx.minesDefused == Mission::MINES_PER_ZONE) addScore(5);
                // [A-H4] Ignore red until the robot is clear of this tile.
                g_redLockoutUntilMm = odoMm() + Mission::MINE_REARM_MM;
                transitionTo(navState());
            }
            break;

        case RobotState::VICTIM_SCAN:
            r = stepVictimScan(snap);
            if (r == SequenceResult::SUCCESS)      transitionTo(RobotState::VICTIM_ALIGN);
            else if (r != SequenceResult::RUNNING) retryVictim();
            break;

        case RobotState::VICTIM_ALIGN:
            r = stepVictimAlign(snap);
            if (r == SequenceResult::SUCCESS)      transitionTo(RobotState::VICTIM_APPROACH);
            else if (r != SequenceResult::RUNNING) retryVictim();
            break;

        case RobotState::VICTIM_APPROACH:
            r = stepVictimApproach(snap);
            if (r == SequenceResult::SUCCESS)      transitionTo(RobotState::VICTIM_GRIP);
            else if (r != SequenceResult::RUNNING) retryVictim();
            break;

        case RobotState::VICTIM_GRIP:
            r = stepVictimGrip(snap);
            if (r == SequenceResult::SUCCESS)      transitionTo(RobotState::VICTIM_LIFT);
            else if (r != SequenceResult::RUNNING) retryVictim();
            break;

        case RobotState::VICTIM_LIFT:
            if (stepVictimLift(snap) == SequenceResult::SUCCESS) {
                _ctx.victimAcquired = true;
                // [A-H5] The robot is facing into the safe tile. Turn round and
                // re-find the line before EXIT_SEEK starts following it.
                _ctx.pendingTurn = TurnDirection::U_TURN;
                transitionTo(RobotState::TURNING);
            }
            break;

        case RobotState::EXIT_HOLD:
            if (stepExitHold(snap) == SequenceResult::SUCCESS) {
                _ctx.victimDelivered = true;
                addScore(20);   // 20 (out of zone, scored at the stripe) + 20 here
                transitionTo(RobotState::FINISHED);
            }
            break;

        case RobotState::LACK_OF_PROGRESS:
            if (stepLackOfProgress(snap) == SequenceResult::SUCCESS) {
                resumeFromCheckpoint();
                transitionTo(navState());
            }
            break;

        case RobotState::FINISHED:
        case RobotState::FAULT:
        case RobotState::BOOT:
        default:
            break;
    }
}


// ============================================================================
//  Startup
// ============================================================================

SequenceResult TaskHandler::stepCalibrating(const SensorSnapshot& snap) {
    (void)snap;
    const uint32_t now = millis();

    switch (g_phase) {
        case 0:
            // Line sweep: the operator moves the robot across the line.
            _sensors.sampleLineCalibration();
            if (timeInState() > CAL_SWEEP_MS) {
                _sensors.endLineCalibration();
                _indicator.beep(CAL_ACK_MS);   // "set it down"
                g_phase = 1;
                g_phaseStartMs = now;
            }
            return SequenceResult::RUNNING;

        case 1:
            // Give the operator time to let go before the gyro bias capture.
            if (now - g_phaseStartMs >= CAL_SETTLE_MS) {
                _sensors.requestImuBiasCalibration();       // runs on sensor task
                g_phase = 2;
            }
            return SequenceResult::RUNNING;

        case 2:
            if (_sensors.imuCalibrationBusy()) return SequenceResult::RUNNING;
#if SYROBIX_REQUIRE_COLOR_CAL
            g_calColorIdx  = 0;
            g_blinkCount   = 0;
            g_phaseStartMs = now;
            g_phase = 3;
#else
            g_phase = 6;
#endif
            return SequenceResult::RUNNING;

        case 3: {
            // [A-C9] Prompt: (index + 1) short beeps.
            //   1 = WHITE floor   2 = BLACK line   3 = GREEN safe tile
            //   4 = RED mine tile
            // BOTH heads must be over the surface (use the 300 mm tiles; a
            // 25 mm marker cannot cover both heads).
            const uint8_t beeps = static_cast<uint8_t>(g_calColorIdx + 1);
            const uint32_t elapsed = now - g_phaseStartMs;
            if (g_blinkCount < beeps &&
                elapsed >= static_cast<uint32_t>(g_blinkCount) * CAL_BEEP_PERIOD) {
                _indicator.beep(CAL_BEEP_MS);
                ++g_blinkCount;
            } else if (g_blinkCount >= beeps &&
                       elapsed >= static_cast<uint32_t>(beeps) * CAL_BEEP_PERIOD) {
                g_phase = 4;
            }
            return SequenceResult::RUNNING;
        }

        case 4:
            // Operator positions the heads, then presses START.
            if (_indicator.buttonRisingEdge()) {
                _sensors.requestColorReference(CAL_SEQUENCE[g_calColorIdx]);
                g_phase = 5;
            }
            return SequenceResult::RUNNING;

        case 5:
            if (_sensors.colorReferenceBusy()) return SequenceResult::RUNNING;
            _indicator.beep(CAL_ACK_MS);
            if (++g_calColorIdx >= CAL_COLORS) {
                g_phase = 6;
            } else {
                g_blinkCount   = 0;
                g_phaseStartMs = now;
                g_phase = 3;
            }
            return SequenceResult::RUNNING;

        case 6:
        default:
            return _sensors.isFullyCalibrated() ? SequenceResult::SUCCESS
                                                : SequenceResult::FAILED;
    }
}

SequenceResult TaskHandler::stepIdleArmed(const SensorSnapshot& snap) {
    (void)snap;
    return _indicator.buttonRisingEdge() ? SequenceResult::SUCCESS
                                         : SequenceResult::RUNNING;
}


// ============================================================================
//  Navigation
// ============================================================================

SequenceResult TaskHandler::stepLineFollow(const SensorSnapshot& snap) {
    const float d        = odoMm();
    const bool  carrying = _ctx.victimAcquired;
    const bool  newFrame = (snap.timestampMs != g_lastColorStamp);
    g_lastColorStamp = snap.timestampMs;

    // ---------------------------------------------------------------- RED --
    // [A-C8] Classified by run LENGTH in encoder mm:
    //   not carrying: run >= RED_TILE_CONFIRM_MM            -> mine tile
    //   carrying:     run ENDS within EXIT_STRIPE_MAX_MM     -> evacuation stripe
    //                 run reaches EXIT_MINE_MIN_RUN_MM       -> mine tile
    // A red mine tile can no longer end the match, and the thin stripe no
    // longer has to survive a 220 ms timer it was physically too narrow for.
    const bool red = _vision_isRed(snap) && (d >= g_redLockoutUntilMm);
    if (red) {
        if (g_redStartMm < 0.0f) { g_redStartMm = d; g_redFrames = 0; }
        if (newFrame && g_redFrames < 255) ++g_redFrames;
        const float run = d - g_redStartMm;
        const float mineRun = carrying ? Mission::EXIT_MINE_MIN_RUN_MM
                                       : Mission::RED_TILE_CONFIRM_MM;
        if (run >= mineRun) {
            g_redRunAtConfirm = run;
            _ctx.zoneEntered  = true;
            transitionTo(RobotState::MINE_DEFUSE);
            return SequenceResult::RUNNING;
        }
    } else if (g_redStartMm >= 0.0f) {
        const float   run    = d - g_redStartMm;
        const uint8_t frames = g_redFrames;
        g_redStartMm = -1.0f;
        g_redFrames  = 0;
        if (carrying &&
            run >= Mission::EXIT_STRIPE_MIN_MM &&
            run <= Mission::EXIT_STRIPE_MAX_MM &&
            frames >= Mission::EXIT_STRIPE_MIN_FRAMES) {
            addScore(20);   // victim is out of the zone
            transitionTo(RobotState::EXIT_HOLD);
            return SequenceResult::RUNNING;
        }
    }

    // ------------------------------------------------------ GREEN (tile) --
    const bool greenBoth = (snap.colorLeft  == TileColor::GREEN) &&
                           (snap.colorRight == TileColor::GREEN);
    if (greenBoth) {
        if (g_greenStartMm < 0.0f) g_greenStartMm = d;
        if (!carrying && (d - g_greenStartMm) >= Mission::GREEN_TILE_CONFIRM_MM) {
            _ctx.zoneEntered = true;
            transitionTo(RobotState::ZONE_ENTERED);
            return SequenceResult::RUNNING;
        }
    } else {
        g_greenStartMm = -1.0f;
    }

    // ---------------------------------------------------------- TERRAIN --
    if (snap.onRamp) {
        transitionTo(RobotState::RAMP_TRANSIT);
        return SequenceResult::RUNNING;
    }

    // --------------------------------------------------------- OBSTACLE --
    if (snap.obstacleAhead) {
        transitionTo(RobotState::OBSTACLE_AVOID);   // verifies victim first
        return SequenceResult::RUNNING;
    }

    // ---------------------------------------------------------- MARKERS --
    // [A-C5] Accumulate markers over a distance window; decide AT the bar.
    //  - a dead-end pair seen with skew (one head a frame before the other)
    //    now still reads as BOTH -> U-turn, instead of latching LEFT/RIGHT
    //    from the first frame;
    //  - markers after a bar belong to other approach directions: ignored.
    if (d >= g_greenIgnoreUntilMm) {
        if (snap.colorLeft  == TileColor::GREEN) g_mkL = true;
        if (snap.colorRight == TileColor::GREEN) g_mkR = true;
        if ((g_mkL || g_mkR) && g_mkStartMm < 0.0f) g_mkStartMm = d;
    }

    const bool atBar = snap.lineActiveCount >= Tune::BRANCH_MIN_HITS;
    if (atBar && g_barMm < 0.0f) g_barMm = d;

    // Decide once the heads have caught up with the array (geometry offset),
    // and never while both heads still sit on green (could be the safe tile).
    if (g_barMm >= 0.0f && !greenBoth &&
        (d - g_barMm) >= Hw::COLOR_HEADS_BEHIND_ARRAY_MM) {
        if (g_mkStartMm >= 0.0f) {
            _ctx.pendingTurn = (g_mkL && g_mkR) ? TurnDirection::U_TURN
                             : (g_mkL ? TurnDirection::LEFT : TurnDirection::RIGHT);
            // Pivot centre must reach the bar; part of that distance is
            // already covered if the decision was deferred.
            g_turnAdvanceMm = Hw::LINE_ARRAY_TO_PIVOT_MM - (d - g_barMm);
            if (g_turnAdvanceMm < 0.0f) g_turnAdvanceMm = 0.0f;
            transitionTo(RobotState::TURNING);
            return SequenceResult::RUNNING;
        }
        // No markers: straight through. Anything green in the next stretch
        // belongs to another approach direction.
        g_greenIgnoreUntilMm = d + Mission::MARKER_POST_BAR_IGNORE_MM;
        g_barMm = -1.0f;
    }

    // A marker with no bar after it was noise (or a bump shadow): forget it.
    if (g_mkStartMm >= 0.0f && (d - g_mkStartMm) > Mission::MARKER_WINDOW_MM) {
        g_mkL = g_mkR = false;
        g_mkStartMm = -1.0f;
    }

    // ------------------------------------------------------------- LOST --
    if (detectLackOfProgress(snap)) {
        transitionTo(RobotState::LACK_OF_PROGRESS);
    }
    return SequenceResult::RUNNING;
}

SequenceResult TaskHandler::stepTurning(const SensorSnapshot& snap) {
    const TurnDirection dir = _ctx.pendingTurn;
    if (!isRotational(dir)) {
        return SequenceResult::SUCCESS;   // STRAIGHT/NONE: nothing to do
    }

    const uint32_t now = millis();
    const bool     isU = (dir == TurnDirection::U_TURN);
    if (timeInState() > (isU ? WD_UTURN_MS : WD_TURNING_MS)) {
        return SequenceResult::TIMED_OUT;
    }

    const float yaw = yawNow(snap);

    if (isU) {
        switch (g_phase) {
            case 0:
                // Cumulative rotation, integrated from wrapped per-tick deltas:
                // a plain |wrap(yaw - start)| peaks at 180 and then FALLS, so it
                // cannot measure a half turn reliably.
                g_prevYaw   = yaw;
                g_turnAccum = 0.0f;
                drivePivot(Tune::SPEED_PIVOT, TurnDirection::RIGHT);
                g_phase = 1;
                return SequenceResult::RUNNING;

            case 1:
                g_turnAccum += wrapDeg(yaw - g_prevYaw);
                g_prevYaw    = yaw;
                if (fabsf(g_turnAccum) >= U_TURN_DONE_DEG) {
                    driveBrake();
                    g_phaseStartMs = now;
                    g_phase = 2;
                }
                return SequenceResult::RUNNING;

            case 2:
                if (now - g_phaseStartMs < DEAD_END_PAUSE_MS) return SequenceResult::RUNNING;
                g_refYaw = yaw;   // search reference
                g_phase  = 3;
                // fall through
            case 3:
                if (!snap.lineLost) { driveBrake(); return SequenceResult::SUCCESS; }
                drivePivot(Tune::SPEED_PIVOT, TurnDirection::LEFT);
                if (wrapDeg(yaw - g_refYaw) >= UTURN_SEARCH_DEG) g_phase = 4;
                return SequenceResult::RUNNING;

            case 4:
            default:
                if (!snap.lineLost) { driveBrake(); return SequenceResult::SUCCESS; }
                drivePivot(Tune::SPEED_PIVOT, TurnDirection::RIGHT);
                if (wrapDeg(yaw - g_refYaw) <= -UTURN_SEARCH_DEG) {
                    driveBrake();   // not found either side: let the WD call it
                }
                return SequenceResult::RUNNING;
        }
    }

    // --- Branch turn ----------------------------------------------------------
    switch (g_phase) {
        case 0:
            // Advance so the pivot happens ON the intersection. Distance, not
            // time: 350 ms meant a different distance at every battery level.
            g_refMm = odoMm();
            driveTank(Tune::SPEED_SLOW, Tune::SPEED_SLOW);
            g_phase = 1;
            return SequenceResult::RUNNING;

        case 1:
            if (odoMm() - g_refMm >= g_turnAdvanceMm) {
                g_refYaw  = yaw;
                g_sawLost = false;
                drivePivot(Tune::SPEED_PIVOT, dir);
                g_phase = 2;
            }
            return SequenceResult::RUNNING;

        case 2:
        default: {
            // [A-C4] At a 4-way (or a T with a through-line) the array is still
            // ON the straight branch when the pivot starts, so "line found and
            // centred" was already true on tick one and the robot went straight.
            // It must first LEAVE that line (or rotate far enough that it
            // certainly has), and never accept a branch before TURN_MIN_DEG.
            if (snap.lineLost) g_sawLost = true;
            const float turned = fabsf(wrapDeg(yaw - g_refYaw));
            if (turned >= TURN_ASSUME_LEFT_DEG) g_sawLost = true;

            if (g_sawLost && turned >= TURN_MIN_DEG &&
                !snap.lineLost && fabsf(snap.linePosition) < 0.35f) {
                driveBrake();
                return SequenceResult::SUCCESS;
            }
            return SequenceResult::RUNNING;
        }
    }
}

SequenceResult TaskHandler::stepObstacleAvoid(const SensorSnapshot& snap) {
    if (timeInState() > WD_OBSTACLE_MS) return SequenceResult::TIMED_OUT;

    const float d   = odoMm();
    const float yaw = yawNow(snap);
    const float sgn = (g_avoidDir == TurnDirection::LEFT) ? 1.0f : -1.0f;
    const bool  greenBoth = (snap.colorLeft  == TileColor::GREEN) &&
                            (snap.colorRight == TileColor::GREEN);

    switch (g_phase) {
        case 0:
            // [A-C6] Before the victim is aboard, the "obstacle" may BE the
            // victim: it stands on the safe tile and trips obstacleAhead about
            // when the heads reach the green edge — before the 60 mm green
            // confirmation. Creep and look for green first.
            if (_ctx.victimAcquired) { g_phase = 10; return SequenceResult::RUNNING; }
            g_refMm        = d;
            g_greenStartMm = -1.0f;
            driveTank(Tune::SPEED_APPROACH, Tune::SPEED_APPROACH);
            g_phase = 1;
            return SequenceResult::RUNNING;

        case 1:
            if (greenBoth) {
                if (g_greenStartMm < 0.0f) g_greenStartMm = d;
                if (d - g_greenStartMm >= Mission::OBSTACLE_VERIFY_GREEN_MM) {
                    _ctx.zoneEntered = true;
                    transitionTo(RobotState::ZONE_ENTERED);   // it's the victim
                    return SequenceResult::RUNNING;
                }
            } else {
                g_greenStartMm = -1.0f;
            }
            if ((d - g_refMm) >= Mission::OBSTACLE_VERIFY_MM ||
                (rangeValid(snap.rangeFrontMm) &&
                 snap.rangeFrontMm < Tune::TOF_GRIP_MM + 20)) {
                driveBrake();
                g_phase = 10;
            }
            return SequenceResult::RUNNING;

        case 10: {
            // [A-C7] Pick the side: right by default, left if something sits
            // close on the right and the left is clear.
            const bool rightBlocked = rangeValid(snap.rangeRightMm) &&
                                      snap.rangeRightMm < Mission::OBSTACLE_SIDE_CLEAR_MM;
            const bool leftBlocked  = rangeValid(snap.rangeLeftMm) &&
                                      snap.rangeLeftMm  < Mission::OBSTACLE_SIDE_CLEAR_MM;
            g_avoidDir = (rightBlocked && !leftBlocked) ? TurnDirection::LEFT
                                                        : TurnDirection::RIGHT;
            g_refYaw = yaw;
            drivePivot(Tune::SPEED_PIVOT, g_avoidDir);
            g_phase = 11;
            return SequenceResult::RUNNING;
        }

        case 11: {
            const float turned = ((g_avoidDir == TurnDirection::LEFT) ? 1.0f : -1.0f) *
                                 wrapDeg(yaw - g_refYaw);
            if (turned >= Mission::OBSTACLE_TURN_DEG) {
                g_refMm = d;
                driveTank(Tune::SPEED_SLOW, Tune::SPEED_SLOW);
                g_phase = 12;
            }
            return SequenceResult::RUNNING;
        }

        case 12:
            if (d - g_refMm >= Mission::OBSTACLE_LEG_MM) {
                drivePivot(Tune::SPEED_PIVOT,
                           (g_avoidDir == TurnDirection::LEFT) ? TurnDirection::RIGHT
                                                               : TurnDirection::LEFT);
                g_phase = 13;
            }
            return SequenceResult::RUNNING;

        case 13:
            // Turn back PAST parallel, so the robot converges on the line.
            if (sgn * wrapDeg(yaw - g_refYaw) <= -Mission::OBSTACLE_RETURN_DEG) {
                g_refMm = d;
                driveTank(Tune::SPEED_SLOW, Tune::SPEED_SLOW);
                g_phase = 14;
            }
            return SequenceResult::RUNNING;

        case 14:
        default:
            if (!snap.lineLost) {
                driveBrake();
                addScore(20);
                return SequenceResult::SUCCESS;
            }
            if (d - g_refMm > Mission::OBSTACLE_REJOIN_MAX_MM) {
                driveBrake();
                return SequenceResult::FAILED;
            }
            return SequenceResult::RUNNING;
    }
}

SequenceResult TaskHandler::stepRampTransit(const SensorSnapshot& snap) {
    // Line following continues at reduced speed; wait for pitch to settle.
    if (!snap.onRamp && timeInState() > 500) {
        addScore(10);
        return SequenceResult::SUCCESS;
    }
    return SequenceResult::RUNNING;
}


// ============================================================================
//  Mine defusal
// ============================================================================

SequenceResult TaskHandler::stepDefuseMine(const SensorSnapshot& snap) {
    (void)snap;
    const uint32_t now = millis();

    switch (g_phase) {
        case 0: {
            // [A-H4] Park the chassis CENTRE on the tile centre (rule 5.4: the
            // robot is on a tile only when more than half of it is inside).
            // Heads are g_redRunAtConfirm into the tile; the centre trails
            // them by COLOR_HEADS_TO_CENTER_MM.
            float advance = (Mission::TILE_SIZE_MM * 0.5f) -
                            (g_redRunAtConfirm - Hw::COLOR_HEADS_TO_CENTER_MM);
            if (advance < 0.0f) advance = 0.0f;
            if (advance > static_cast<float>(Mission::TILE_SIZE_MM)) {
                advance = static_cast<float>(Mission::TILE_SIZE_MM);
            }
            g_refMm    = odoMm();
            g_targetMm = advance;
            startLineFollow(Tune::SPEED_SLOW);   // lines cross mine tiles
            g_phase = 1;
            return SequenceResult::RUNNING;
        }

        case 1:
            if ((odoMm() - g_refMm) >= g_targetMm ||
                (now - g_phaseStartMs) > Mission::MINE_ADVANCE_TIMEOUT_MS) {
                stopDriving();
                _indicator.ledOff();
                g_phaseStartMs = now;          // dwell starts at standstill
                g_phase = 2;
            }
            return SequenceResult::RUNNING;

        case 2:   // 1. remain on the tile, LED dark
            if (now - g_phaseStartMs >= Mission::MINE_DWELL_MS) {
                _indicator.ledOn();
                g_blinkCount   = 0;
                g_phaseStartMs = now;
                g_phase = 3;
            }
            return SequenceResult::RUNNING;

        case 3:   // 2. blink: ON half
            if (now - g_phaseStartMs >= Mission::MINE_LED_ON_MS) {
                _indicator.ledOff();
                g_phaseStartMs = now;
                g_phase = 4;
            }
            return SequenceResult::RUNNING;

        case 4:   // 2. blink: OFF half
            if (now - g_phaseStartMs >= Mission::MINE_LED_OFF_MS) {
                if (++g_blinkCount >= Mission::MINE_LED_BLINKS) {
                    _indicator.beep(Mission::MINE_BUZZER_MS);
                    g_phase = 5;
                } else {
                    _indicator.ledOn();
                    g_phase = 3;
                }
                g_phaseStartMs = now;
            }
            return SequenceResult::RUNNING;

        case 5:   // 3. audible confirmation
        default:
            return (now - g_phaseStartMs >= Mission::MINE_BUZZER_MS)
                       ? SequenceResult::SUCCESS
                       : SequenceResult::RUNNING;
    }
}


// ============================================================================
//  Victim rescue
// ============================================================================

SequenceResult TaskHandler::stepVictimScan(const SensorSnapshot& snap) {
    if (timeInState() > WD_SCAN_MS) return SequenceResult::TIMED_OUT;

    const uint32_t now = millis();
    const float    yaw = yawNow(snap);

    switch (g_phase) {
        case 0:
            // [A-C3] Swing to the right edge of the arc first.
            g_scanStartYaw  = yaw;
            g_scanBestRange = Tune::TOF_INVALID;
            g_scanBestYaw   = yaw;
            drivePivot(Tune::SPEED_PIVOT, TurnDirection::RIGHT);
            g_phase = 1;
            return SequenceResult::RUNNING;

        case 1:
            if (wrapDeg(yaw - g_scanStartYaw) <= -Tune::SCAN_HALF_ARC_DEG) {
                driveBrake();
                g_brakeMs = now;
                g_phase = 2;
            }
            return SequenceResult::RUNNING;

        case 2: {
            // Stationary sample: must be a measurement that STARTED after the
            // chassis stopped rocking, so range and yaw describe one pose.
            const bool fresh = freshFrontSample(snap, g_brakeMs);
            if (!fresh && (now - g_brakeMs) < Tune::SCAN_SAMPLE_TIMEOUT_MS) {
                return SequenceResult::RUNNING;
            }
            if (fresh && rangeValid(snap.rangeFrontMm) &&
                snap.rangeFrontMm > Tune::TOF_GRIP_MM / 2 &&
                snap.rangeFrontMm < Tune::TOF_VICTIM_MAX_MM &&
                snap.rangeFrontMm < g_scanBestRange) {
                g_scanBestRange = snap.rangeFrontMm;
                g_scanBestYaw   = yaw;
            }

            const float rel = wrapDeg(yaw - g_scanStartYaw);
            if (rel >= Tune::SCAN_HALF_ARC_DEG) {
                if (g_scanBestRange >= Tune::TOF_VICTIM_MAX_MM) {
                    return SequenceResult::FAILED;
                }
                _ctx.victimFound      = true;
                _ctx.victimBearingDeg = g_scanBestYaw;
                _ctx.victimRangeMm    = g_scanBestRange;
                return SequenceResult::SUCCESS;
            }
            g_stepTarget = rel + Tune::SCAN_COARSE_STEP_DEG;
            drivePivot(Tune::SPEED_PIVOT, TurnDirection::LEFT);
            g_phase = 3;
            return SequenceResult::RUNNING;
        }

        case 3:
        default:
            // Brake early: SPEED_PIVOT carries momentum past the step.
            if (wrapDeg(yaw - g_scanStartYaw) >=
                g_stepTarget - Tune::SCAN_STEP_LEAD_DEG) {
                driveBrake();
                g_brakeMs = now;
                g_phase = 2;
            }
            return SequenceResult::RUNNING;
    }
}

SequenceResult TaskHandler::stepVictimAlign(const SensorSnapshot& snap) {
    if (timeInState() > WD_ALIGN_MS) return SequenceResult::TIMED_OUT;

    const uint32_t now = millis();
    const float    err = wrapDeg(_ctx.victimBearingDeg - yawNow(snap));

    switch (g_phase) {
        case 0:
            if (fabsf(err) <= Tune::ALIGN_TOLERANCE_DEG ||
                g_aux >= Tune::ALIGN_MAX_NUDGES) {
                driveBrake();
                g_brakeMs = now;
                g_phase = 2;                 // verify
                return SequenceResult::RUNNING;
            }
            g_alignSign    = (err > 0.0f) ? 1 : -1;
            g_pulse        = fabsf(err) < Tune::ALIGN_PULSE_BELOW_DEG;
            g_phaseStartMs = now;
            drivePivot(Tune::SPEED_PIVOT,
                       (err > 0.0f) ? TurnDirection::LEFT : TurnDirection::RIGHT);
            g_phase = 1;
            return SequenceResult::RUNNING;

        case 1: {
            const bool crossed = ((err > 0.0f) ? 1 : -1) != g_alignSign;
            const bool done = crossed ||
                (g_pulse ? (now - g_phaseStartMs) >= Tune::ALIGN_PULSE_MS
                         : fabsf(err) <= Tune::ALIGN_BRAKE_LEAD_DEG);
            if (done) {
                driveBrake();
                g_phaseStartMs = now;
                g_phase = 3;
            }
            return SequenceResult::RUNNING;
        }

        case 3:   // settle, then re-evaluate
            if (now - g_phaseStartMs >= Tune::ALIGN_SETTLE_MS) {
                ++g_aux;
                g_phase = 0;
            }
            return SequenceResult::RUNNING;

        case 2:   // verify: something must actually be ahead
        default: {
            const bool fresh = freshFrontSample(snap, g_brakeMs);
            if (!fresh && (now - g_brakeMs) < Tune::SCAN_SAMPLE_TIMEOUT_MS) {
                return SequenceResult::RUNNING;
            }
            if (fresh && rangeValid(snap.rangeFrontMm) &&
                snap.rangeFrontMm < Tune::TOF_VICTIM_MAX_MM) {
                _ctx.victimRangeMm = snap.rangeFrontMm;
                return SequenceResult::SUCCESS;
            }
            return SequenceResult::FAILED;   // bounded retry via retryVictim()
        }
    }
}

SequenceResult TaskHandler::stepVictimApproach(const SensorSnapshot& snap) {
    if (timeInState() > WD_APPROACH_MS) return SequenceResult::TIMED_OUT;

    const float    d = odoMm();
    const uint16_t r = snap.rangeFrontMm;
    const bool     v = rangeValid(r);

    switch (g_phase) {
        case 0:
            g_refMm = d;
            driveTank(Tune::SPEED_APPROACH, Tune::SPEED_APPROACH);
            g_phase = 1;
            return SequenceResult::RUNNING;

        case 1:
            // [A-H5] Lower the arm only now, close to the victim — never
            // before the scan, where a lowered jaw on a pivot knocks it over.
            if (v && r <= Tune::TOF_ARM_LOWER_MM) {
                driveBrake();
                g_rangeAtLower = r;
                _manipulator.armDown();
                _manipulator.openGripper();
                g_phaseStartMs = millis();
                g_phase = 2;
            } else if (d - g_refMm > _ctx.victimRangeMm + APPROACH_OVERSHOOT_MM) {
                driveBrake();
                return SequenceResult::FAILED;
            }
            return SequenceResult::RUNNING;

        case 2:
            if (_manipulator.isSettled() ||
                (millis() - g_phaseStartMs) > GRIP_SETTLE_TIMEOUT_MS) {
                g_refMm = d;
                driveTank(Tune::SPEED_APPROACH, Tune::SPEED_APPROACH);
                g_phase = 3;
            }
            return SequenceResult::RUNNING;

        case 3:
        default: {
            if (v && r <= Tune::TOF_GRIP_MM + GRIP_TOLERANCE_MM) {
                driveBrake();
                return SequenceResult::SUCCESS;
            }
            // Fallback if the lowered jaws occlude the ranger: close the
            // remaining gap measured at the moment the arm went down.
            const float gap = static_cast<float>(g_rangeAtLower) -
                              static_cast<float>(Tune::TOF_GRIP_MM);
            if (!v && (d - g_refMm) >= gap) {
                driveBrake();
                return SequenceResult::SUCCESS;
            }
            if ((d - g_refMm) > gap + APPROACH_OVERSHOOT_MM) {
                driveBrake();
                return SequenceResult::FAILED;
            }
            return SequenceResult::RUNNING;
        }
    }
}

SequenceResult TaskHandler::stepVictimGrip(const SensorSnapshot& snap) {
    switch (g_phase) {
        case 0:
            _manipulator.closeGripper();
            g_phase = 1;
            g_phaseStartMs = millis();
            return SequenceResult::RUNNING;

        case 1:
            if (!_manipulator.isSettled()) {
                return ((millis() - g_phaseStartMs) > GRIP_SETTLE_TIMEOUT_MS)
                           ? SequenceResult::FAILED : SequenceResult::RUNNING;
            }
            g_phase = 2;
            g_phaseStartMs = millis();
            return SequenceResult::RUNNING;

        case 2:
        default:
            if ((millis() - g_phaseStartMs) < Hw::SERVO_SETTLE_MS) {
                return SequenceResult::RUNNING;
            }
            if (_manipulator.verifyGrasp(snap.rangeFrontMm)) {
                return SequenceResult::SUCCESS;
            }
            return SequenceResult::FAILED;   // retryVictim() reopens the jaw
    }
}

SequenceResult TaskHandler::stepVictimLift(const SensorSnapshot& snap) {
    (void)snap;

    if (g_phase == 0) {
        _manipulator.armToCarry();
        g_phase = 1;
        g_phaseStartMs = millis();
        return SequenceResult::RUNNING;
    }
    if (_manipulator.isSettled() || (millis() - g_phaseStartMs) > 3000) {
        return SequenceResult::SUCCESS;
    }
    return SequenceResult::RUNNING;
}

SequenceResult TaskHandler::stepExitHold(const SensorSnapshot& snap) {
    (void)snap;
    // Brake was posted on entry and the mailbox keeps it applied.
    return (timeInState() >= Mission::EXIT_HOLD_MS) ? SequenceResult::SUCCESS
                                                    : SequenceResult::RUNNING;
}


// ============================================================================
//  Recovery
// ============================================================================

bool TaskHandler::detectLackOfProgress(const SensorSnapshot& snap) {
    // [A-M1] Distance-based: a gap tile at SPEED_SLOW is no longer "lost".
    // The stall timer only catches a robot that lost the line and stopped.
    if (!snap.lineLost) {
        g_lostSinceMs = 0;
        return false;
    }
    const uint32_t now = millis();
    const float    d   = odoMm();
    if (g_lostSinceMs == 0) {
        g_lostSinceMs = now;
        g_lostSinceMm = d;
        return false;
    }
    if (fabsf(d - g_lostSinceMm) > Tune::LINE_LOST_MAX_MM ||
        (now - g_lostSinceMs) > Tune::LINE_LOST_STALL_MS) {
        g_lostSinceMs = 0;
        return true;
    }
    return false;
}

SequenceResult TaskHandler::stepLackOfProgress(const SensorSnapshot& snap) {
    (void)snap;
    // Halted (brake posted on entry). Manual 400/400 ms blink.
    if ((timeInState() / 400) % 2 == 0) {
        _indicator.ledOn();
    } else {
        _indicator.ledOff();
    }
    return _indicator.buttonRisingEdge() ? SequenceResult::SUCCESS
                                         : SequenceResult::RUNNING;
}

void TaskHandler::resumeFromCheckpoint() {
    stopDriving();

    _ctx.pendingTurn    = TurnDirection::NONE;
    _ctx.zoneEntered    = false;
    _ctx.victimAttempts = 0;

    // Fresh placement: no lockouts carried over from the previous attempt.
    g_redLockoutUntilMm  = -1.0e9f;
    g_greenIgnoreUntilMm = -1.0e9f;

    // A victim dropped inside the zone is repositioned by the referee.
    // @note Rule question for the judges' briefing: if the robot is still
    //       GRIPPING the victim when lack of progress is called outside the
    //       zone, is it replaced with the victim? If yes, keep
    //       victimAcquired and do not open the jaw here.
    if (!_ctx.victimDelivered) {
        _ctx.victimFound    = false;
        _ctx.victimAcquired = false;
        _manipulator.armToCarry();
        _manipulator.openGripper();
    }
}


// ============================================================================
//  Scoring model — diagnostics only, never a control input
// ============================================================================

void TaskHandler::addScore(uint16_t points) {
    _ctx.estimatedScore = static_cast<uint16_t>(_ctx.estimatedScore + points);
}

uint16_t TaskHandler::estimateScore() const {
    return _ctx.estimatedScore;
}
