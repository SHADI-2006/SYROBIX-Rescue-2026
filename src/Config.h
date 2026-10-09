/**
 * ============================================================================
 *  @file       Config.h
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Single source of truth for pin mapping, physical constants,
 *              tuning parameters, mission constants and RTOS configuration.
 *
 *  @note       NOTHING in this file allocates memory or emits code. It is a
 *              pure compile-time configuration surface. Every magic number in
 *              the codebase MUST originate here.
 *
 *  ---------------------------------------------------------------------------
 *  REVISION 3 — PCA9685 PERIPHERAL BUS
 *  ---------------------------------------------------------------------------
 *    Servos, the status LED and an ACTIVE buzzer moved off native GPIO onto an
 *    Adafruit PCA9685 (I2C 0x40, shared GPIO9/10 bus). ESP32Servo is gone, and
 *    so is every servo/buzzer LEDC channel, timer reservation and static_assert
 *    that existed only to keep the buzzer off ESP32Servo's timers.
 *      - Pins::SERVO_ARM/SERVO_GRIPPER (19/20) and BUZZER/STATUS_LED (43/44)
 *        are DELETED. Those four GPIOs are now free (see Pins:: SPARE note).
 *      - New: namespace PcaChannels, Hw::PCA9685_* and Hw::SERVO_* constants.
 *      - KEPT: Hw::LEDC_CH_MOTOR_LEFT/RIGHT. The TB6612 motor PWM is still
 *        native LEDC at 20 kHz; a PCA9685 tops out near 1.5 kHz and would put
 *        the 200 Hz control loop's output on a shared I2C bus.
 *      - Mission::MINE_BUZZER_HZ removed: an active buzzer has a fixed pitch.
 *
 *  ---------------------------------------------------------------------------
 *  AUDIT REVISION 2 (VL53L1X + championship audit) — what changed here
 *  ---------------------------------------------------------------------------
 *    [A-C2] LINE_KD 1400 -> 12. Td = KD/KP was 5.4 s; it is now ~46 ms.
 *           LINE_MAX_REVERSE_PWM caps how hard the PID may reverse a wheel.
 *    [A-C5] Marker/tile confirmations are DISTANCE-based (encoder mm), not
 *           time-based, so they no longer drift with speed.
 *    [A-C8] EXIT_STRIPE_MAX_MM / EXIT_MINE_MIN_RUN_MM separate the thin red
 *           evacuation stripe from a red mine tile by red run LENGTH.
 *    [A-H4] Mine centring and re-arm distances.
 *    [A-C3] Stop-and-go victim scan constants (the old SCAN_* were unused).
 *    New Hw:: geometry constants (MEASURE THEM): LINE_ARRAY_TO_PIVOT_MM,
 *           COLOR_HEADS_TO_CENTER_MM, COLOR_HEADS_BEHIND_ARRAY_MM, IMU_YAW_SIGN.
 *
 *  ---------------------------------------------------------------------------
 *  POST-REVIEW REVISION 1 — earlier changes
 *  ---------------------------------------------------------------------------
 *    [FIX 2] Tune::LED_PATTERN_ZONE_* was bit-for-bit identical to the scored
 *            mine blink (250 ms on / 250 ms off). A judge could not tell the
 *            dwell-phase status blinking from the three scored blinks. The ZONE
 *            pattern is now a long slow pulse, and a static_assert below makes
 *            a future collision a compile error.
 *    [FIX 6] (superseded by Rev 3: servos and buzzer moved to the PCA9685,
 *            so the LEDC timer-sharing hazard no longer exists.)
 *    [FIX 3] (superseded by A-H4: MINE_REARM_MM) the mine sequence now drives off the tile
 *            before returning to LINE_FOLLOW, so the same mine is not defused
 *            forever.
 *    [FIX 9] Tile-confirmation windows moved here from TaskHandler.cpp, plus
 *            EXIT_SEEK_MIN_MS.
 * ---------------------------------------------------------------------------
 *  PIN BUDGET WARNING
 *  ---------------------------------------------------------------------------
 *  The declared hardware set now requires 29 GPIOs (Rev 3 moved 4 outputs onto
 *  the PCA9685). The ESP32-S3 exposes a usable set of GPIO 0-21 and 38-48
 *  (GPIO 26-37 are reserved by SPI flash / octal PSRAM on N16R8-class modules
 *  and MUST NOT be used).
 *
 *  Two deliberate hardware-level compromises were made to fit the budget:
 *    1. PWM and DIRECTION lines are SHARED PER SIDE. Both TB6612FNG drivers
 *       receive the same left/right control signals, so the front and rear
 *       motor of a side are electrically locked together. This is correct for
 *       skid-steer kinematics and costs no capability.
 *    2. Wheel encoders are populated on ONE motor per side only. In skid-steer
 *       the co-side motors are rigidly coupled through the ground, so a second
 *       encoder per side would be redundant instrumentation.
 *
 *  Rev 3 freed GPIO 19, 20, 43 and 44 (see Pins:: SPARE note). The first new
 *  input should be a gripper contact switch: grasp is still only INFERRED.
 * ============================================================================
 */

#pragma once

#include <stdint.h>

// ============================================================================
//  SECTION 1 — PIN MAP
// ============================================================================

namespace Pins {

// ---------------------------------------------------------------------------
//  1.1 Line sensor array — 8x TCR5000 (analog), split 6 FRONT + 2 REAR
//  MUST live on ADC1. ADC2 is unusable while Wi-Fi is active on ESP32-S3.
//  GPIO1..GPIO8 == ADC1_CH0..ADC1_CH7.
//
//  Indices 0-5 are the FRONT row (index 0 = leftmost) and are the only
//  channels that feed LineArray::readPosition()'s weighted-centroid PID.
//  Indices 6-7 are a separate REAR pair, physically on the same ADC1 bus and
//  sampled every tick like any other channel, but never part of the front
//  centroid — see LineArray::isRearOnLine() / rawRearLeft() / rawRearRight().
// ---------------------------------------------------------------------------
static constexpr uint8_t LINE_SENSOR[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

// ---------------------------------------------------------------------------
//  1.2 Shared I2C bus (MPU6500 + 3x VL53L1X)
// ---------------------------------------------------------------------------
static constexpr uint8_t I2C_SDA = 9;
static constexpr uint8_t I2C_SCL = 10;

// ---------------------------------------------------------------------------
//  1.3 Drivetrain — 2x TB6612FNG, control lines ganged per side
//      Driver #1 -> FRONT axle (ch.A = front-left, ch.B = front-right)
//      Driver #2 -> REAR  axle (ch.A = rear-left,  ch.B = rear-right)
//      Both drivers share STBY.
// ---------------------------------------------------------------------------
static constexpr uint8_t MOTOR_PWM_LEFT   = 11;  // -> D1.PWMA + D2.PWMA
static constexpr uint8_t MOTOR_PWM_RIGHT  = 12;  // -> D1.PWMB + D2.PWMB
static constexpr uint8_t MOTOR_LEFT_IN1   = 13;  // -> D1.AIN1 + D2.AIN1
static constexpr uint8_t MOTOR_LEFT_IN2   = 14;  // -> D1.AIN2 + D2.AIN2
static constexpr uint8_t MOTOR_RIGHT_IN1  = 15;  // -> D1.BIN1 + D2.BIN1
static constexpr uint8_t MOTOR_RIGHT_IN2  = 16;  // -> D1.BIN2 + D2.BIN2
static constexpr uint8_t MOTOR_STBY       = 17;  // LOW = both drivers coasting

// ---------------------------------------------------------------------------
//  1.4 Quadrature encoders (one instrumented motor per side)
//      All four pins must be interrupt-capable — on ESP32-S3 every GPIO is.
// ---------------------------------------------------------------------------
static constexpr uint8_t ENCODER_LEFT_A   = 18;
static constexpr uint8_t ENCODER_LEFT_B   = 21;
static constexpr uint8_t ENCODER_RIGHT_A  = 38;
static constexpr uint8_t ENCODER_RIGHT_B  = 39;

// ---------------------------------------------------------------------------
//  1.5 VL53L1X XSHUT lines
//      All three sensors boot on the same 0x29 address. Boot sequence holds
//      every XSHUT LOW, then RELEASES them one at a time to re-address.
//      See LidarBank::begin() and SYROBIX_TOF_XSHUT_OPEN_DRAIN below.
// ---------------------------------------------------------------------------
static constexpr uint8_t TOF_XSHUT_FRONT  = 40;
static constexpr uint8_t TOF_XSHUT_LEFT   = 41;
static constexpr uint8_t TOF_XSHUT_RIGHT  = 42;

// ---------------------------------------------------------------------------
//  1.6 Colour sensors — 2x TCS3200
//      S0/S1 (frequency scaling) are strapped to 20% in hardware to save I/O.
//      S2/S3 (photodiode filter select) are shared; only OUT is per-sensor,
//      which is safe because the two sensors are always sampled in lockstep.
//
//  @warning GPIO45 and GPIO46 are STRAPPING pins (VDD_SPI select and boot
//           mode). Driving them as outputs after boot is fine, but nothing
//           may hold them at the wrong level THROUGH RESET. Keep any external
//           pull weak and on the correct side, or the module will not boot.
// ---------------------------------------------------------------------------
static constexpr uint8_t COLOR_S2         = 45;  // shared (strapping pin)
static constexpr uint8_t COLOR_S3         = 46;  // shared (strapping pin)
static constexpr uint8_t COLOR_OUT_LEFT   = 47;
static constexpr uint8_t COLOR_OUT_RIGHT  = 48;

// ---------------------------------------------------------------------------
//  1.7 Manipulator, status LED, buzzer -> PCA9685 (see namespace PcaChannels)
//      [Rev 3] No native GPIO any more. Both servos, the status LED and the
//      active buzzer are PCA9685 outputs on the shared I2C bus (1.2).
//
//  SPARE GPIO freed by Rev 3 — currently UNUSED, do not reassign casually:
//    GPIO19 / GPIO20 : native USB D- / D+. Leave free and the native USB port
//                      works for flashing and serial (USB CDC On Boot).
//    GPIO43 / GPIO44 : UART0 TX / RX, wired to the DevKit's USB-UART bridge.
//                      Leave free and Serial telemetry works in match builds.
//  If a new input is needed (a gripper contact switch is the obvious one),
//  prefer a different free pin and keep these four as the debug ports.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//  1.8 Human interface
//      START_BUTTON sits on GPIO0, a strapping pin: it is pulled HIGH by an
//      external resistor and must NOT be held down through reset or the MCU
//      enters download mode. It stays on native GPIO: the PCA9685 is
//      output-only.
// ---------------------------------------------------------------------------
static constexpr uint8_t START_BUTTON     = 0;   // active LOW

} // namespace Pins


// ============================================================================
//  SECTION 1b — PCA9685 CHANNEL MAP  [Rev 3]
// ============================================================================
//
//  16-channel, 12-bit PWM driver at Hw::PCA9685_ADDR on the shared I2C bus.
//  Each output on the Adafruit breakout has a 220 R series resistor.
//
//  @warning ACTIVE BUZZER (channel 3): the PCA9685 sources at most ~10 mA
//           through that 220 R, at the 3.3 V logic level. A typical active
//           buzzer wants 20-30 mA at 5 V, so wired directly it will be faint
//           or silent. Drive it through a low-side switch: channel 3 -> 1 k
//           -> NPN base (2N2222 / S8050) or an AO3400 gate, buzzer between the
//           5 V rail and the collector/drain. The firmware is the same either
//           way (full ON = buzzing).
//  @warning SERVO POWER is the PCA's V+ terminal, NOT VCC. VCC is 3.3 V logic
//           from the ESP32. V+ needs its own 5-6 V UBEC rated for the MG996R
//           stall current (~2.5 A) plus a 470-1000 uF low-ESR cap right at the
//           terminal. Grounds common. This is the separation that prevents a
//           repeat of the servo-spike burn-out.
//  @note    The PCA9685 also answers its ALL-CALL address (0x70) by default.
//           Nothing else on this bus may use 0x40 or 0x70; static_asserts
//           below enforce it for the ToF and IMU addresses.
// ============================================================================

namespace PcaChannels {

static constexpr uint8_t SERVO_ARM      = 0;   // MG996R — lift
static constexpr uint8_t SERVO_GRIPPER  = 1;   // MG90S  — jaw
static constexpr uint8_t STATUS_LED     = 2;   // single-colour LED
static constexpr uint8_t BUZZER         = 3;   // ACTIVE buzzer (on/off only)

} // namespace PcaChannels

static_assert(PcaChannels::SERVO_ARM     < 16 && PcaChannels::SERVO_GRIPPER < 16 &&
              PcaChannels::STATUS_LED    < 16 && PcaChannels::BUZZER        < 16,
              "PCA9685 has 16 channels (0-15).");
static_assert(PcaChannels::SERVO_ARM  != PcaChannels::SERVO_GRIPPER &&
              PcaChannels::SERVO_ARM  != PcaChannels::STATUS_LED &&
              PcaChannels::SERVO_ARM  != PcaChannels::BUZZER &&
              PcaChannels::SERVO_GRIPPER != PcaChannels::STATUS_LED &&
              PcaChannels::SERVO_GRIPPER != PcaChannels::BUZZER &&
              PcaChannels::STATUS_LED != PcaChannels::BUZZER,
              "Two functions share a PCA9685 channel.");


// ============================================================================
//  SECTION 2 — HARDWARE CONSTANTS
// ============================================================================

namespace Hw {

// --- Chassis geometry (measure and correct after assembly) -----------------
static constexpr float WHEEL_DIAMETER_MM   = 65.0f;
static constexpr float TRACK_WIDTH_MM      = 150.0f;  // left-right wheel centres
static constexpr float ENCODER_CPR         = 1440.0f; // counts/rev AFTER gearbox,
                                                      // quadrature x4 decoded

/** Millimetres of arc travelled per encoder count. */
static constexpr float MM_PER_COUNT =
    (3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_CPR;

// --- Sensor geometry (MEASURE ON THE REAL CHASSIS, then correct) ----------
/**
 * Forward distance from the FRONT line array to the chassis pivot centre
 * (midway between the axles for a 4WD skid-steer). A branch turn drives this
 * far past the moment the array meets the bar, so the pivot happens ON the
 * intersection instead of short of it.
 */
static constexpr float LINE_ARRAY_TO_PIVOT_MM      = 70.0f;
/**
 * Distance the colour heads sit BEHIND the chassis centre is negative; this
 * is the distance from the colour heads BACK to the chassis centre (positive
 * when the heads are forward of centre). Used to park the chassis centre on
 * the middle of a mine tile (rule 5.4: >half the robot inside the tile).
 */
static constexpr float COLOR_HEADS_TO_CENTER_MM    = 80.0f;
/**
 * How far the colour heads trail the FRONT line array. 0 when the heads are
 * level with or ahead of the array. If they trail it, the array meets the
 * intersection bar BEFORE the heads have passed the markers, so the turn
 * decision is deferred by this much travel.
 */
static constexpr float COLOR_HEADS_BEHIND_ARRAY_MM = 0.0f;
/**
 * +1 if the MPU6500 reports a LEFT (counter-clockwise, seen from above) pivot
 * as positive yaw; -1 if the board is mounted upside down. Every heading
 * decision in TaskHandler assumes positive yaw == turned LEFT. Verify on the
 * bench: pivot left by hand and watch yaw in telemetry.
 */
static constexpr float IMU_YAW_SIGN                = 1.0f;

// --- Motor PWM (LEDC) ------------------------------------------------------
static constexpr uint32_t PWM_FREQ_HZ      = 20000;   // above audible range
static constexpr uint8_t  PWM_RESOLUTION   = 10;      // bits
static constexpr int16_t  PWM_MAX          = 1023;

/**
 * Static-friction (stiction) floor: any commanded PWM whose magnitude is
 * nonzero but below this is boosted up to it (see MotorSide::setPwm()).
 *
 * Raised from 180 to 220 for the 4x JGA25-370 / 65 mm rubber-tread wheel
 * chassis. A 4-wheel skid-steer drivetrain scrubs all four contact patches
 * sideways on every zero-radius pivot, and a standard tread wheel presents
 * far more sideways grip than an omni/mecanum roller would on the same
 * surface — so the previous floor, tuned assuming a lower-friction wheel,
 * left a wider "buzzing but not turning" dead zone on this hardware.
 *
 * @warning This is a bench-informed starting point, not a measured constant
 *          — validate on the actual competition tile (a stationary chassis
 *          commanded to pivot should visibly rotate, not just hum) and
 *          retune in ~20-count steps: raise it if the motors buzz without
 *          moving, lower it if small line-PID corrections now snap instead
 *          of easing in.
 * @warning ANY commanded magnitude between 1 and this value is silently
 *          rewritten to this value. Every differential-drive constant in the
 *          firmware must therefore stay at or above it, or the differential
 *          you wrote is not the differential the wheels get.
 * @invariant Must stay below Tune::SPEED_APPROACH (enforced by the
 *            static_assert below Tune::), or the floor will silently
 *            override the final victim-approach speed and the range-
 *            terminated grip in TaskHandler::stepVictimApproach() will
 *            close faster than TOF_GRIP_MM can be caught cleanly.
 */
static constexpr int16_t  PWM_MIN_MOVE     = 220;

// ---------------------------------------------------------------------------
//  LEDC — MOTOR PWM ONLY  [Rev 3]
//
//  The two TB6612 PWM lines are the only native LEDC users left. Both sit on
//  channels 0/1, i.e. LEDC timer 0, at Hw::PWM_FREQ_HZ. The servo timers and
//  the buzzer channel (and the static_asserts that kept them apart) are gone
//  with ESP32Servo. RobotDrivetrain addresses these by channel on core 2.x
//  and by pin on core 3.x.
// ---------------------------------------------------------------------------
static constexpr uint8_t LEDC_CH_MOTOR_LEFT    = 0;
static constexpr uint8_t LEDC_CH_MOTOR_RIGHT   = 1;

// --- I2C -------------------------------------------------------------------
static constexpr uint32_t I2C_CLOCK_HZ     = 400000;
static constexpr uint8_t  MPU6500_ADDR     = 0x68;
static constexpr uint8_t  TOF_ADDR_FRONT   = 0x30;
static constexpr uint8_t  TOF_ADDR_LEFT    = 0x31;
static constexpr uint8_t  TOF_ADDR_RIGHT   = 0x32;
static constexpr uint8_t  PCA9685_ADDR     = 0x40;   // [Rev 3] A0-A5 jumpers open
static constexpr uint8_t  PCA9685_ALLCALL  = 0x70;   // power-on ALL-CALL address

// --- PCA9685 [Rev 3] ---------------------------------------------------------
/**
 * Internal oscillator frequency. The datasheet says 25 MHz but real parts
 * run anywhere from ~23 to ~27 MHz, and every servo pulse width scales with
 * the error. 27 MHz is Adafruit's typical measured value. To calibrate:
 * command SERVO_MIN_US on a channel, measure the pulse with a scope or logic
 * analyser, and set PCA_OSC_HZ = 27e6 * (measured_us / commanded_us).
 */
static constexpr uint32_t PCA_OSC_HZ       = 27000000UL;
static constexpr float    SERVO_FREQ_HZ    = 50.0f;      // standard analog servo
static constexpr uint16_t SERVO_MIN_US     = 500;        // pulse at   0 deg
static constexpr uint16_t SERVO_MAX_US     = 2400;       // pulse at 180 deg
static constexpr uint8_t  SERVO_MAX_ANGLE  = 180;
/** Health check cadence: MODE1 / PRESCALE are read back and the chip is
 *  re-initialised if it has reset (brown-out on a servo spike sets SLEEP and
 *  restores the 200 Hz default prescaler, which silently stops every output). */
static constexpr uint32_t PCA_HEALTH_PERIOD_MS = 1000;
/** Consecutive failed health reads before the PCA is reported down. */
static constexpr uint8_t  PCA_FAIL_STREAK  = 3;

// --- Servo travel limits (degrees; clamp hard, MG996R will stall happily) ---
static constexpr uint8_t ARM_ANGLE_DOWN    = 10;
static constexpr uint8_t ARM_ANGLE_CARRY   = 75;   // yields ~40 mm ground clearance
static constexpr uint8_t GRIPPER_OPEN      = 100;
static constexpr uint8_t GRIPPER_CLOSED    = 25;
static constexpr uint16_t SERVO_SETTLE_MS  = 400;

} // namespace Hw

static_assert(Hw::LEDC_CH_MOTOR_LEFT != Hw::LEDC_CH_MOTOR_RIGHT,
              "The two motor sides need separate LEDC channels.");
static_assert(Hw::SERVO_MIN_US < Hw::SERVO_MAX_US && Hw::SERVO_MAX_US < 20000,
              "Servo pulse range must be increasing and fit in one 50 Hz frame.");
static_assert(Hw::ARM_ANGLE_DOWN  <= Hw::SERVO_MAX_ANGLE &&
              Hw::ARM_ANGLE_CARRY <= Hw::SERVO_MAX_ANGLE &&
              Hw::GRIPPER_OPEN    <= Hw::SERVO_MAX_ANGLE &&
              Hw::GRIPPER_CLOSED  <= Hw::SERVO_MAX_ANGLE,
              "A servo pose is outside 0..SERVO_MAX_ANGLE.");
static_assert(Hw::PCA9685_ADDR != Hw::MPU6500_ADDR &&
              Hw::PCA9685_ADDR != Hw::TOF_ADDR_FRONT &&
              Hw::PCA9685_ADDR != Hw::TOF_ADDR_LEFT &&
              Hw::PCA9685_ADDR != Hw::TOF_ADDR_RIGHT,
              "An I2C device collides with the PCA9685 address.");
static_assert(Hw::PCA9685_ALLCALL != Hw::MPU6500_ADDR &&
              Hw::PCA9685_ALLCALL != Hw::TOF_ADDR_FRONT &&
              Hw::PCA9685_ALLCALL != Hw::TOF_ADDR_LEFT &&
              Hw::PCA9685_ALLCALL != Hw::TOF_ADDR_RIGHT,
              "An I2C device collides with the PCA9685 ALL-CALL address 0x70.");


// ============================================================================
//  SECTION 3 — CONTROL TUNING
// ============================================================================

namespace Tune {

// --- Line-following PID (error := normalised lateral position, -1.0 .. +1.0)
static constexpr float LINE_KP             = 260.0f;
static constexpr float LINE_KI             = 0.0f;    // integral invites windup
                                                      // on a discontinuous line
/**
 * [A-C2] Derivative gain, per SECOND (the PID divides by the measured dt).
 *
 * The previous 1400 gave a derivative time Td = KD/KP = 5.4 s, against a
 * normal 30-100 ms for a line follower. The D term then saturated the output
 * at any error rate above 1023/1400 = 0.73 /s — i.e. any chassis rotation
 * faster than ~12 deg/s — turning the controller into a +/-1023 bang-bang.
 * LINE_D_FILTER_ALPHA cannot fix that: a first-order IIR with alpha 0.25 at
 * 200 Hz has tau ~17 ms (fc ~9 Hz). It removes noise; it does not shrink a
 * sustained derivative.
 *
 * 12 gives Td ~46 ms. Tuning procedure: KD = 0, raise KP until the robot
 * oscillates gently on a straight, back KP off ~20 %, then raise KD in steps
 * of 4 until corner overshoot stops. Do not exceed Td ~ 0.15 s (KD ~ 40).
 */
static constexpr float LINE_KD             = 12.0f;
static constexpr float LINE_D_FILTER_ALPHA = 0.25f;   // low-pass on derivative
/**
 * [A-C2] The line PID may drive the inner wheel backwards at most this hard.
 * Without it a saturated correction put one side at full reverse (-1023).
 */
static constexpr int16_t LINE_MAX_REVERSE_PWM = 300;

// --- Heading PID (error := degrees, used for in-place rotation) ------------
// [A-C1] HEADING_KP/KI/KD/TOLERANCE removed with the dead closed-loop
// rotation API. Rotations are closed on IMU yaw in TaskHandler.

// --- Speed setpoints (PWM counts) -----------------------------------------
static constexpr int16_t SPEED_CRUISE      = 620;
static constexpr int16_t SPEED_SLOW        = 380;   // ramps, obstacles, tiles
static constexpr int16_t SPEED_APPROACH    = 260;   // final victim approach

/**
 * Zero-radius pivot effort (PWM counts), and the heading-PID's output clamp.
 *
 * Raised from 320 to 420 for the same reason as Hw::PWM_MIN_MOVE above: four
 * standard-tread wheels all scrubbing sideways at once resist rotation far
 * more than a 2-driven-wheel or omni/caster layout, and the old value was
 * tuned before the wheel hardware was finalised. This is the effort handed
 * to every pivot: TURNING, OBSTACLE_AVOID, U-turns, and the stop-and-go
 * victim scan/align (which used to pivot at 240 and stalled).
 *
 * @warning Bench-informed starting point — validate on the actual tile
 *          surface. A pivot that visibly stalls or judders needs this
 *          raised further. Rotations brake EARLY (SCAN_STEP_LEAD_DEG,
 *          ALIGN_BRAKE_LEAD_DEG) to absorb this effort's momentum.
 */
static constexpr int16_t SPEED_PIVOT       = 420;

// --- Line array thresholds -------------------------------------------------
static constexpr uint16_t LINE_ON_THRESHOLD    = 600;  // post-normalisation, 0-1000

/**
 * Minimum number of the 6 FRONT line sensors that must read dark for
 * LineArray::isIntersectionCandidate() to fire.
 *
 * The array is now 6 sensors wide in front (2 more are a separate rear pair
 * — see Pins::LINE_SENSOR), not 8. The old value of 6 required 6-of-8
 * (~75%) full-width coverage; 5-of-6 (~83%) is the closest achievable
 * equivalent and errs slightly stricter, which is the safe direction for a
 * signal that is only a hypothesis until the colour sensors corroborate it
 * (see LineArray::isIntersectionCandidate()'s own note).
 */
static constexpr uint8_t  INTERSECTION_MIN_HITS = 5;   // of the 6 FRONT sensors dark
/**
 * [A-M1] Line-lost -> lack-of-progress, by DISTANCE. A 100 mm gap at
 * SPEED_SLOW took ~625 ms, which the old 450 ms timer called a lost line.
 * LINE_LOST_STALL_MS only catches a robot that lost the line AND stopped.
 */
static constexpr float    LINE_LOST_MAX_MM      = 150.0f;
static constexpr uint32_t LINE_LOST_STALL_MS    = 1500;
/**
 * [A-C5] Front sensors dark for "the array is on an intersection bar or a
 * side branch". A 90-deg side branch darkens about half the 6-sensor row
 * plus the through-line sensor, so 4 catches branches the 5-hit
 * isIntersectionCandidate() would miss.
 */
static constexpr uint8_t  BRANCH_MIN_HITS       = 4;

// --- Time-of-flight decision distances (mm) -------------------------------
static constexpr uint16_t TOF_OBSTACLE_MM      = 120;  // front, triggers avoid
static constexpr uint16_t TOF_VICTIM_MAX_MM    = 320;  // scan window ceiling
static constexpr uint16_t TOF_GRIP_MM          = 45;   // stop-and-close distance
/** [A-H5] Arm is lowered only once the victim is this close — never before
 *  the scan, where a lowered jaw swept by a pivot knocks the victim over.
 *  @warning If your front ToF is OCCLUDED by the lowered jaws, the approach
 *           falls back to odometry for the last leg (see stepVictimApproach). */
static constexpr uint16_t TOF_ARM_LOWER_MM     = 110;
static constexpr uint16_t TOF_INVALID          = 8190; // sentinel: "no valid range".
                                                       // VL53L1X never reports >4000,
                                                       // so this cannot collide.

// --- Victim scan: STOP-AND-GO chassis sweep [A-C3] -------------------------
//  Pivot a step at SPEED_PIVOT, brake, wait for a ToF sample that STARTED
//  after the chassis settled, record (range, yaw), repeat. Continuous pivoting
//  at 240 PWM stalled on 4 scrubbing wheels, and even at 420 the yaw and range
//  in one snapshot were not time-aligned. See TaskHandler::stepVictimScan().
static constexpr float    SCAN_HALF_ARC_DEG      = 70.0f;  // sweep -70..+70
static constexpr float    SCAN_COARSE_STEP_DEG   = 8.0f;
static constexpr float    SCAN_STEP_LEAD_DEG     = 3.0f;   // brake early: momentum
static constexpr uint16_t SCAN_SETTLE_MS         = 40;     // chassis rock-out
static constexpr uint16_t SCAN_SAMPLE_TIMEOUT_MS = 400;

// --- Victim alignment [A-C3] -------------------------------------------------
static constexpr float    ALIGN_TOLERANCE_DEG    = 3.0f;
static constexpr float    ALIGN_BRAKE_LEAD_DEG   = 6.0f;   // continuous-pivot mode
static constexpr float    ALIGN_PULSE_BELOW_DEG  = 12.0f;  // below: pulse mode
static constexpr uint16_t ALIGN_PULSE_MS         = 45;
static constexpr uint16_t ALIGN_SETTLE_MS        = 150;
static constexpr uint8_t  ALIGN_MAX_NUDGES       = 8;

// --- Status LED blink patterns (ms) -----------------------------------------
//  Driven by Indicator::update() at the 50 Hz mission tick, plain
//  full-on/full-off writes to PcaChannels::STATUS_LED — see
//  Indicator::patternIsOn().
//  Each pattern is defined as "LED on for the first ON_MS of every
//  PERIOD_MS window", except CARRYING, which is a distinct double-blip.
//  StatusColor::OFF has no pattern (always off) and READY is solid (always
//  on), so neither needs an entry here.
//
//  [FIX 2] NO STATUS PATTERN MAY MATCH THE SCORED MINE BLINK.
//  Mission::MINE_LED_ON_MS / MINE_LED_OFF_MS define a 250/250 ms blink. The
//  ZONE pattern used to be exactly that (500 ms period, 250 ms on), so the
//  status LED blinked identically for the whole 5 s dwell and a judge saw
//  ~13 indistinguishable blinks instead of exactly 3. ZONE is now a long
//  slow pulse that cannot be confused with it, and TaskHandler now also
//  blanks the LED on entry to MINE_DEFUSE. Belt and braces, because this is
//  the difference between scoring the mine and scoring zero.
static constexpr uint16_t LED_PATTERN_BOOT_PERIOD_MS  = 300;   // BOOTING: fast blink
static constexpr uint16_t LED_PATTERN_BOOT_ON_MS      = 150;
static constexpr uint16_t LED_PATTERN_CALIB_PERIOD_MS = 1000;  // CALIBRATING: slow blink
static constexpr uint16_t LED_PATTERN_CALIB_ON_MS     = 500;
static constexpr uint16_t LED_PATTERN_RUN_PERIOD_MS   = 1000;  // RUNNING: brief heartbeat
static constexpr uint16_t LED_PATTERN_RUN_ON_MS       = 100;
static constexpr uint16_t LED_PATTERN_ZONE_PERIOD_MS  = 1600;  // ZONE: long slow pulse
static constexpr uint16_t LED_PATTERN_ZONE_ON_MS      = 1100;  //       (was 500/250)
static constexpr uint16_t LED_PATTERN_CARRY_PERIOD_MS = 1000;  // CARRYING: double-blip
static constexpr uint16_t LED_PATTERN_CARRY_PULSE_MS  = 80;    // width of each blip
static constexpr uint16_t LED_PATTERN_CARRY_GAP_MS    = 80;    // gap between the two blips
static constexpr uint16_t LED_PATTERN_ERROR_PERIOD_MS = 120;   // ERROR: very fast blink
static constexpr uint16_t LED_PATTERN_ERROR_ON_MS     = 60;

} // namespace Tune

/**
 * PWM_MIN_MOVE must stay strictly below SPEED_APPROACH: MotorSide::setPwm()
 * silently boosts any smaller nonzero magnitude up to the stiction floor, so
 * if the floor ever crept up to or past the victim-approach speed, the robot
 * would close on the victim faster than the caller asked for and the
 * range-terminated stop in TaskHandler::stepVictimApproach() would overshoot
 * TOF_GRIP_MM more often. This guards that invariant at compile time so a
 * future retune of either constant cannot silently break grip accuracy.
 */
static_assert(Hw::PWM_MIN_MOVE < Tune::SPEED_APPROACH,
              "Hw::PWM_MIN_MOVE must stay below Tune::SPEED_APPROACH or the "
              "stiction floor will override the victim-approach speed.");


// ============================================================================
//  SECTION 3b — TIME-OF-FLIGHT (VL53L1X, Pololu driver pololu/VL53L1X ^1.3.1)
// ============================================================================
//
//  DISTANCE MODE. Every decision distance in this firmware is <= 320 mm
//  (Tune::TOF_*). SHORT mode reaches ~1.3 m, has the best ambient-light
//  immunity of the three modes (venue floodlights!), and is the only mode
//  that permits a 20 ms budget. LONG mode buys range this robot never uses
//  and pays for it with worse behaviour under strong ambient IR. All three
//  units therefore run SHORT. MEDIUM/LONG remain selectable at runtime via
//  LidarBank::setDistanceMode() for bench experiments.
//
//  TIMING BUDGET. Configured here in MILLISECONDS. NOTE: the Pololu driver's
//  setMeasurementTimingBudget() takes MICROSECONDS (ms-based budgets are the
//  SparkFun API). LidarBank converts; nothing outside LidarBank.cpp may call
//  the driver directly. Minimum legal budget: 20 ms in SHORT, 33 ms in
//  MEDIUM/LONG (ST UM2356). The inter-measurement period must be >= budget.
//
//  MINIMUM RANGE. VL53L1X accuracy collapses below ~40 mm, and very close
//  targets come back as RangeValidMinRangeClipped. LidarBank ACCEPTS clipped
//  samples (they mean "something is right against the lens"), which is what
//  Manipulator::verifyGrasp() needs when the victim sits inside the jaws.
// ============================================================================

/** VL53L1X ranging mode, decoupled from the Pololu enum so Config.h stays
 *  library-free. Mapped in LidarBank.cpp. */
enum class TofDistanceMode : uint8_t { SHORT, MEDIUM, LONG };

namespace Tof {

static constexpr uint8_t  DEFAULT_ADDR        = 0x29; // every unit boots here

// --- Per-unit ranging profile ---------------------------------------------
static constexpr TofDistanceMode FRONT_MODE   = TofDistanceMode::SHORT;
static constexpr uint16_t FRONT_BUDGET_MS     = 33;   // victim/grip precision
static constexpr uint16_t FRONT_PERIOD_MS     = 38;   // budget + 5 ms slack

static constexpr TofDistanceMode SIDE_MODE    = TofDistanceMode::SHORT;
static constexpr uint16_t SIDE_BUDGET_MS      = 33;
static constexpr uint16_t SIDE_PERIOD_MS      = 50;   // sides are advisory

/** Legal floor per mode (ST UM2356). */
static constexpr uint16_t MIN_BUDGET_SHORT_MS = 20;
static constexpr uint16_t MIN_BUDGET_LONG_MS  = 33;   // MEDIUM and LONG
static constexpr uint16_t MAX_BUDGET_MS       = 1000;
static constexpr uint16_t PERIOD_SLACK_MS     = 5;

// --- Bring-up --------------------------------------------------------------
static constexpr uint16_t RESET_HOLD_MS       = 10;   // all XSHUT low
static constexpr uint16_t BOOT_WAIT_MS        = 10;   // tBOOT is 1.2 ms max;
                                                      // the rest is pull-up RC
static constexpr uint8_t  INIT_RETRIES        = 3;
/** Driver I/O timeout. MUST be non-zero: VL53L1X::init() polls boot status
 *  in a loop that only terminates on this timeout. 0 == hang forever. */
static constexpr uint16_t IO_TIMEOUT_MS       = 100;

// --- Data quality ------------------------------------------------------------
/** A sample older than (STALE_PERIODS * period + STALE_MARGIN_MS) is stale. */
static constexpr uint8_t  STALE_PERIODS       = 3;
static constexpr uint16_t STALE_MARGIN_MS     = 20;
static constexpr uint16_t MIN_RELIABLE_MM     = 40;

/** Static per-unit offset (mm) added to every accepted sample, indexed by
 *  LidarId. Measure against a white target at 100 mm with the cover/bracket
 *  fitted, and enter (true - reported). The Pololu driver has no offset
 *  calibration API, so this is where the bracket geometry is compensated. */
static constexpr int16_t  OFFSET_MM[3]        = { 0, 0, 0 };

// --- Optional ROI narrowing (FRONT only). Requires SYROBIX_TOF_USE_ROI 1. --
//  Default 16x16 SPADs = ~27 deg FoV. 8x8 is ~15 deg: sharper victim bearing,
//  fewer wall returns in the corner tile, at the cost of signal (range).
static constexpr uint8_t  FRONT_ROI_W         = 8;
static constexpr uint8_t  FRONT_ROI_H         = 8;

} // namespace Tof

static_assert(Tof::FRONT_BUDGET_MS >=
                  ((Tof::FRONT_MODE == TofDistanceMode::SHORT)
                       ? Tof::MIN_BUDGET_SHORT_MS : Tof::MIN_BUDGET_LONG_MS),
              "Tof::FRONT_BUDGET_MS is below the VL53L1X minimum for its mode.");
static_assert(Tof::SIDE_BUDGET_MS >=
                  ((Tof::SIDE_MODE == TofDistanceMode::SHORT)
                       ? Tof::MIN_BUDGET_SHORT_MS : Tof::MIN_BUDGET_LONG_MS),
              "Tof::SIDE_BUDGET_MS is below the VL53L1X minimum for its mode.");
static_assert(Tof::FRONT_PERIOD_MS >= Tof::FRONT_BUDGET_MS &&
              Tof::SIDE_PERIOD_MS  >= Tof::SIDE_BUDGET_MS,
              "VL53L1X inter-measurement period must be >= its timing budget.");
static_assert(Hw::TOF_ADDR_FRONT != Hw::TOF_ADDR_LEFT &&
              Hw::TOF_ADDR_FRONT != Hw::TOF_ADDR_RIGHT &&
              Hw::TOF_ADDR_LEFT  != Hw::TOF_ADDR_RIGHT,
              "Two ToF units share an I2C address.");
static_assert(Hw::TOF_ADDR_FRONT != Tof::DEFAULT_ADDR &&
              Hw::TOF_ADDR_LEFT  != Tof::DEFAULT_ADDR &&
              Hw::TOF_ADDR_RIGHT != Tof::DEFAULT_ADDR,
              "A ToF target address is 0x29: the next unit to boot collides.");
static_assert(Hw::TOF_ADDR_FRONT != Hw::MPU6500_ADDR &&
              Hw::TOF_ADDR_LEFT  != Hw::MPU6500_ADDR &&
              Hw::TOF_ADDR_RIGHT != Hw::MPU6500_ADDR,
              "A ToF address collides with the MPU6500.");
static_assert(Tune::TOF_GRIP_MM >= Tof::MIN_RELIABLE_MM,
              "TOF_GRIP_MM is inside the VL53L1X unreliable near-field.");
static_assert(Tune::TOF_INVALID > 4000,
              "TOF_INVALID must sit above any real VL53L1X range.");


// ============================================================================
//  SECTION 4 — MISSION CONSTANTS (derived from the SYROBIX rulebook)
// ============================================================================

namespace Mission {

static constexpr uint16_t TILE_SIZE_MM         = 300;
static constexpr uint16_t VICTIM_DIAMETER_MM   = 50;   // 40-50 mm spec
static constexpr uint16_t VICTIM_HEIGHT_MM     = 130;  // 100-130 mm spec
static constexpr uint16_t VICTIM_MASS_G        = 100;  // maximum

static constexpr uint32_t MATCH_DURATION_MS    = 6UL * 60UL * 1000UL;

/** Mine defusal is scored ONLY in this exact order. Any deviation = 0 points. */
static constexpr uint32_t MINE_DWELL_MS        = 5000; // 1. remain on tile
static constexpr uint8_t  MINE_LED_BLINKS      = 3;    // 2. blink LED
static constexpr uint16_t MINE_LED_ON_MS       = 250;
static constexpr uint16_t MINE_LED_OFF_MS      = 250;
static constexpr uint16_t MINE_BUZZER_MS       = 600;  // 3. confirmation beep
                                                       //    (active buzzer: fixed pitch)

/**
 * [A-H4] Mine positioning and re-arm, all in encoder millimetres.
 *
 * Rule 5.4: a robot has reached a tile only when MORE THAN HALF of it is
 * inside. Stopping the moment red is confirmed left the colour heads ~40 mm
 * into the tile and the chassis centre outside it. The sequence now line-
 * follows until the chassis CENTRE is on the tile centre, then dwells.
 *
 * After a defusal, red is ignored for MINE_REARM_MM of travel. This replaces
 * the old "crawl until not red" phase, whose timeout path re-entered the same
 * mine forever whenever the classifier stuck on red.
 */
static constexpr float    MINE_REARM_MM          = 200.0f;
static constexpr uint32_t MINE_ADVANCE_TIMEOUT_MS = 3000;

/**
 * [A-C5] Colour confirmations by DISTANCE.
 *
 * Green means two things: a 25x25 mm intersection marker, and the 300 mm safe
 * tile. A red run means two things: a mine tile (300 mm) and the evacuation
 * stripe (the width of a line, 10-20 mm). Length separates both pairs with a
 * wide margin; time did not, because it scales with speed.
 */
static constexpr float GREEN_TILE_CONFIRM_MM     = 60.0f;   // marker reads ~35 mm
static constexpr float RED_TILE_CONFIRM_MM       = 40.0f;   // not carrying
static constexpr float MARKER_WINDOW_MM          = 120.0f;  // marker -> bar
static constexpr float MARKER_POST_BAR_IGNORE_MM = 80.0f;   // markers past a bar
                                                            // belong to others

/**
 * [A-C8] EXIT discriminator (carrying the victim).
 *   red run that ENDS within EXIT_STRIPE_MAX_MM  -> the evacuation stripe
 *   red run that LASTS EXIT_MINE_MIN_RUN_MM      -> a mine tile: defuse it
 *                                                   (points if not yet done),
 *                                                   then keep seeking the exit
 * A stripe must also span EXIT_STRIPE_MIN_FRAMES colour frames, so a single
 * misclassified frame on the black line cannot end the match.
 */
static constexpr float   EXIT_STRIPE_MAX_MM      = 60.0f;
static constexpr float   EXIT_STRIPE_MIN_MM      = 8.0f;   // a real stripe spans 3-6 frames
static constexpr uint8_t EXIT_STRIPE_MIN_FRAMES  = 2;
static constexpr float   EXIT_MINE_MIN_RUN_MM    = 120.0f;

/**
 * [A-C6/C7] Obstacle handling.
 * Before the victim is aboard, an object ahead may BE the victim on the safe
 * tile. The robot crawls up to OBSTACLE_VERIFY_MM looking for a green run of
 * OBSTACLE_VERIFY_GREEN_MM before treating it as an obstacle.
 * The bypass is IMU-angle and encoder-distance based:
 *   turn out TURN_DEG, drive LEG_MM, turn back past parallel by RETURN_DEG,
 *   drive until the line is found (max REJOIN_MAX_MM).
 * @warning Geometry: lateral clearance = LEG * sin(TURN) must exceed half the
 *          chassis width + the obstacle radius. Check it for YOUR chassis.
 */
static constexpr float    OBSTACLE_VERIFY_MM       = 60.0f;
static constexpr float    OBSTACLE_VERIFY_GREEN_MM = 30.0f;
static constexpr float    OBSTACLE_TURN_DEG        = 50.0f;
static constexpr float    OBSTACLE_LEG_MM          = 220.0f;
static constexpr float    OBSTACLE_RETURN_DEG      = 40.0f;
static constexpr float    OBSTACLE_REJOIN_MAX_MM   = 600.0f;
static constexpr uint16_t OBSTACLE_SIDE_CLEAR_MM   = 250;

/** [A-H3] Scan/align/approach/grip retries before lack of progress. */
static constexpr uint8_t  MAX_VICTIM_ATTEMPTS    = 3;

/** [A-C9] Colour frames averaged per taught reference. */
static constexpr uint8_t  COLOR_CAL_FRAMES       = 8;

/** Rule: stop dead on the evacuation tile for 5 s to end the run. */
static constexpr uint32_t EXIT_HOLD_MS         = 5000;

static constexpr uint8_t  MINES_PER_ZONE       = 3;

} // namespace Mission

/**
 * [FIX 2] Compile-time guard: no status pattern may present the same cadence
 * as the scored mine blink. If a future retune makes ZONE a 250/250 blink
 * again, this fails the build instead of quietly costing the mine points.
 */
static_assert(!(Tune::LED_PATTERN_ZONE_ON_MS == Mission::MINE_LED_ON_MS &&
                Tune::LED_PATTERN_ZONE_PERIOD_MS ==
                    (Mission::MINE_LED_ON_MS + Mission::MINE_LED_OFF_MS)),
              "StatusColor::ZONE blinks at exactly the scored mine cadence; a "
              "judge cannot distinguish the dwell from the three scored blinks.");


// ============================================================================
//  SECTION 5 — ENUMERATIONS
// ============================================================================

/** Top-level mission state machine. Owned exclusively by TaskHandler. */
enum class RobotState : uint8_t {
    BOOT,                ///< Peripherals initialising
    CALIBRATING,         ///< Line array + IMU bias capture, clock is running
    IDLE_ARMED,          ///< Calibrated, waiting on the start button
    LINE_FOLLOW,         ///< Nominal navigation
    TURNING,             ///< Executing a committed branch turn
    OBSTACLE_AVOID,      ///< Arc around a floor obstacle, then reacquire line
    RAMP_TRANSIT,        ///< Pitch detected, traction profile changed
    ZONE_ENTERED,        ///< Coloured tile detected, mission zone logic active
    MINE_DEFUSE,         ///< Executing the dwell/blink/buzz/clear sequence
    VICTIM_SCAN,         ///< Servo sweep with the front ToF
    VICTIM_ALIGN,        ///< Closed-loop pivot onto the victim bearing
    VICTIM_APPROACH,     ///< Closed-loop advance until TOF_GRIP_MM
    VICTIM_GRIP,         ///< Jaw close + contact confirmation
    VICTIM_LIFT,         ///< Arm to carry height
    EXIT_SEEK,           ///< Full navigation while carrying; ends on the
                         ///< red evacuation STRIPE (run-length, not time)
    EXIT_HOLD,           ///< Stationary 5 s terminator
    LACK_OF_PROGRESS,    ///< Awaiting manual reset at last checkpoint
    FINISHED,            ///< Run over, motors disabled
    FAULT                ///< Unrecoverable peripheral failure
};

/** Classification emitted by the TCS3200 pair. */
enum class TileColor : uint8_t {
    UNKNOWN = 0,
    WHITE,       ///< Field floor
    BLACK,       ///< Line / speed-bump crossing
    GREEN,       ///< Intersection marker OR the safe (victim) tile
    RED,         ///< Mine tile, or the evacuation stripe
    SILVER       ///< Reserved: zone-entry strip on some field builds
};

/** Committed branch decision at an intersection. */
enum class TurnDirection : uint8_t { NONE, LEFT, RIGHT, U_TURN, STRAIGHT };

/** @return true when @p d is something RobotDrivetrain::pivot() can actually
 *          rotate toward. NONE and STRAIGHT are not rotations. [FIX 4] */
static constexpr bool isRotational(TurnDirection d) {
    return d == TurnDirection::LEFT ||
           d == TurnDirection::RIGHT ||
           d == TurnDirection::U_TURN;
}

/** Logical identity of each VL53L1X, used to index LidarBank. */
enum class LidarId : uint8_t { FRONT = 0, LEFT = 1, RIGHT = 2, COUNT = 3 };

/** Which physical side of the drivetrain a command addresses. */
enum class DriveSide : uint8_t { LEFT = 0, RIGHT = 1 };

/**
 * Visual status codes driven onto the single-colour status LED as
 * non-blocking blink patterns (see Indicator::update() / patternIsOn() and
 * the Tune::LED_PATTERN_* constants above). No colour information survives
 * on this hardware — only timing distinguishes one status from another, which
 * is exactly why no pattern may collide with the scored mine blink.
 */
enum class StatusColor : uint8_t {
    OFF, BOOTING, CALIBRATING, READY, RUNNING, ZONE, CARRYING, ERROR
};


// ============================================================================
//  SECTION 6 — FreeRTOS TOPOLOGY
// ============================================================================
//  Core 0 : latency-tolerant work — I2C transactions, colour integration,
//           ToF ranging, mission supervision. Also hosts the Wi-Fi/BT stack if
//           telemetry is ever enabled, hence the deliberate separation.
//  Core 1 : hard real-time work only — encoder odometry, PID, PWM output. This
//           core must never block on I2C.
// ============================================================================

namespace Rtos {

static constexpr uint8_t  CORE_SENSORS      = 0;
static constexpr uint8_t  CORE_CONTROL      = 1;

// --- Priorities (higher preempts lower) -----------------------------------
static constexpr uint8_t  PRIO_CONTROL      = 5;   ///< PID + drivetrain
static constexpr uint8_t  PRIO_LINE         = 4;   ///< ADC sampling of TCR5000
static constexpr uint8_t  PRIO_MISSION      = 3;   ///< State machine
static constexpr uint8_t  PRIO_SENSORS      = 2;   ///< I2C sensors
static constexpr uint8_t  PRIO_TELEMETRY    = 1;   ///< Serial diagnostics

// --- Stack depths (words, not bytes) --------------------------------------
static constexpr uint32_t STACK_CONTROL     = 4096;
static constexpr uint32_t STACK_LINE        = 3072;
static constexpr uint32_t STACK_MISSION     = 8192;  ///< deepest call chain
static constexpr uint32_t STACK_SENSORS     = 4096;
static constexpr uint32_t STACK_TELEMETRY   = 3072;

// --- Loop periods ----------------------------------------------------------
static constexpr uint32_t PERIOD_CONTROL_MS   = 5;    ///< 200 Hz PID
static constexpr uint32_t PERIOD_LINE_MS      = 5;    ///< 200 Hz, phase-locked
static constexpr uint32_t PERIOD_MISSION_MS   = 20;   ///< 50 Hz supervision
static constexpr uint32_t PERIOD_SENSORS_MS   = 30;   ///< ~33 Hz I2C round-robin
static constexpr uint32_t PERIOD_TELEMETRY_MS = 200;

// --- Mutex acquisition budget ---------------------------------------------
//  The control task must NEVER wait longer than this for the sensor snapshot;
//  it uses the previous sample instead of missing a deadline.
static constexpr uint32_t MUTEX_TIMEOUT_MS    = 2;

} // namespace Rtos


// ============================================================================
//  SECTION 7 — BUILD SWITCHES
// ============================================================================

#define SYROBIX_ENABLE_TELEMETRY   1   ///< Serial diagnostics task
#define SYROBIX_ENABLE_IMU         1   ///< MPU6500 fusion (encoders are primary)
#define SYROBIX_DRY_RUN            0   ///< 1 = compute everything, never drive

/** 1 = release XSHUT by switching the pin to INPUT and letting the breakout's
 *  own pull-up raise it (correct for Pololu and GY-53L1-style boards, whose
 *  XSHUT pull-up goes to the sensor's 2.8 V rail — driving 3.3 V into it
 *  back-feeds that rail). 0 = drive XSHUT actively HIGH, ONLY for bare
 *  modules that have NO pull-up on XSHUT. Measure before choosing. */
#define SYROBIX_TOF_XSHUT_OPEN_DRAIN 1

/** 1 = narrow the FRONT ranger's ROI to Tof::FRONT_ROI_W x FRONT_ROI_H.
 *  Needs pololu/VL53L1X >= 1.3.0 (setROISize). Off until bench-validated. */
#define SYROBIX_TOF_USE_ROI          0

/** [A-C9] 1 = CALIBRATING teaches WHITE/BLACK/GREEN/RED on the field and
 *  refuses to arm without them. 0 = bench only (heuristic classifier). */
#define SYROBIX_REQUIRE_COLOR_CAL    1

/** [Rev 3] GPIO43/44 (UART0) and GPIO19/20 (native USB) are free again, so
 *  Serial telemetry works in match builds over either USB port (see
 *  platformio.ini for which one Serial maps to). */
