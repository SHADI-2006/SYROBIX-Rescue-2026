/**
 * ============================================================================
 *  @file       TaskHandler.h
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Manipulator, operator indicators, FreeRTOS task topology, and
 *              the mission state machine that implements the SYROBIX rescue
 *              sequences.
 *
 *  ---------------------------------------------------------------------------
 *  POST-REVIEW REVISION — what changed in this file
 *  ---------------------------------------------------------------------------
 *    [FIX 1] _vision_isRed() is DECLARED. TaskHandler.cpp defined and called
 *            it but it appeared nowhere in this header — a hard compile error.
 *    [FIX 2] Indicator's contract now says the LED is blanked on ENTRY to
 *            MINE_DEFUSE, not at the start of the blink phase. The manual
 *            latch was only covering phase 2, so the ZONE status pattern
 *            blinked through the whole 5 s dwell at the exact same cadence as
 *            the scored blinks.
 *    [FIX 3] stepDefuseMine() gained a fourth phase: clear the tile.
 *    [FIX 4] The intersection decision is latched in stepLineFollow(), not
 *            re-sampled a tick later in stepIntersectionDecide().
 *    [FIX 8] Indicator::buttonRisingEdge() no longer keeps its edge state in
 *            a function-local static.
 *
 *  ---------------------------------------------------------------------------
 *  DESIGN NOTE — WHY THE STATE MACHINE NEVER BLOCKS
 *  ---------------------------------------------------------------------------
 *  Every rescue sequence below is written as a re-entrant step function that
 *  returns a SequenceResult. The mission task calls one step per 20 ms tick.
 *  Nothing calls delay(). Nothing spins on a sensor.
 *
 *  This costs a little clarity and buys two things that matter more: the
 *  match clock and the emergency stop stay responsive during a five-second
 *  mine dwell or a slow servo travel, and a stuck sequence times out instead
 *  of hanging the robot until the referee calls lack of progress.
 *
 *  ---------------------------------------------------------------------------
 *  DESIGN NOTE — WHO MAY CALL transitionTo()
 *  ---------------------------------------------------------------------------
 *  Only the mission task, and only from inside tickStateMachine()'s call tree.
 *  A step function that transitions itself must then return RUNNING, never
 *  SUCCESS, or the dispatcher will transition a second time on top of it.
 *  transitionTo() clears every sub-phase variable, so a self-transition
 *  followed by a dispatcher transition silently restarts the new state.
 * ============================================================================
 */

#pragma once

#include <stdint.h>
#include "Config.h"
#include "RobotDrivetrain.h"
#include "SensorsSystem.h"


// ============================================================================
//  SequenceResult — return contract for every non-blocking step function
// ============================================================================

enum class SequenceResult : uint8_t {
    RUNNING,    ///< Call me again next tick
    SUCCESS,    ///< Goal met, advance the state machine
    FAILED,     ///< Goal unreachable, escalate to recovery
    TIMED_OUT   ///< Watchdog expired, escalate to LACK_OF_PROGRESS
};


// ============================================================================
//  PCA9685 output bus  [Rev 3]
// ============================================================================

/**
 * @brief Thin, failure-aware front end for the Adafruit PCA9685 that now
 *        drives both servos, the status LED and the active buzzer.
 *
 * @details The driver is the global `Adafruit_PWMServoDriver pca` (0x40) in
 *          TaskHandler.cpp. TaskHandler::begin() runs pca.begin(),
 *          pca.setOscillatorFrequency(Hw::PCA_OSC_HZ) and pca.setPWMFreq(50),
 *          then PcaBus::adoptConfiguration(). Manipulator and Indicator write
 *          through servoAngle() / digital() only.
 *
 *          SERVOS: pca.setPWM(ch, 0, map(angle, 0, 180, 102, 491)) — 102..491
 *          ticks of the 50 Hz frame is 500..2400 us. (Not writeMicroseconds():
 *          Adafruit's version re-reads PRESCALE over I2C on every call, with no
 *          error path.)
 *
 *          LED / BUZZER: full ON is pca.setPWM(ch, 4096, 0), full OFF is
 *          pca.setPWM(ch, 0, 4096) — the PCA9685's dedicated bit-12 codes.
 *          (setPWM(ch, 0, 4095) is NOT fully on: it still drops low for one
 *          4.9 us tick every frame.)
 *
 *          RECOVERY: a servo spike that browns out the PCA9685 resets it —
 *          SLEEP set, prescaler back to the 200 Hz default, every output dead.
 *          pcaService() reads MODE1/PRESCALE back every
 *          Hw::PCA_HEALTH_PERIOD_MS and re-initialises on mismatch. Each
 *          (re)initialisation bumps pcaGeneration(); Manipulator and Indicator
 *          compare it against the generation they last wrote under and
 *          re-send every output, so a recovered chip gets its full state back.
 *
 * @warning THREADING. All PCA traffic is issued from the Core-0 MISSION task
 *          (plus setup() before any task exists). The Core-0 SENSOR task
 *          shares the same TwoWire bus for the IMU and ToF. Arduino-ESP32's
 *          TwoWire takes an internal (priority-inheriting) mutex per
 *          transaction, so the two never interleave on the wire. Never call
 *          these from the Core-1 control or line tasks: they must not block
 *          on I2C.
 */
namespace PcaBus {

/** @brief Verifies and records the configuration that pca.begin() +
 *         pca.setPWMFreq(50) just programmed (chip awake, prescaler read back
 *         and cached), and bumps generation().
 *  @pre   Wire.begin(SDA, SCL, ...) has ALREADY run (SensorsSystem::begin()).
 *         pca.begin() calls Wire.begin() with no pins; on Arduino-ESP32 that is
 *         a no-op once the bus is up, but if it ran first it would claim the
 *         default pins instead of GPIO9/10.
 *  @return false if the registers cannot be read back or the chip is asleep. */
bool     adoptConfiguration();

/** @brief Read-back health check + automatic re-init. Mission task, every tick
 *         (rate-limited internally to Hw::PCA_HEALTH_PERIOD_MS). */
void     service(uint32_t nowMs);

/** @return true while the last health check (or init) succeeded. */
bool     isUp();

/** @brief Records that initialisation failed (pca.begin() did not answer). */
void     markDown();

/** @return Incremented on every successful (re)initialisation. */
uint32_t generation();

/** @brief pca.setPWM(channel, 0, map(angle, 0, 180, 102, 491)).
 *  @return true if the I2C write was acknowledged. */
bool     servoAngle(uint8_t channel, uint8_t angleDeg);

/** @brief pca.setPWM(channel, 4096, 0) for ON, pca.setPWM(channel, 0, 4096)
 *         for OFF. @return true if the I2C write was acknowledged. */
bool     digital(uint8_t channel, bool on);

} // namespace PcaBus


// ============================================================================
//  Manipulator — MG996R lift arm + MG90S gripper (PCA9685 channels 0 / 1)
// ============================================================================

/**
 * @brief Two-degree-of-freedom pick-and-carry mechanism.
 *
 * @details Deliberately minimal: a rigid arm on one servo and a jaw on
 *          another. There is no wrist, no cable-and-cam lift, and no linkage.
 *          The victim is a 40-50 mm cylinder that always sits on a known tile,
 *          so the extra degrees of freedom would add failure modes without
 *          adding capability.
 *
 *          The arm angle-to-height relationship is h = L * sin(theta), which
 *          is closed-form and needs no empirical lookup table.
 *
 * @note    [Rev 3] Outputs go to PcaChannels::SERVO_ARM / SERVO_GRIPPER via
 *          PcaBus::servoAngle(). The slew limiter (fractional-degree accumulator)
 *          is unchanged, line for line. What is new is only HOW a pose reaches
 *          the servo: a failed I2C write is retried on the next tick instead
 *          of being assumed, and a PCA re-initialisation re-sends both poses.
 *
 * @warning Both servos are commanded through a slew limiter. Stepping an
 *          MG996R across its full travel in one write draws a current spike.
 *          The servos are powered from the PCA9685's V+ terminal on a
 *          DEDICATED 5-6 V UBEC with bulk capacitance — see the warning at
 *          namespace PcaChannels in Config.h.
 */
class Manipulator {
public:
    Manipulator();

    /** Drives both servos to the stowed carry pose. The PCA9685 must already
     *  be initialised (TaskHandler::begin() does this first). */
    void begin();

    // --- Non-blocking pose commands ---------------------------------------

    void openGripper();
    void closeGripper();
    void armDown();
    void armToCarry();

    /**
     * @brief Slew-limited servo update. Call every mission tick.
     * @param dtSec Elapsed time since the previous call.
     */
    void update(float dtSec);

    /** @return true once both servos have reached their commanded targets. */
    bool isSettled() const;

    // --- Grasp verification ------------------------------------------------

    /**
     * @brief Confirms the victim is actually held.
     * @param frontRangeMm Live front ToF reading.
     * @return true when the jaw has closed onto something of the right size.
     *
     * @note  Grasp is inferred rather than sensed: the jaw reaches its
     *        commanded close, and the front ranger still sees an object at
     *        grip distance. Rev 3 freed four GPIOs; a jaw contact switch on one
     *        of them should replace this inference — it is the weakest link
     *        in the pick sequence.
     */
    bool verifyGrasp(uint16_t frontRangeMm) const;

    uint8_t armAngle()     const;
    uint8_t gripperAngle() const;

private:
    uint8_t _armTarget,     _armCurrent;
    uint8_t _gripperTarget, _gripperCurrent;
    float   _slewAccumulator;
    bool    _carrying;

    // --- [Rev 3] output bookkeeping -----------------------------------------
    uint8_t  _armWritten;       ///< angle last ACKed by the PCA9685
    uint8_t  _gripperWritten;
    bool     _armValid;         ///< false -> resend on the next update()
    bool     _gripperValid;
    uint32_t _outGeneration;    ///< PcaBus::generation() at the last full send

    /** Sends @p angle to @p channel; records success in @p written / @p valid. */
    static void writeServo(uint8_t channel, uint8_t angle,
                           uint8_t& written, bool& valid);
    /** Re-sends anything not yet acknowledged, or everything after a PCA re-init. */
    void refreshOutputs();
};


// ============================================================================
//  Indicator — status LED (PCA ch 2), ACTIVE buzzer (PCA ch 3), start button
// ============================================================================

/**
 * @brief Operator-facing I/O and the scored mine-defusal signal chain.
 *
 * @warning The rulebook scores the defusal signals in a fixed order: dwell,
 *          then three LED blinks, then the buzzer. Out-of-order or missing
 *          signals score zero for that mine. The sequencing lives in
 *          TaskHandler::stepDefuseMine(), and this class must not reorder it.
 *
 * @note    [Rev 3] Both outputs are PCA9685 channels switched full ON / full
 *          OFF. The buzzer is ACTIVE (fixed pitch), so the old tone(Hz, ms)
 *          became beep(ms): ON now, OFF when _toneEndMs passes. Writes are
 *          change-driven (the pattern engine runs at 50 Hz but only touches the
 *          bus when the LED actually toggles), a failed write is retried next
 *          tick, and a PCA re-initialisation re-sends both states.
 *
 * @warning ledOn()/ledOff() latch "manual mode": once either is called,
 *          update() stops re-driving the LED from the active StatusColor's
 *          pattern until the next setStatus() call hands control back.
 *
 *          [FIX 2] THE LATCH MUST BE TAKEN ON ENTRY TO MINE_DEFUSE, NOT AT
 *          THE FIRST BLINK, or a status pattern can blink during the dwell and
 *          be mistaken for the scored blinks. Two defences stay in place:
 *            1. TaskHandler::transitionTo() calls ledOff() on entry to
 *               MINE_DEFUSE, so the LED is dark for the entire dwell.
 *            2. Tune::LED_PATTERN_ZONE_* is a long slow pulse that cannot be
 *               mistaken for a 250/250 blink, guarded by a static_assert in
 *               Config.h.
 */
class Indicator {
public:
    Indicator();

    /** Button pin + both outputs OFF + BOOTING pattern. The PCA9685 must
     *  already be initialised (TaskHandler::begin() does this first). */
    void begin();

    /** @brief Selects a status and hands LED control back to the automatic
     *         blink-pattern engine (clears manual mode — see the class note). */
    void setStatus(StatusColor color);

    /** @brief Manual LED control. Latches manual mode — see the class note. */
    void ledOn();
    void ledOff();

    /** @brief Active buzzer ON for @p durationMs, switched OFF by update(). */
    void beep(uint16_t durationMs);
    /** @brief Active buzzer OFF now, cancelling any running beep. */
    void beepOff();

    /** @return Debounced, active-low button state. */
    bool buttonPressed();

    /** @return true exactly once per press, on the released -> pressed edge. */
    bool buttonRisingEdge();

    /** @brief Button debounce, beep timeout, blink pattern (unless manual),
     *         output retry/refresh. Call every mission tick. */
    void update();

private:
    StatusColor _status;
    bool        _manualOverride;   ///< true after ledOn()/ledOff(); cleared by setStatus()
    uint32_t    _patternStartMs;   ///< phase-zero reference for the active pattern

    bool        _lastButtonRaw;
    bool        _debouncedState;
    bool        _edgePrev;         ///< previous debounced level
    uint32_t    _lastDebounceMs;
    uint32_t    _toneEndMs;        ///< 0 = no beep running

    // --- [Rev 3] output bookkeeping -----------------------------------------
    bool        _ledWant,    _ledWritten,    _ledValid;
    bool        _buzzerWant, _buzzerWritten, _buzzerValid;
    uint32_t    _outGeneration;

    /** @return Whether the LED should be ON at @p elapsedMs into @p status's pattern. */
    static bool patternIsOn(StatusColor status, uint32_t elapsedMs);

    void writeLed(bool on);
    void writeBuzzer(bool on);
    void refreshOutputs();
};


// ============================================================================
//  MissionContext — everything the run needs to remember
// ============================================================================

/**
 * @brief Volatile per-run state. Reset on every start-button press.
 */
struct MissionContext {
    RobotState    state;
    RobotState    previousState;
    uint32_t      stateEnteredMs;
    uint32_t      matchStartMs;

    // --- Navigation --------------------------------------------------------
    TurnDirection pendingTurn;
    uint8_t       lackOfProgressCount;
    // [A-V4] tilesTraversed / checkpointIndex removed: written once, never read.

    // --- Zone progress -----------------------------------------------------
    bool          zoneEntered;
    uint8_t       minesDefused;
    bool          victimAcquired;
    bool          victimDelivered;

    // --- Victim scan results ----------------------------------------------
    float         victimBearingDeg;   ///< Servo-frame bearing from coarse sweep
    uint16_t      victimRangeMm;
    bool          victimFound;
    uint8_t       victimAttempts;     ///< [A-H3] scan..grip retries this zone visit

    // --- Diagnostics -------------------------------------------------------
    uint16_t      estimatedScore;
    bool          emergencyStopped;
};


// ============================================================================
//  TaskHandler — task topology and mission logic
// ============================================================================

/**
 * @brief Creates and owns every FreeRTOS task, and hosts the rescue sequences.
 *
 * ---------------------------------------------------------------------------
 *  TASK MAP
 * ---------------------------------------------------------------------------
 *  Core 1 (real-time, must never touch I2C)
 *    controlTask   prio 5, 200 Hz  — drivetrain PID and PWM output
 *    lineTask      prio 4, 200 Hz  — TCR5000 ADC sweep, publishes line fields
 *
 *  Core 0 (latency-tolerant)
 *    missionTask   prio 3,  50 Hz  — this state machine
 *    sensorTask    prio 2,  33 Hz  — IMU, ToF round-robin, colour pair
 *    telemetryTask prio 1,   5 Hz  — serial diagnostics, optional
 *
 *  Cross-core traffic is exactly one object wide: SensorSnapshot. Motion
 *  requests travel the other way through RobotDrivetrain's request setters.
 *
 *  ONE EXCEPTION, and it is deliberate: while the state is CALIBRATING the
 *  mission task owns LineArray directly and lineTask is gated off. Both
 *  sampling the same ADC channels and the same per-channel extrema at once is
 *  a data race — see lineLoop().
 * ---------------------------------------------------------------------------
 */
class TaskHandler {
public:
    TaskHandler(RobotDrivetrain& drive, SensorsSystem& sensors);

    /**
     * @brief Initialises subsystems and spawns all five tasks pinned to cores.
     * @return false if any task creation or peripheral bring-up fails.
     */
    bool begin();

    /** @brief Latches the emergency stop from any context, including an ISR. */
    void emergencyStop();

    /** @return false if begin() failed because the PCA9685 did not answer
     *          (as opposed to a task-creation failure). [Rev 3] */
    bool peripheralsUp() const;

    const MissionContext& context() const;

private:
    RobotDrivetrain& _drive;
    SensorsSystem&   _sensors;
    Manipulator      _manipulator;
    Indicator        _indicator;
    MissionContext   _ctx;

    // ======================================================================
    //  Task entry points (static trampolines into the member loops)
    // ======================================================================
    static void controlTaskEntry(void* pv);
    static void lineTaskEntry(void* pv);
    static void missionTaskEntry(void* pv);
    static void sensorTaskEntry(void* pv);
    static void telemetryTaskEntry(void* pv);

    void controlLoop();     ///< Core 1 — vTaskDelayUntil, 5 ms
    void lineLoop();        ///< Core 1 — vTaskDelayUntil, 5 ms
    void missionLoop();     ///< Core 0 — vTaskDelayUntil, 20 ms
    void sensorLoop();      ///< Core 0 — vTaskDelayUntil, 30 ms
    void telemetryLoop();   ///< Core 0 — vTaskDelayUntil, 200 ms

    // ======================================================================
    //  Sensor helpers
    // ======================================================================

    /**
     * @brief Both colour heads agree the tile is red.                 [FIX 1]
     * @note  A single-head red is far more likely to be a marker at the edge
     *        of an intersection than a mine tile, so both must agree.
     *        This was defined in the .cpp but never declared here, which made
     *        the whole translation unit fail to compile.
     */
    bool _vision_isRed(const SensorSnapshot& snap) const;

    // ======================================================================
    //  State machine
    // ======================================================================

    /** @brief Dispatches one tick to the handler for the current state. */
    void tickStateMachine(const SensorSnapshot& snap, float dtSec);

    /** @brief Transitions with entry actions and a fresh state timestamp.
     *  @note  Clears EVERY sub-phase and colour-debounce variable. See the
     *         "who may call transitionTo()" note at the top of this file. */
    void transitionTo(RobotState next);

    /** @return Milliseconds elapsed in the current state. */
    uint32_t timeInState() const;

    /** @return Milliseconds remaining in the six-minute match. */
    uint32_t timeRemaining() const;

    // ======================================================================
    //  Navigation behaviours
    // ======================================================================

    /**
     * @brief Nominal line following plus every tile/terrain trigger.
     * @note  [FIX 4] This is where an intersection decision is LATCHED, and
     *        [FIX 5] where the dead-end marker pair is latched, because both
     *        signals only exist while the markers are under the colour heads.
     */
    SequenceResult stepLineFollow(const SensorSnapshot& snap);


    /** @note [FIX 4] Returns SUCCESS immediately if pendingTurn is not
     *        rotational, so a failed latch costs one tick instead of the full
     *        4 s watchdog and a trip through LACK_OF_PROGRESS. */
    SequenceResult stepTurning(const SensorSnapshot& snap);

    /** @brief Arcs around a floor obstacle and reacquires the line beyond it. */
    SequenceResult stepObstacleAvoid(const SensorSnapshot& snap);

    /** @brief Raises torque and lowers speed for the ramp; restores on exit. */
    SequenceResult stepRampTransit(const SensorSnapshot& snap);

    // ======================================================================
    //  Rescue sequences
    // ======================================================================

    /**
     * @brief Two-pass sweep for the victim, by ROTATING THE CHASSIS.
     *
     * @details There is no pan servo in the pin budget — Config.h has exactly
     *          two servos, arm and gripper, and neither aims the ToF. So the
     *          sweep rotates the robot slowly and samples the fixed front
     *          ranger. That is also better than panning: a panning servo finds
     *          a bearing you must then rotate onto, reintroducing the
     *          open-loop error the design exists to avoid. Rotating to scan
     *          leaves the robot already pointing at the victim.
     *
     * @note    [A-C3] STOP-AND-GO: pivot one SCAN_COARSE_STEP_DEG at
     *          SPEED_PIVOT, brake, and record (range, yaw) only from a ToF
     *          sample that started after the chassis settled. Sweeps
     *          -SCAN_HALF_ARC_DEG..+SCAN_HALF_ARC_DEG around the entry heading.
     */
    SequenceResult stepVictimScan(const SensorSnapshot& snap);

    /**
     * @brief [A-C3] Rotates onto the scanned bearing, closed on IMU yaw
     *        (gyro, not wheel odometry, so skid-steer slip does not matter):
     *        full-speed pivot with early brake, then timed pulse nudges, then
     *        a stationary range check that something really is ahead.
     */
    SequenceResult stepVictimAlign(const SensorSnapshot& snap);

    /**
     * @brief Closes on the victim until the front range reaches TOF_GRIP_MM.
     *        [A-H5] Stops at TOF_ARM_LOWER_MM to lower the arm, then creeps;
     *        falls back to odometry if the lowered jaws occlude the ranger.
     * @note  Range-terminated, not distance-terminated. Driving a fixed
     *        distance assumes the scan range was exact; closing on a live
     *        measurement does not.
     */
    SequenceResult stepVictimApproach(const SensorSnapshot& snap);

    SequenceResult stepVictimGrip(const SensorSnapshot& snap);
    SequenceResult stepVictimLift(const SensorSnapshot& snap);

    /**
     * @brief The scored mine sequence, in mandated order.
     *        Setup: [A-H4] line-follow until the chassis CENTRE is on the tile
     *               centre (rule 5.4), then stop
     *        1: hold station for MINE_DWELL_MS, LED DARK
     *        2: blink the LED exactly MINE_LED_BLINKS times
     *        3: sound the buzzer for MINE_BUZZER_MS
     * @note  The old "crawl off the tile" phase is gone: after SUCCESS red is
     *        ignored for MINE_REARM_MM of travel, which cannot livelock on a
     *        classifier stuck on red the way the timed crawl could.
     *
     * @note  Any interruption restarts the sequence from phase 1. A partial
     *        sequence scores nothing, so there is no value in resuming it.
     */
    SequenceResult stepDefuseMine(const SensorSnapshot& snap);

    // [A-C8] stepExitSeek() removed. EXIT_SEEK now runs stepLineFollow() —
    // the SAME navigation (markers, obstacles, ramps, line-lost) — with
    // carrying-mode red handling: a short red run is the evacuation stripe,
    // a long one is a mine tile.

    /** @brief Full stop on the evacuation tile for EXIT_HOLD_MS to end the run. */
    SequenceResult stepExitHold(const SensorSnapshot& snap);

    // ======================================================================
    //  Recovery
    // ======================================================================

    /**
     * @brief Detects the rulebook's lack-of-progress conditions: line lost
     *        past the next tile, arrival at an out-of-sequence tile, or a
     *        sequence watchdog expiry.
     */
    bool detectLackOfProgress(const SensorSnapshot& snap);

    /** @brief Halts, signals the operator, and waits for a button press. */
    SequenceResult stepLackOfProgress(const SensorSnapshot& snap);

    /** @brief Restores clean state after the robot is replaced at a checkpoint. */
    void resumeFromCheckpoint();

    // ======================================================================
    //  Startup
    // ======================================================================

    /** @brief Line-array sweep and IMU bias capture. The match clock runs. */
    SequenceResult stepCalibrating(const SensorSnapshot& snap);

    /** @brief Armed and waiting on the single mandated start button. */
    SequenceResult stepIdleArmed(const SensorSnapshot& snap);

    // ======================================================================
    //  Scoring model (diagnostics only, never a control input)
    // ======================================================================
    void addScore(uint16_t points);
    uint16_t estimateScore() const;

    // ======================================================================
    //  [A-C1..C8] Helpers
    // ======================================================================
    /** LINE_FOLLOW before the victim is aboard, EXIT_SEEK after. */
    RobotState navState() const;
    /** IMU yaw when healthy, else encoder heading (degraded on skid-steer). */
    float yawNow(const SensorSnapshot& snap) const;
    /** Forward odometry, mm. Cross-core safe (encoder counts are spinlocked). */
    float odoMm() const;
    /** [A-H3] Bounded retry of the victim sequence. */
    void retryVictim();
};
