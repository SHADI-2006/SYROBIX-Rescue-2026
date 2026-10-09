/**
 * ============================================================================
 *  @file       SensorsSystem.h
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Sensor abstraction layer: TCR5000 line array, MPU6500 IMU,
 *              three-sensor VL53L1X bank (LidarBank.h), and the TCS3200
 *              colour pair.
 *
 *  ---------------------------------------------------------------------------
 *  POST-REVIEW REVISION — what changed in this file
 *  ---------------------------------------------------------------------------
 *    [FIX 1] The hand-rolled `typedef void* SemaphoreHandle_t;` collided with
 *            the real FreeRTOS typedef the moment Arduino.h reached the same
 *            translation unit. Replaced with the actual headers.
 *    [FIX 8] publish() is split. updateLineArray() (Core 1, 200 Hz) now calls
 *            publishLine(), which touches ONLY the line fields. The old code
 *            had the Core-1 line task reading _imu / _lidar / _vision while
 *            the Core-0 sensor task was mid-update on them — the mutex guards
 *            _snapshot, not the sensor objects.
 *    [FIX 8] Imu's ramp/bump state moved out of file-static globals into real
 *            members, and isBumpDetected() is now a first-class getter, so the
 *            snapshot stops shipping the bump latch in slipDetected.
 *    [FIX 7] LineArray::calibrateFinish() — _calibrated no longer flips true
 *            from a mid-sweep transient before the sweep is actually over.
 *    [BENCH] LidarBank::setFocus() — pins the round-robin to one ranger so the
 *            front ToF refreshes every 30 ms instead of every 90 ms during the
 *            victim sequences, where range latency directly costs the grip.
 *
 *  ---------------------------------------------------------------------------
 *  CONCURRENCY MODEL
 *  ---------------------------------------------------------------------------
 *  Two producers, several consumers, one shared snapshot:
 *
 *    Core 0 / SensorTask  -> LidarBank, Imu, ColorVision   (slow, I2C-bound)
 *    Core 1 / LineTask    -> LineArray                     (fast, ADC-bound)
 *    Core 1 / ControlTask -> reads the snapshot, never blocks on it
 *    Core 0 / MissionTask -> reads the snapshot, may block briefly
 *
 *  OWNERSHIP IS STRICT, and this is what the split publisher enforces:
 *  a task publishes ONLY the fields fed by the sensor objects it owns. The
 *  mutex serialises writes to _snapshot; it does not and cannot serialise
 *  access to the sensor objects themselves.
 *
 *  [Rev 3] The I2C bus is also used by the MISSION task for the PCA9685
 *  (servos, LED, buzzer). TwoWire serialises each transaction with its own
 *  mutex, so the PCA traffic and the IMU/ToF traffic never interleave on the
 *  wire; none of this class's sensor objects are touched by the mission task.
 *
 *  The one exception is the CALIBRATING state, where the mission task drives
 *  LineArray directly (calibrateSample / calibrateFinish). The line task is
 *  gated off for the duration — see TaskHandler::lineLoop().
 *
 *  The line array is sampled on Core 1 deliberately. Line position is the only
 *  measurement inside the 200 Hz control loop, and routing it through a
 *  cross-core queue would add jitter to the one signal that cannot tolerate it.
 *  Everything else crosses cores exactly once, through SensorSnapshot.
 * ============================================================================
 */

#pragma once

#include <stdint.h>

// [FIX 1] The real thing. The previous forward declaration
//     struct SemaphoreHandle_s;
//     typedef void* SemaphoreHandle_t;
// is a hard conflicting-declaration error against FreeRTOS's own typedef as
// soon as Arduino.h lands in the same TU, which it does in every .cpp here.
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "Config.h"
#include "LidarBank.h"
#include "LineArray.h"


// ============================================================================
//  LineArray — moved to LineArray.h (Rev 3). Included above.
// ============================================================================

// ============================================================================
//  Imu — MPU6500 over I2C
// ============================================================================

/**
 * @brief Gyro/accel wrapper providing yaw rate, integrated yaw and chassis
 *        pitch.
 *
 * @details Yaw here is a SECONDARY heading source. Encoder differential is
 *          primary because a consumer-grade MEMS gyro drifts several degrees
 *          per minute, and a six-minute match is long enough for that to
 *          matter. The IMU earns its place for two things the encoders cannot
 *          do: detecting the ramp via pitch, and detecting wheel slip by
 *          disagreeing with the encoders.
 */
class Imu {
public:
    Imu();

    /** @return false if the WHO_AM_I register does not answer. */
    bool begin();

    /**
     * @brief Captures the zero-rate bias. The robot MUST be stationary.
     * @param samples Number of averaged samples (512 is ample at 30 Hz).
     *
     * @warning Blocking, ~400-500 ms. Only legal from CALIBRATING, before the
     *          start button is armed and while the line task is gated off.
     */
    void calibrateBias(uint16_t samples = 512);

    /** @brief One I2C read cycle. Called from the Core-0 sensor task. */
    void update(float dtSec);

    float yawDeg()        const;  ///< Integrated, wraps to [-180, +180]
    float yawRateDegS()   const;
    float pitchDeg()      const;  ///< Positive = nose up
    float rollDeg()       const;

    /** @return true when |pitch| indicates the robot is on the ramp. */
    bool isOnRamp() const;

    /**
     * @brief Speed-bump transient latch.                              [FIX 8]
     *
     * A 10 mm bump is a sharp vertical acceleration spike that decays within a
     * couple of ticks; a ramp is sustained pitch with no spike. This used to
     * live in a file-static with no getter, and publish() shipped it in
     * SensorSnapshot::slipDetected — two unrelated signals in one field.
     */
    bool isBumpDetected() const;

    /**
     * @brief Slip detector.
     * @param encoderYawRateDegS Yaw rate implied by the wheel differential.
     * @return true when gyro and encoders disagree beyond tolerance, meaning
     *         at least one wheel is spinning without traction.
     */
    bool detectSlip(float encoderYawRateDegS) const;

    void resetYaw();

    /** [A-H1] Live health: false only after IMU_FAIL_STREAK consecutive bus
     *  failures, and true again on the next good read. */
    bool isHealthy()    const;
    bool isCalibrated() const;

private:
    float _yawDeg, _yawRate, _pitchDeg, _rollDeg;
    float _biasX, _biasY, _biasZ;
    bool  _calibrated;
    bool  _healthy;
    bool  _present;     ///< WHO_AM_I answered at boot; never retried if not
    uint8_t _errStreak; ///< consecutive failed reads

    // --- Ramp / bump discrimination state. [FIX 8] Was file-static in the
    //     .cpp, which made it shared across instances and invisible to callers.
    uint8_t  _rampTicks;
    bool     _rampLatched;
    bool     _bumpLatched;
    uint32_t _bumpStampMs;
};


// ============================================================================
//  LidarBank — 3x VL53L1X, now in its own translation unit (LidarBank.h/.cpp)
// ============================================================================
//  Moved out for the VL53L1X migration so this header stays free of any ToF
//  driver types. SensorsSystem still owns the instance (_lidar) and the
//  Core-0 sensor task is still the only caller of LidarBank::updateNext().

// ============================================================================
//  ColorVision — 2x TCS3200
// ============================================================================

/**
 * @brief Left/right colour classifier for green intersection markers, red mine
 *        tiles, and the green safe tile.
 *
 * @details The TCS3200 reports colour as an output FREQUENCY, so each channel
 *          costs a pulse-width measurement. Four filters (R/G/B/clear) per
 *          sensor per cycle is the dominant cost in the Core-0 task, which is
 *          why colour runs at ~33 Hz and not in the control loop.
 *
 *          S2/S3 are shared across both sensors by design, so the two are
 *          always on the same filter at the same instant and must be sampled
 *          as a pair.
 */
class ColorVision {
public:
    ColorVision();

    void begin();

    /**
     * @brief Records the current reading as the reference for a known colour.
     * @note  Run this on the actual field, on the actual tiles, during the
     *        permitted calibration window. Datasheet values are not usable.
     */

    /**
     * @brief [A-C9] Starts averaging the next Mission::COLOR_CAL_FRAMES
     *        sweeps (both heads) into the reference for @p color.
     * @note  Sensor task only — it is update() that accumulates. The mission
     *        requests this through SensorsSystem::requestColorReference(),
     *        so S2/S3 are never driven from two tasks.
     */
    void beginReference(TileColor color);
    bool referenceBusy() const;
    bool hasReference(TileColor color) const;

    /** @brief Samples all filters on both sensors and reclassifies. */
    void update();

    TileColor left()  const;
    TileColor right() const;

    /** @return Encoded marker pair, which maps directly onto a turn decision. */
    TurnDirection interpretMarkers() const;

    /**
     * @brief Both sensors agree on the same colour.
     * @note  Required before acting on a mine or safe tile: a single-sensor
     *        hit is more likely to be a marker at the edge of an intersection.
     */
    bool bothAgree(TileColor color) const;

    uint16_t rawRed(bool rightSensor)   const;
    uint16_t rawGreen(bool rightSensor) const;
    uint16_t rawBlue(bool rightSensor)  const;

private:
    struct ChannelSet { uint16_t r, g, b, c; };
    ChannelSet _rawLeft;
    ChannelSet _rawRight;
    ChannelSet _reference[6];   ///< Indexed by TileColor
    TileColor  _left;
    TileColor  _right;

    // --- [A-C9] reference capture (sensor task only) -----------------------
    int8_t   _capturing;         ///< TileColor index being taught, -1 = none
    uint8_t  _capFrames;
    uint32_t _capR, _capG, _capB, _capC;

    static TileColor classify(const ChannelSet& sample,
                              const ChannelSet* references);
    uint16_t measureFilter(uint8_t outPin, bool s2, bool s3) const;
};


// ============================================================================
//  SensorSnapshot — the single cross-core data contract
// ============================================================================

/**
 * @brief Plain-old-data view of every sensor, published atomically.
 *
 * @details Consumers copy this struct out under the mutex and then release it
 *          immediately. No consumer holds a reference to live sensor objects,
 *          which is what makes the whole system analysable: there is exactly
 *          one place where sensor data crosses a core boundary.
 *
 *          The line fields are written by publishLine() on Core 1; everything
 *          else by publish() on Core 0. Neither publisher touches the other's
 *          fields, so neither ever reads a sensor object it does not own.
 */
struct SensorSnapshot {
    // --- Line (produced on Core 1, via publishLine()) ----------------------
    float    linePosition;
    uint8_t  lineMask;
    uint8_t  lineActiveCount;
    bool     lineLost;
    bool     intersectionCandidate;
    bool     rearOnLine;   ///< LineArray::isRearOnLine() — see that method's note

    // --- Inertial (produced on Core 0) -------------------------------------
    float    yawDeg;
    float    yawRateDegS;
    float    pitchDeg;
    bool     onRamp;
    bool     bumpDetected;   ///< [FIX 8] was being published as slipDetected
    /**
     * @note Slip needs the encoder-implied yaw rate, which lives in
     *       RobotDrivetrain on Core 1. Call
     *       SensorsSystem::setEncoderYawRate() from the control loop and this
     *       becomes live; until then it is always false and no consumer should
     *       read it as meaningful.
     */
    bool     slipDetected;
    bool     imuHealthy;          ///< [A-H1] false -> use encoder heading

    // --- Ranging -----------------------------------------------------------
    uint16_t rangeFrontMm;
    uint16_t rangeLeftMm;
    uint16_t rangeRightMm;
    bool     obstacleAhead;
    /** [A-C3] millis() at which the FRONT range was collected (0 = never).
     *  Lets stop-and-go logic demand a sample taken AFTER the chassis
     *  settled, instead of trusting whatever the snapshot holds. */
    uint32_t rangeFrontStampMs;

    // --- Colour ------------------------------------------------------------
    TileColor colorLeft;
    TileColor colorRight;
    TurnDirection markerHint;

    // --- Housekeeping ------------------------------------------------------
    uint32_t timestampMs;      ///< last slow-sensor publish
    uint32_t lineTimestampMs;  ///< last line publish
    bool     allSensorsHealthy;
};


// ============================================================================
//  SensorsSystem — facade and snapshot publisher
// ============================================================================

/**
 * @brief Aggregates every sensor and owns the mutex-protected snapshot.
 *
 * @details Ownership rules, now enforced by the split publisher rather than
 *          by convention alone:
 *            - Only SensorTask (Core 0) calls updateSlowSensors(), which
 *              publishes the inertial / ranging / colour fields.
 *            - Only LineTask (Core 1) calls updateLineArray(), which publishes
 *              the line fields and nothing else.
 *            - Everyone else calls getSnapshot() and nothing else.
 *
 *          getSnapshot() takes a timeout because the control task must fail
 *          fast and reuse its previous sample rather than miss a deadline.
 */
class SensorsSystem {
public:
    SensorsSystem();

    /**
     * @brief Brings up I2C, then every sensor in dependency order.
     * @return false if any mandatory sensor fails; the mission layer should
     *         then enter RobotState::FAULT rather than start a doomed run.
     */
    bool begin();

    // --- Producers ---------------------------------------------------------

    /** @brief Core-0 tick: IMU, one ToF, colour pair. Publishes those fields. */
    void updateSlowSensors(float dtSec);

    /** @brief Core-1 tick: ADC sweep of the line array. Publishes line fields
     *         ONLY — it must never read a sensor Core 0 owns. */
    void updateLineArray();

    /**
     * @brief Hands the control loop's encoder-implied yaw rate to the IMU so
     *        SensorSnapshot::slipDetected can carry a real value.
     * @note  Safe to call from Core 1: two naturally-atomic 32-bit stores, no
     *        mutex, so the control task can never block on the sensor task.
     */
    void setEncoderYawRate(float degS);

    // --- Consumer ----------------------------------------------------------

    /**
     * @param out       Destination copy.
     * @param timeoutMs Mutex wait budget.
     * @return false if the mutex was not acquired; @p out is left untouched.
     */
    bool getSnapshot(SensorSnapshot& out,
                     uint32_t timeoutMs = Rtos::MUTEX_TIMEOUT_MS) const;

    // --- Calibration (IDLE / CALIBRATING states only) ----------------------
    //  @warning The line calls drive LineArray from the MISSION task. The line
    //           task must be gated off for the duration — TaskHandler's
    //           g_lineSamplingEnabled does exactly that while CALIBRATING.

    void beginLineCalibration();
    void sampleLineCalibration();
    void endLineCalibration();

    /**
     * [A-C9 / A-H1] IMU bias and colour references are POSTED requests,
     * executed by the Core-0 sensor task, which owns the MPU6500 state and
     * the shared TCS3200 S2/S3 lines. The old versions ran on the mission
     * task, concurrently with the sensor task driving the same hardware.
     * Callable from any task; poll the *Busy() predicate for completion.
     * The robot must be stationary for the IMU request.
     */
    void requestImuBiasCalibration();
    bool imuCalibrationBusy() const;
    void requestColorReference(TileColor color);
    bool colorReferenceBusy() const;
    /** WHITE, BLACK, GREEN and RED have all been taught. */
    bool colorCalibrated() const;

    bool isFullyCalibrated() const;

    // --- Direct access, for calibration UI and unit tests only -------------
    LineArray&   lineArray();
    Imu&         imu();
    LidarBank&   lidar();
    ColorVision& vision();

private:
    LineArray   _line;
    Imu         _imu;
    LidarBank   _lidar;
    ColorVision _vision;

    SensorSnapshot     _snapshot;
    SemaphoreHandle_t  _mutex;
    bool               _healthy;

    volatile float _encoderYawRateDegS;
    volatile bool  _encoderYawValid;

    // --- [A-C9 / A-H1] posted calibration requests ---------------------------
    volatile bool   _imuCalReq;
    volatile int8_t _colorReq;    ///< TileColor index, -1 = none
    volatile bool   _colorBusy;

    /** @brief Core-0 fields only: inertial, ranging, colour. */
    void publish();

    /** @brief Core-1 fields only: the six line fields. [FIX 8] */
    void publishLine();
};
