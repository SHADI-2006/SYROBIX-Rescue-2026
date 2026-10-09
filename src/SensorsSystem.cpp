/**
 * ============================================================================
 *  @file       SensorsSystem.cpp
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Implementation of Imu (MPU6500), ColorVision (2x TCS3200) and
 *              the SensorsSystem facade. LidarBank (3x VL53L1X) lives in
 *              LidarBank.cpp. The SensorsSystem
 *              facade that publishes the cross-core snapshot.
 *
 *  @dependency pololu/VL53L1X @ ^1.3.1   (via LidarBank.cpp)
 *              The MPU6500 and TCS3200 are driven register-level / bit-bang
 *              here; neither warrants a library.
 *
 *  ---------------------------------------------------------------------------
 *  POST-REVIEW REVISION — what changed in this file
 *  ---------------------------------------------------------------------------
 *    [FIX 1] The header no longer forward-declares SemaphoreHandle_t, so the
 *            "REQUIRED ONE-LINE HEADER FIX" note that used to live here is
 *            gone — the fix is applied in SensorsSystem.h.
 *
 *    [FIX 8] publish() is SPLIT.  This was the real cross-core race, and it
 *            was not visible from the mutex: updateLineArray() runs on Core 1
 *            at 200 Hz and used to call the full publish(), which reads _imu,
 *            _lidar and _vision — objects the Core-0 sensor task mutates. The
 *            mutex serialises writes to _snapshot; it does nothing for the
 *            sensor objects themselves. Concretely, LidarBank::updateNext()
 *            writes _range[idx] then _stamp[idx] in two separate stores, and
 *            a Core-1 publish landing between them pairs a fresh range with a
 *            stale timestamp, so isValid() lies. ColorVision::update() writes
 *            _left then _right, so bothAgree() could see one head from this
 *            filter sweep and one from the last.
 *            Now each task publishes only the fields it owns, and the two
 *            sets are disjoint. The race is gone by construction, not by luck.
 *
 *    [FIX 8] Imu's ramp/bump state moved from file-static globals into real
 *            members, and isBumpDetected() is a proper getter. publish() used
 *            to ship the bump latch inside SensorSnapshot::slipDetected —
 *            two different signals in one field, and Imu::detectSlip() was
 *            never called at all. slipDetected is now fed by a real slip
 *            check as soon as the control loop calls setEncoderYawRate().
 *
 *    [FIX 7] endLineCalibration() now calls calibrateFinish() before read(),
 *            so isCalibrated() cannot go true from a mid-sweep transient.
 *
 *    [BENCH] LidarBank::setFocus() / clearFocus(). Round-robin over three
 *            rangers at a 30 ms tick refreshes any one of them only every
 *            ~90 ms. VICTIM_APPROACH terminates on a 57 mm range window while
 *            driving, so that is up to 90 ms of closing blind. The mission
 *            layer now focuses FRONT for the victim sequences.
 *
 *  ---------------------------------------------------------------------------
 *  NAMING: getYaw() / detectRampAndBump()
 *  ---------------------------------------------------------------------------
 *  The headers name these yawDeg() and isOnRamp(). Implemented under the header
 *  names, since those are what TaskHandler.h already calls. Bump detection now
 *  has a getter (isBumpDetected()) and real member state, so the file-static
 *  workaround is retired.
 *
 *  ---------------------------------------------------------------------------
 *  WHERE THE TIME GOES (measured budget per Core-0 tick, 30 ms period)
 *  ---------------------------------------------------------------------------
 *    IMU burst read (14 bytes, one transaction) ..............  ~0.45 ms
 *    VL53L1X service (non-blocking, see LidarBank.cpp) .......  ~0.1-2.4 ms
 *    TCS3200 pair, 4 filters, both sensors ...................  ~1.90 ms
 *    Snapshot publish under mutex ............................  ~0.02 ms
 *                                                              ---------
 *                                                               ~2.7 ms  (9%)
 *
 *  The colour pair dominates, because every filter costs a pulse measurement
 *  plus a photodiode settle. Three things keep it bounded: S2/S3 are shared so
 *  both sensors are read back-to-back on one filter with one settle, the
 *  ranging is continuous-mode so it never blocks on a fresh conversion, and
 *  the IMU is a single burst rather than seven register reads.
 * ============================================================================
 */

#include "SensorsSystem.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>


// ============================================================================
//  File-local state and constants
// ============================================================================

namespace {

// --- MPU6500 register map --------------------------------------------------
constexpr uint8_t MPU_SMPLRT_DIV    = 0x19;
constexpr uint8_t MPU_CONFIG        = 0x1A;
constexpr uint8_t MPU_GYRO_CONFIG   = 0x1B;
constexpr uint8_t MPU_ACCEL_CONFIG  = 0x1C;
constexpr uint8_t MPU_ACCEL_XOUT_H  = 0x3B;
constexpr uint8_t MPU_PWR_MGMT_1    = 0x6B;
constexpr uint8_t MPU_WHO_AM_I      = 0x75;
constexpr uint8_t MPU_WHOAMI_6500   = 0x70;

/** +/-500 deg/s full scale -> 65.5 LSB per deg/s. */
constexpr float GYRO_LSB_PER_DPS    = 65.5f;
/** +/-4 g full scale -> 8192 LSB per g. */
constexpr float ACCEL_LSB_PER_G     = 8192.0f;

/**
 * Gyro readings below this magnitude are treated as zero.
 *
 * Bias calibration removes the constant offset but not the noise floor, and
 * integrating that noise is exactly how a stationary robot's heading walks
 * away over a six-minute match.
 */
constexpr float GYRO_DEADBAND_DPS   = 0.35f;

// --- Ramp / bump discrimination -------------------------------------------
/** Rulebook caps ramp incline at 25 deg. Assert the ramp well below that. */
constexpr float RAMP_ENTER_DEG      = 9.0f;
/** Hysteresis: a single threshold chatters at the ramp transition. */
constexpr float RAMP_EXIT_DEG       = 6.0f;
/** Sustained pitch required before the ramp is believed. */
constexpr uint8_t RAMP_CONFIRM_TICKS = 3;

/**
 * A speed bump is 10 mm tall and produces a brief vertical acceleration spike
 * with almost no sustained pitch. A ramp produces sustained pitch with almost
 * no spike. Discriminating on duration is what keeps a bump from being
 * mistaken for a ramp entry.
 */
constexpr float BUMP_ACCEL_G        = 1.55f;
constexpr uint16_t BUMP_LATCH_MS    = 250;

/** Gyro-vs-encoder disagreement beyond this means a wheel is slipping. */
constexpr float SLIP_TOLERANCE_DPS  = 22.0f;

// --- TCS3200 ---------------------------------------------------------------
/**
 * pulseIn timeout. At 20% frequency scaling in normal light the output sits
 * between roughly 10 and 60 kHz, so a half-period is 8-50 us. 3 ms is a
 * generous ceiling that still fails fast on a dead sensor.
 */
constexpr uint32_t COLOR_PULSE_TIMEOUT_US = 3000;
/** Photodiode bank settle after switching S2/S3. */
constexpr uint16_t COLOR_SETTLE_US        = 120;
/** Normalised channel ratios are compared in this many parts per thousand. */
constexpr uint16_t COLOR_RATIO_SCALE      = 1000;

inline float clampF(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

inline float wrapDeg(float deg) {
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/** Blocking single-register write. Used only during bring-up. */
bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t& out) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, static_cast<uint8_t>(1)) != 1) return false;
    out = Wire.read();
    return true;
}

/** [A-H1] Consecutive failed IMU reads before it is reported unhealthy. */
constexpr uint8_t IMU_FAIL_STREAK = 5;

/** [A-C9] Weight of the brightness dimension relative to one chroma axis
 *  (as a right-shift of its squared difference: 2 == quarter weight, i.e.
 *  half weight on the distance). Brightness moves with ride height and venue
 *  light more than chroma does, but without it black and white are the SAME
 *  point in clear-normalised chroma space. */
constexpr uint8_t  COLOR_BRIGHT_SHIFT = 2;
/** [A-C9] Squared distance beyond which a sample matches NO reference and is
 *  reported UNKNOWN. UNKNOWN is the safe answer: it triggers nothing. */
constexpr uint32_t COLOR_REJECT_DIST2 = 350UL * 350UL;

} // anonymous namespace


// ============================================================================
//  Imu — MPU6500
// ============================================================================

Imu::Imu()
    : _yawDeg(0.0f), _yawRate(0.0f), _pitchDeg(0.0f), _rollDeg(0.0f),
      _biasX(0.0f), _biasY(0.0f), _biasZ(0.0f),
      _calibrated(false), _healthy(false), _present(false), _errStreak(0),
      _rampTicks(0), _rampLatched(false),
      _bumpLatched(false), _bumpStampMs(0) {}

bool Imu::begin() {
    uint8_t who = 0;
    if (!i2cReadReg(Hw::MPU6500_ADDR, MPU_WHO_AM_I, who)) {
        _healthy = false;
        return false;
    }
    // 0x70 is MPU6500. An MPU6050 answers 0x68 and an ICM-20689 0x98; accept
    // only what we were built for rather than half-working on a lookalike.
    if (who != MPU_WHOAMI_6500) {
        _healthy = false;
        return false;
    }

    // Reset, then select the best available clock (gyro X PLL).
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_PWR_MGMT_1, 0x80);
    delay(100);
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_PWR_MGMT_1, 0x01);
    delay(10);

    // DLPF at 41 Hz. The chassis vibrates hard on the speed bumps and an
    // unfiltered gyro will alias that straight into the yaw integral.
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_CONFIG,       0x03);
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_SMPLRT_DIV,   0x04);   // 200 Hz
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_GYRO_CONFIG,  0x08);   // +/-500 dps
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_ACCEL_CONFIG, 0x08);   // +/-4 g

    _present   = true;
    _healthy   = true;
    _errStreak = 0;
    return true;
}

void Imu::calibrateBias(uint16_t samples) {
    if (!_present) return;

    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    uint16_t taken = 0;

    for (uint16_t i = 0; i < samples; ++i) {
        Wire.beginTransmission(Hw::MPU6500_ADDR);
        Wire.write(MPU_ACCEL_XOUT_H);
        if (Wire.endTransmission(false) != 0) continue;
        if (Wire.requestFrom(Hw::MPU6500_ADDR, static_cast<uint8_t>(14)) != 14) continue;

        for (uint8_t skip = 0; skip < 8; ++skip) Wire.read();   // accel + temp

        const int16_t gx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        const int16_t gy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        const int16_t gz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

        sumX += gx; sumY += gy; sumZ += gz;
        ++taken;

        // [A-H1] Now runs on the SENSOR task (posted request), so nothing else
        // touches the MPU6500 state meanwhile. Yield so telemetry breathes.
        vTaskDelay(1);
    }

    if (taken == 0) {
        _calibrated = false;
        return;
    }

    _biasX = static_cast<float>(sumX / taken);
    _biasY = static_cast<float>(sumY / taken);
    _biasZ = static_cast<float>(sumZ / taken);
    _calibrated = true;

    resetYaw();
}

void Imu::update(float dtSec) {
    // [A-H1] One NACK used to set _healthy = false forever: yaw froze, and
    // every IMU-closed rotation (U-turn, scan, align) ran into its watchdog.
    // Now only a STREAK of failures marks it unhealthy, and every tick retries.
    if (!_present) return;

    // Single 14-byte burst: accel XYZ, temperature, gyro XYZ. Seven separate
    // register reads would cost seven address phases and seven stop conditions
    // for identical data.
    Wire.beginTransmission(Hw::MPU6500_ADDR);
    Wire.write(MPU_ACCEL_XOUT_H);
    bool ok = (Wire.endTransmission(false) == 0);
    if (ok && Wire.requestFrom(Hw::MPU6500_ADDR, static_cast<uint8_t>(14)) != 14) {
        ok = false;
    }
    if (!ok) {
        if (_errStreak < 255) ++_errStreak;
        if (_errStreak >= IMU_FAIL_STREAK) _healthy = false;
        return;
    }
    _errStreak = 0;
    _healthy   = true;

    const int16_t ax = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t ay = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t az = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read();                                   // temperature
    const int16_t gx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t gy = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const int16_t gz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

    (void)gx; (void)gy;   // Only yaw is integrated; pitch/roll come from accel.

    // --- Yaw ---------------------------------------------------------------
    float rate = Hw::IMU_YAW_SIGN *
                 (static_cast<float>(gz) - _biasZ) / GYRO_LSB_PER_DPS;
    if (fabsf(rate) < GYRO_DEADBAND_DPS) {
        rate = 0.0f;
    }
    _yawRate = rate;

    if (_calibrated) {
        // Absolute heading across a full 180 turn: integrate then wrap, so the
        // caller always sees a continuous [-180, +180) value rather than an
        // unbounded accumulator.
        _yawDeg = wrapDeg(_yawDeg + rate * dtSec);
    }

    // --- Attitude from gravity --------------------------------------------
    const float axg = static_cast<float>(ax) / ACCEL_LSB_PER_G;
    const float ayg = static_cast<float>(ay) / ACCEL_LSB_PER_G;
    const float azg = static_cast<float>(az) / ACCEL_LSB_PER_G;

    _pitchDeg = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * 57.2957795f;
    _rollDeg  = atan2f(ayg, azg) * 57.2957795f;

    // --- Ramp vs bump discrimination --------------------------------------
    // A 10 mm speed bump is a transient: a sharp vertical acceleration spike
    // that decays within a couple of ticks. A ramp is the opposite: sustained
    // pitch, no spike. Requiring RAMP_CONFIRM_TICKS of continuous pitch before
    // latching is what stops a bump from being read as a ramp entry.
    const float magnitude = sqrtf(axg * axg + ayg * ayg + azg * azg);
    const uint32_t nowMs = millis();

    if (magnitude > BUMP_ACCEL_G) {
        _bumpLatched = true;
        _bumpStampMs = nowMs;
    } else if (_bumpLatched && (nowMs - _bumpStampMs) > BUMP_LATCH_MS) {
        _bumpLatched = false;
    }

    const float absPitch = fabsf(_pitchDeg);
    if (!_rampLatched) {
        if (absPitch > RAMP_ENTER_DEG) {
            if (++_rampTicks >= RAMP_CONFIRM_TICKS) {
                _rampLatched = true;
            }
        } else {
            _rampTicks = 0;
        }
    } else {
        if (absPitch < RAMP_EXIT_DEG) {
            _rampLatched = false;
            _rampTicks   = 0;
        }
    }
}

float Imu::yawDeg()      const { return _yawDeg;   }
float Imu::yawRateDegS() const { return _yawRate;  }
float Imu::pitchDeg()    const { return _pitchDeg; }
float Imu::rollDeg()     const { return _rollDeg;  }

bool Imu::isOnRamp() const {
    // Latched and hysteretic. Reporting the raw threshold here would make the
    // mission layer's RAMP_TRANSIT state flicker at every transition edge.
    return _rampLatched;
}

bool Imu::isBumpDetected() const {
    return _bumpLatched;
}

bool Imu::detectSlip(float encoderYawRateDegS) const {
    if (!_healthy || !_calibrated) {
        return false;   // No trustworthy reference; do not cry wolf.
    }
    return fabsf(_yawRate - encoderYawRateDegS) > SLIP_TOLERANCE_DPS;
}

bool Imu::isHealthy()    const { return _healthy;    }
bool Imu::isCalibrated() const { return _calibrated; }

void Imu::resetYaw() {
    _yawDeg = 0.0f;
}


// LidarBank implementation: see LidarBank.cpp (VL53L1X).


// ============================================================================
//  ColorVision — two TCS3200 with shared S2/S3
// ============================================================================

ColorVision::ColorVision()
    : _left(TileColor::UNKNOWN), _right(TileColor::UNKNOWN),
      _capturing(-1), _capFrames(0), _capR(0), _capG(0), _capB(0), _capC(0) {
    _rawLeft  = { 0, 0, 0, 0 };
    _rawRight = { 0, 0, 0, 0 };
    for (uint8_t i = 0; i < 6; ++i) {
        _reference[i] = { 0, 0, 0, 0 };
    }
}

void ColorVision::begin() {
    pinMode(Pins::COLOR_S2, OUTPUT);
    pinMode(Pins::COLOR_S3, OUTPUT);
    pinMode(Pins::COLOR_OUT_LEFT,  INPUT);
    pinMode(Pins::COLOR_OUT_RIGHT, INPUT);

    // S0/S1 are strapped to 20% scaling in hardware (see Config.h) to save two
    // GPIOs. 20% keeps the output frequency inside what pulseIn can time
    // reliably while staying fast enough to read at 33 Hz.

    digitalWrite(Pins::COLOR_S2, LOW);
    digitalWrite(Pins::COLOR_S3, LOW);
}

uint16_t ColorVision::measureFilter(uint8_t outPin, bool s2, bool s3) const {
    digitalWrite(Pins::COLOR_S2, s2 ? HIGH : LOW);
    digitalWrite(Pins::COLOR_S3, s3 ? HIGH : LOW);
    delayMicroseconds(COLOR_SETTLE_US);

    const uint32_t halfPeriodUs = pulseIn(outPin, LOW, COLOR_PULSE_TIMEOUT_US);

    if (halfPeriodUs == 0) {
        return 0;   // Timed out: no light, or a dead sensor.
    }

    // Convert period to an intensity proxy. The TCS3200 outputs a frequency
    // proportional to irradiance, so a SHORT pulse means a BRIGHT reading —
    // working directly in period would invert every comparison downstream.
    uint32_t intensity = 500000UL / halfPeriodUs;   // ~kHz * 1000
    if (intensity > 65535UL) intensity = 65535UL;
    return static_cast<uint16_t>(intensity);
}

void ColorVision::update() {
    // Group by filter, not by sensor, so both heads see the same filter in the
    // same window. (measureFilter() re-drives S2/S3 and re-settles on every
    // call; that is harmless, just not the saving an older comment claimed.)

    // RED: S2=LOW,  S3=LOW
    _rawLeft.r  = measureFilter(Pins::COLOR_OUT_LEFT,  false, false);
    _rawRight.r = measureFilter(Pins::COLOR_OUT_RIGHT, false, false);

    // BLUE: S2=LOW, S3=HIGH
    _rawLeft.b  = measureFilter(Pins::COLOR_OUT_LEFT,  false, true);
    _rawRight.b = measureFilter(Pins::COLOR_OUT_RIGHT, false, true);

    // CLEAR: S2=HIGH, S3=LOW
    _rawLeft.c  = measureFilter(Pins::COLOR_OUT_LEFT,  true, false);
    _rawRight.c = measureFilter(Pins::COLOR_OUT_RIGHT, true, false);

    // GREEN: S2=HIGH, S3=HIGH
    _rawLeft.g  = measureFilter(Pins::COLOR_OUT_LEFT,  true, true);
    _rawRight.g = measureFilter(Pins::COLOR_OUT_RIGHT, true, true);

    // Both classifications land together, at the end of one complete filter
    // sweep. Only the Core-0 publisher reads them now, so a consumer can never
    // see a left head from this sweep paired with a right head from the last.
    // [A-C9] Reference capture: average both heads over COLOR_CAL_FRAMES
    // complete sweeps. A frame where either head timed out is skipped.
    if (_capturing >= 0 && _rawLeft.c != 0 && _rawRight.c != 0) {
        _capR += (static_cast<uint32_t>(_rawLeft.r) + _rawRight.r) / 2;
        _capG += (static_cast<uint32_t>(_rawLeft.g) + _rawRight.g) / 2;
        _capB += (static_cast<uint32_t>(_rawLeft.b) + _rawRight.b) / 2;
        _capC += (static_cast<uint32_t>(_rawLeft.c) + _rawRight.c) / 2;
        if (++_capFrames >= Mission::COLOR_CAL_FRAMES) {
            ChannelSet& ref = _reference[static_cast<uint8_t>(_capturing)];
            ref.r = static_cast<uint16_t>(_capR / _capFrames);
            ref.g = static_cast<uint16_t>(_capG / _capFrames);
            ref.b = static_cast<uint16_t>(_capB / _capFrames);
            ref.c = static_cast<uint16_t>(_capC / _capFrames);
            if (ref.c == 0) ref.c = 1;   // 0 means "never taught"
            _capturing = -1;
        }
    }

    _left  = classify(_rawLeft,  _reference);
    _right = classify(_rawRight, _reference);
}

void ColorVision::beginReference(TileColor color) {
    const uint8_t idx = static_cast<uint8_t>(color);
    if (idx == 0 || idx >= 6) return;   // UNKNOWN is not teachable
    _capR = _capG = _capB = _capC = 0;
    _capFrames = 0;
    _capturing = static_cast<int8_t>(idx);
}

bool ColorVision::referenceBusy() const { return _capturing >= 0; }

bool ColorVision::hasReference(TileColor color) const {
    const uint8_t idx = static_cast<uint8_t>(color);
    return idx > 0 && idx < 6 && _reference[idx].c != 0;
}

TileColor ColorVision::classify(const ChannelSet& sample,
                                const ChannelSet* references) {
    if (sample.c == 0) {
        return TileColor::UNKNOWN;   // Sensor timed out entirely.
    }

    // Normalise by the clear channel. This is what makes the classifier robust
    // to venue lighting: absolute intensity changes with ambient light, but the
    // ratio between channels is a property of the tile.
    const uint16_t nr = static_cast<uint16_t>(
        (static_cast<uint32_t>(sample.r) * COLOR_RATIO_SCALE) / sample.c);
    const uint16_t ng = static_cast<uint16_t>(
        (static_cast<uint32_t>(sample.g) * COLOR_RATIO_SCALE) / sample.c);
    const uint16_t nb = static_cast<uint16_t>(
        (static_cast<uint32_t>(sample.b) * COLOR_RATIO_SCALE) / sample.c);

    // --- Preferred path: nearest calibrated reference ----------------------
    // Only meaningful once the references were taught on the actual field
    // (CALIBRATING, via SensorsSystem::requestColorReference()).
    TileColor best      = TileColor::UNKNOWN;
    uint32_t  bestDist  = 0xFFFFFFFFUL;
    bool      haveRefs  = false;

    // [A-C9] Brightness dimension, relative to the taught WHITE reference.
    // Clear-normalised chroma alone puts black and white on the same point
    // (both grey), and puts a dark noisy black close to a dim red.
    const ChannelSet& white = references[static_cast<uint8_t>(TileColor::WHITE)];
    const bool haveBright = (white.c != 0);
    const int32_t nBright = haveBright
        ? static_cast<int32_t>((static_cast<uint32_t>(sample.c) * COLOR_RATIO_SCALE) / white.c)
        : 0;

    for (uint8_t i = 1; i < 6; ++i) {          // skip index 0 (UNKNOWN)
        const ChannelSet& ref = references[i];
        if (ref.c == 0) continue;              // this colour was never taught
        haveRefs = true;

        const int32_t rr = static_cast<int32_t>(
            (static_cast<uint32_t>(ref.r) * COLOR_RATIO_SCALE) / ref.c);
        const int32_t rg = static_cast<int32_t>(
            (static_cast<uint32_t>(ref.g) * COLOR_RATIO_SCALE) / ref.c);
        const int32_t rb = static_cast<int32_t>(
            (static_cast<uint32_t>(ref.b) * COLOR_RATIO_SCALE) / ref.c);

        const int32_t dr = static_cast<int32_t>(nr) - rr;
        const int32_t dg = static_cast<int32_t>(ng) - rg;
        const int32_t db = static_cast<int32_t>(nb) - rb;

        uint32_t dist = static_cast<uint32_t>(dr * dr + dg * dg + db * db);
        if (haveBright) {
            const int32_t rBright = static_cast<int32_t>(
                (static_cast<uint32_t>(ref.c) * COLOR_RATIO_SCALE) / white.c);
            const int32_t dl = nBright - rBright;
            dist += static_cast<uint32_t>(dl * dl) >> COLOR_BRIGHT_SHIFT;
        }
        if (dist < bestDist) {
            bestDist = dist;
            best     = static_cast<TileColor>(i);
        }
    }

    if (haveRefs) {
        // [A-C9] Nearest neighbour ALWAYS returns something; a sample that is
        // near nothing taught (a head half on a marker edge, a shadow) must
        // come back UNKNOWN, not as whichever colour happened to be closest.
        return (bestDist <= COLOR_REJECT_DIST2) ? best : TileColor::UNKNOWN;
    }

    // --- Fallback: ratio heuristics ---------------------------------------
    // Used only before field calibration, so the robot is testable on the
    // bench. Do NOT run a scoring match on this path — the thresholds below
    // are guesses about a floor this code has never seen.
    if (ng > nr + 120 && ng > nb + 120) return TileColor::GREEN;
    if (nr > ng + 120 && nr > nb + 100) return TileColor::RED;
    if (sample.c < 400)                 return TileColor::BLACK;
    if (nr > 250 && ng > 250 && nb > 250) return TileColor::WHITE;
    return TileColor::UNKNOWN;
}

TileColor ColorVision::left()  const { return _left;  }
TileColor ColorVision::right() const { return _right; }

TurnDirection ColorVision::interpretMarkers() const {
    const bool greenLeft  = (_left  == TileColor::GREEN);
    const bool greenRight = (_right == TileColor::GREEN);

    // Rulebook: two markers, one either side, means a dead end — turn around.
    //
    // @note U_TURN is returned ONLY while both heads are green. The mission
    //       layer must therefore latch this hint at the moment green begins,
    //       not after green ends — testing for U_TURN once the heads have
    //       gone white is a condition that can never be true. See
    //       TaskHandler::stepLineFollow()'s g_greenHint. [FIX 5]
    if (greenLeft && greenRight) return TurnDirection::U_TURN;
    if (greenLeft)               return TurnDirection::LEFT;
    if (greenRight)              return TurnDirection::RIGHT;
    return TurnDirection::STRAIGHT;
}

bool ColorVision::bothAgree(TileColor color) const {
    return (_left == color) && (_right == color);
}

uint16_t ColorVision::rawRed(bool rightSensor) const {
    return rightSensor ? _rawRight.r : _rawLeft.r;
}
uint16_t ColorVision::rawGreen(bool rightSensor) const {
    return rightSensor ? _rawRight.g : _rawLeft.g;
}
uint16_t ColorVision::rawBlue(bool rightSensor) const {
    return rightSensor ? _rawRight.b : _rawLeft.b;
}


// ============================================================================
//  SensorsSystem — facade and snapshot publisher
// ============================================================================

SensorsSystem::SensorsSystem()
    : _mutex(nullptr), _healthy(false),
      _encoderYawRateDegS(0.0f), _encoderYawValid(false),
      _imuCalReq(false), _colorReq(-1), _colorBusy(false) {
    _snapshot = SensorSnapshot();
}

bool SensorsSystem::begin() {
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) {
        return false;
    }

    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL, Hw::I2C_CLOCK_HZ);

    // Order matters. The ToF bank must be addressed before anything else scans
    // the bus, because until bringUpSensor() has run, three devices are all
    // sitting on 0x29.
    const bool lidarOk = _lidar.begin();
    const bool imuOk   = _imu.begin();

    _line.begin();
    _vision.begin();

    _healthy = lidarOk && imuOk;
    return _healthy;
}

void SensorsSystem::updateSlowSensors(float dtSec) {
    // [A-H1] Posted IMU bias request runs HERE, on the task that owns the IMU.
    // Blocking (~0.5-1 s, yields every sample); only ever requested while the
    // robot is stationary in CALIBRATING.
    if (_imuCalReq) {
        _imu.calibrateBias(512);
        _imuCalReq = false;
    }

    _imu.update(dtSec);

    // Non-blocking: collects every VL53L1X that has a fresh sample (or only
    // the focused one) and applies posted reconfiguration. Never waits on a
    // conversion — see LidarBank.cpp.
    _lidar.updateNext();

    // [A-C9] Posted colour reference: start the capture, let update() average.
    if (_colorReq >= 0 && !_vision.referenceBusy()) {
        _vision.beginReference(static_cast<TileColor>(_colorReq));
        _colorReq = -1;
    }

    _vision.update();

    if (_colorBusy && _colorReq < 0 && !_vision.referenceBusy()) {
        _colorBusy = false;
    }

    publish();
}

void SensorsSystem::updateLineArray() {
    _line.read();
    publishLine();   // [FIX 8] line fields ONLY — see the note at the top
}

void SensorsSystem::setEncoderYawRate(float degS) {
    // Two naturally-atomic 32-bit stores, no mutex. The control task on Core 1
    // must never be able to block on the sensor task, and a one-tick-old yaw
    // rate is harmless for a slip check with a 22 deg/s tolerance.
    _encoderYawRateDegS = degS;
    _encoderYawValid    = true;
}

void SensorsSystem::publish() {
    if (_mutex == nullptr) return;

    // CORE 0 FIELDS ONLY. Everything read here comes from a sensor object this
    // task owns. The line fields belong to publishLine() on Core 1 and are not
    // touched — that disjointness is what removes the cross-core race, not the
    // mutex, which only serialises writes to _snapshot itself.
    //
    // Nothing that can block belongs in here — the control task on Core 1 is
    // waiting on this mutex with a 2 ms budget.
    // [A-C5] Wait up to ONE tick. With a zero timeout, a reader holding the
    // mutex dropped the whole colour frame — and a 25 mm marker at cruise only
    // yields ~3 frames. This task has 30 ms of slack; a 1 ms wait is free.
    if (xSemaphoreTake(_mutex, 1) != pdTRUE) {
        return;
    }

    _snapshot.yawDeg       = _imu.yawDeg();
    _snapshot.imuHealthy   = _imu.isHealthy();
    _snapshot.yawRateDegS  = _imu.yawRateDegS();
    _snapshot.pitchDeg     = _imu.pitchDeg();
    _snapshot.onRamp       = _imu.isOnRamp();
    _snapshot.bumpDetected = _imu.isBumpDetected();   // [FIX 8] its own field

    // [FIX 8] A real slip check instead of the bump latch wearing slip's name.
    // Stays false until the control loop starts feeding the encoder yaw rate.
    _snapshot.slipDetected = _encoderYawValid
                                 ? _imu.detectSlip(_encoderYawRateDegS)
                                 : false;

    _snapshot.rangeFrontMm  = _lidar.rangeMm(LidarId::FRONT);
    _snapshot.rangeLeftMm   = _lidar.rangeMm(LidarId::LEFT);
    _snapshot.rangeRightMm  = _lidar.rangeMm(LidarId::RIGHT);
    _snapshot.obstacleAhead = _lidar.obstacleAhead();
    _snapshot.rangeFrontStampMs = _lidar.stampMs(LidarId::FRONT);

    _snapshot.colorLeft  = _vision.left();
    _snapshot.colorRight = _vision.right();
    _snapshot.markerHint = _vision.interpretMarkers();

    _snapshot.timestampMs       = millis();
    // [A-V4] Live, not the boot-time verdict.
    _snapshot.allSensorsHealthy = _imu.isHealthy() && _lidar.isUp(LidarId::FRONT);

    xSemaphoreGive(_mutex);
}

void SensorsSystem::publishLine() {
    if (_mutex == nullptr) return;

    // CORE 1 FIELDS ONLY. Six stores and a timestamp. This runs at 200 Hz, so
    // it must stay this small, and it must not read _imu / _lidar / _vision —
    // those live on the other core and are mutated without this mutex.
    if (xSemaphoreTake(_mutex, 0) != pdTRUE) {
        return;   // A reader holds it; the next 5 ms tick will publish.
    }

    _snapshot.linePosition          = _line.readPosition();
    _snapshot.lineMask              = _line.digitalMask();
    _snapshot.lineActiveCount       = _line.activeCount();
    _snapshot.lineLost              = _line.isLineLost();
    _snapshot.intersectionCandidate = _line.isIntersectionCandidate();
    _snapshot.rearOnLine            = _line.isRearOnLine();
    _snapshot.lineTimestampMs       = millis();

    xSemaphoreGive(_mutex);
}

bool SensorsSystem::getSnapshot(SensorSnapshot& out, uint32_t timeoutMs) const {
    if (_mutex == nullptr) return false;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
        // Caller keeps its previous copy. Missing a control deadline is worse
        // than acting on a 5 ms old sample.
        return false;
    }

    out = _snapshot;
    xSemaphoreGive(_mutex);
    return true;
}


// ============================================================================
//  Calibration passthroughs
//
//  @warning Every function below drives LineArray or Imu from the MISSION
//           task. TaskHandler::lineLoop() gates the Core-1 line task off while
//           the state is CALIBRATING, which is what makes this safe. Do not
//           call any of these from another state.
// ============================================================================

void SensorsSystem::beginLineCalibration()  { _line.calibrateReset(); }
void SensorsSystem::sampleLineCalibration() { _line.calibrateSample(); }

void SensorsSystem::endLineCalibration() {
    _line.calibrateFinish();   // [FIX 7] declare the sweep over FIRST...
    _line.read();              // ...then let read() publish its verdict.
}

void SensorsSystem::requestImuBiasCalibration() { _imuCalReq = true; }
bool SensorsSystem::imuCalibrationBusy() const { return _imuCalReq; }

void SensorsSystem::requestColorReference(TileColor color) {
    const uint8_t idx = static_cast<uint8_t>(color);
    if (idx == 0 || idx >= 6) return;
    _colorBusy = true;                       // busy FIRST, so a poller that
    _colorReq  = static_cast<int8_t>(idx);   // sees the request sees busy
}
bool SensorsSystem::colorReferenceBusy() const { return _colorBusy; }

bool SensorsSystem::colorCalibrated() const {
    return _vision.hasReference(TileColor::WHITE) &&
           _vision.hasReference(TileColor::BLACK) &&
           _vision.hasReference(TileColor::GREEN) &&
           _vision.hasReference(TileColor::RED);
}

bool SensorsSystem::isFullyCalibrated() const {
    return _line.isCalibrated() && _healthy && _imu.isCalibrated()
#if SYROBIX_REQUIRE_COLOR_CAL
           && colorCalibrated()
#endif
           ;
}

LineArray&   SensorsSystem::lineArray() { return _line;   }
Imu&         SensorsSystem::imu()       { return _imu;    }
LidarBank&   SensorsSystem::lidar()     { return _lidar;  }
ColorVision& SensorsSystem::vision()    { return _vision; }
