/**
 * ============================================================================
 *  @file       RobotDrivetrain.cpp
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Implementation of PidController, QuadratureEncoder, MotorSide
 *              and RobotDrivetrain.
 *
 *  @target     Arduino-ESP32 core 2.0.x  (ledcSetup / ledcAttachPin / ledcWrite)
 *              On core 3.x these three collapse into ledcAttach(pin,freq,res)
 *              and ledcWrite(pin,duty). See MotorSide::begin().
 *
 *  ---------------------------------------------------------------------------
 *  FOUR DEVIATIONS FROM THE BRIEF — read before integrating
 *  ---------------------------------------------------------------------------
 *  1. OMNI KINEMATICS ARE NOT IMPLEMENTED, and cannot be on this hardware.
 *     Omni/mecanum kinematics need four INDEPENDENTLY commandable wheels plus
 *     roller wheels. Config.h gangs PWM and direction per side across both
 *     TB6612FNGs, so the electrical system exposes exactly two commandable
 *     units, and the parts list has no omni wheels. What is implemented is
 *     correct skid-steer: forward, reverse, arc, and zero-radius pivot. If you
 *     genuinely want strafing, that is a chassis and pin-map change first —
 *     four PWM pins, four direction pairs, and omni wheels — not a maths change.
 *
 *  2. THE 8x TCR5000 READ IS NOT IN THIS FILE. Per the headers, line sensing
 *     belongs to LineArray in SensorsSystem.h, and it is sampled on Core 1 by
 *     lineTask. Implementing it here would put an ADC sweep inside the
 *     drivetrain and break the one-producer rule. It is delivered as
 *     LineArray.cpp instead — same commit, correct owner.
 *
 *  3. THE +/-4000 ERROR SCALE IS COMPUTED, THEN NORMALISED. LineArray::
 *     readPosition() is declared to return -1.0..+1.0, and the brief says to
 *     use only what the headers define. So the classic weighted-centroid
 *     +/-4000 value is computed internally and divided by 4000.0f on the way
 *     out. Nothing is lost: PID gains in Tune:: are already expressed against
 *     the normalised scale.
 *
 *  4. THERE IS NO runPID(). The headers name it followLine() on the drivetrain
 *     and update() on PidController; that pair is the PID entry point. It is
 *     millis()-driven and fully non-blocking, as asked.
 * ============================================================================
 */

#include "RobotDrivetrain.h"

#include <Arduino.h>
#include <math.h>

// ============================================================================
//  File-local helpers
// ============================================================================

namespace {

/** Spinlock guarding the 32-bit encoder counters against a torn cross-core read. */
portMUX_TYPE g_encoderMux = portMUX_INITIALIZER_UNLOCKED;

/** LEDC channel allocation. Channels 2..7 remain free for the servos. */
// [A-H2] Channels come from Config.h (Hw::LEDC_CH_MOTOR_*), where the timer-
// collision static_asserts can actually see them.

// LEDC API compatibility: core 2.x addresses LEDC by channel, core 3.x by pin.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define SYROBIX_LEDC_V3 1
#else
  #define SYROBIX_LEDC_V3 0
#endif

/** x4 quadrature decode table, indexed by (previousState << 2) | currentState. */
constexpr int8_t QUAD_LUT[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/** Previous A/B state per encoder instance, parallel to QuadratureEncoder::_instances. */
volatile uint8_t g_prevState[4] = { 0, 0, 0, 0 };

/** Wraps an angle into [-180, +180). */
inline float wrapDeg(float deg) {
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

inline float clampF(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

} // anonymous namespace


// ============================================================================
//  PidController
// ============================================================================

PidController::PidController(float kp, float ki, float kd,
                             float outMin, float outMax,
                             float derivativeAlpha)
    : _kp(kp), _ki(ki), _kd(kd),
      _outMin(outMin), _outMax(outMax),
      _alpha(clampF(derivativeAlpha, 0.01f, 1.0f)),
      _integral(0.0f),
      _prevError(0.0f),
      _filteredDeriv(0.0f),
      _primed(false) {}

float PidController::update(float error, float dtSec) {
    // A zero or negative dt would divide by zero in the derivative term. This
    // happens in practice on the first tick after a task starts.
    if (dtSec <= 0.0f) {
        dtSec = 1e-3f;
    }

    const float p = _kp * error;

    // --- Derivative, low-pass filtered ------------------------------------
    // The first sample after a reset has no meaningful history: taking a
    // derivative against _prevError == 0 produces a large spurious kick that
    // shows up as a lurch every time the state machine changes state.
    float rawDeriv = 0.0f;
    if (_primed) {
        rawDeriv = (error - _prevError) / dtSec;
    } else {
        _primed = true;
    }
    _filteredDeriv += _alpha * (rawDeriv - _filteredDeriv);
    const float d = _kd * _filteredDeriv;

    // --- Integral with conditional anti-windup ----------------------------
    // Integrate first, then check whether the unsaturated output would leave
    // the actuator range. If it would, roll the integration back rather than
    // letting the term keep growing against a saturated output.
    float i = 0.0f;
    if (_ki != 0.0f) {
        const float candidate = _integral + error * dtSec;
        const float trial = p + (_ki * candidate) + d;
        if (trial > _outMin && trial < _outMax) {
            _integral = candidate;
        }
        i = _ki * _integral;
    }

    _prevError = error;
    return clampF(p + i + d, _outMin, _outMax);
}

void PidController::reset() {
    _integral      = 0.0f;
    _prevError     = 0.0f;
    _filteredDeriv = 0.0f;
    _primed        = false;
}

void PidController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    // Dropping the accumulated integral is deliberate: it was earned under the
    // old gains and is meaningless under the new ones.
    _integral = 0.0f;
}

float PidController::lastError() const { return _prevError; }
float PidController::integral()  const { return _integral; }


// ============================================================================
//  QuadratureEncoder
// ============================================================================

QuadratureEncoder* QuadratureEncoder::_instances[4] = { nullptr, nullptr, nullptr, nullptr };
uint8_t            QuadratureEncoder::_instanceCount = 0;

QuadratureEncoder::QuadratureEncoder(uint8_t pinA, uint8_t pinB, bool inverted)
    : _pinA(pinA), _pinB(pinB), _inverted(inverted),
      _count(0), _lastCount(0) {}

void QuadratureEncoder::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    if (_instanceCount >= 4) {
        return; // Dispatch table full; silently refuse rather than corrupt it.
    }

    const uint8_t slot = _instanceCount++;
    _instances[slot] = this;

    // Seed the decoder with the current line state so the very first edge is
    // decoded against reality instead of against zero.
    g_prevState[slot] = static_cast<uint8_t>((digitalRead(_pinA) << 1) | digitalRead(_pinB));

    void (*handler)() = nullptr;
    switch (slot) {
        case 0: handler = isrTrampoline0; break;
        case 1: handler = isrTrampoline1; break;
        case 2: handler = isrTrampoline2; break;
        case 3: handler = isrTrampoline3; break;
    }

    // CHANGE on both lines gives x4 resolution, which is what Hw::ENCODER_CPR
    // assumes.
    attachInterrupt(digitalPinToInterrupt(_pinA), handler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), handler, CHANGE);

    reset();
}

void IRAM_ATTR QuadratureEncoder::handleEdge() {
    // Resolve this instance's slot. A linear scan of at most four pointers is
    // a handful of cycles and keeps the header's ISR signature intact; the
    // tidier fix is a _prevState member on the class.
    uint8_t slot = 0;
    for (uint8_t i = 0; i < _instanceCount; ++i) {
        if (_instances[i] == this) { slot = i; break; }
    }

    const uint8_t curr = static_cast<uint8_t>(
        (digitalRead(_pinA) << 1) | digitalRead(_pinB));

    const int8_t delta = QUAD_LUT[(g_prevState[slot] << 2) | curr];
    g_prevState[slot] = curr;

    if (delta != 0) {
        _count += _inverted ? -delta : delta;
    }
    // delta == 0 means an illegal transition: both lines changed between
    // samples. Ignoring it is correct — it is a missed edge, not a direction.
}

void IRAM_ATTR QuadratureEncoder::isrTrampoline0() { if (_instances[0]) _instances[0]->handleEdge(); }
void IRAM_ATTR QuadratureEncoder::isrTrampoline1() { if (_instances[1]) _instances[1]->handleEdge(); }
void IRAM_ATTR QuadratureEncoder::isrTrampoline2() { if (_instances[2]) _instances[2]->handleEdge(); }
void IRAM_ATTR QuadratureEncoder::isrTrampoline3() { if (_instances[3]) _instances[3]->handleEdge(); }

int32_t QuadratureEncoder::counts() const {
    portENTER_CRITICAL(&g_encoderMux);
    const int32_t snapshot = _count;
    portEXIT_CRITICAL(&g_encoderMux);
    return snapshot;
}

float QuadratureEncoder::distanceMm() const {
    return static_cast<float>(counts()) * Hw::MM_PER_COUNT;
}

float QuadratureEncoder::velocityMmPerSec(float dtSec) {
    if (dtSec <= 0.0f) {
        return 0.0f;
    }
    const int32_t now   = counts();
    const int32_t delta = now - _lastCount;
    _lastCount = now;
    return (static_cast<float>(delta) * Hw::MM_PER_COUNT) / dtSec;
}

void QuadratureEncoder::reset() {
    portENTER_CRITICAL(&g_encoderMux);
    _count = 0;
    portEXIT_CRITICAL(&g_encoderMux);
    _lastCount = 0;
}


// ============================================================================
//  MotorSide
// ============================================================================

MotorSide::MotorSide(uint8_t pinPwm, uint8_t pinIn1, uint8_t pinIn2,
                     uint8_t ledcChannel, bool inverted)
    : _pinPwm(pinPwm), _pinIn1(pinIn1), _pinIn2(pinIn2),
      _ledcChannel(ledcChannel), _inverted(inverted), _pwm(0) {}

void MotorSide::begin() {
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);

    // 20 kHz at 10-bit resolution: the LEDC divider works out cleanly on the
    // 80 MHz APB clock, and the carrier sits above the audible band so the
    // gearmotors stay quiet at low duty.
#if SYROBIX_LEDC_V3
    // Explicit channel so the timer allocation matches Config.h and cannot
    // be auto-assigned elsewhere. (Rev 3: the only native LEDC users left are
    // these two motor channels; servos and buzzer live on the PCA9685.)
    ledcAttachChannel(_pinPwm, Hw::PWM_FREQ_HZ, Hw::PWM_RESOLUTION, _ledcChannel);
#else
    ledcSetup(_ledcChannel, Hw::PWM_FREQ_HZ, Hw::PWM_RESOLUTION);
    ledcAttachPin(_pinPwm, _ledcChannel);
#endif

    coast();
}

void MotorSide::setPwm(int16_t pwm) {
    if (_inverted) {
        pwm = static_cast<int16_t>(-pwm);
    }

    // Clamp before anything else so the stiction logic below can assume range.
    if (pwm >  Hw::PWM_MAX) pwm =  Hw::PWM_MAX;
    if (pwm < -Hw::PWM_MAX) pwm = -Hw::PWM_MAX;

    if (pwm == 0) {
        _pwm = 0;
        digitalWrite(_pinIn1, LOW);
        digitalWrite(_pinIn2, LOW);
        ledcWrite(ledcTarget(), 0);
        return;
    }

    // Stiction floor. Below roughly 18% duty these gearmotors sit buzzing
    // without turning, which the PID reads as a dead zone and integrates
    // against. Boosting to the floor keeps the actuator monotonic.
    int16_t magnitude = (pwm > 0) ? pwm : static_cast<int16_t>(-pwm);
    if (magnitude < Hw::PWM_MIN_MOVE) {
        magnitude = Hw::PWM_MIN_MOVE;
    }

    const bool forward = (pwm > 0);
    digitalWrite(_pinIn1, forward ? HIGH : LOW);
    digitalWrite(_pinIn2, forward ? LOW  : HIGH);
    ledcWrite(ledcTarget(), static_cast<uint32_t>(magnitude));

    _pwm = forward ? magnitude : static_cast<int16_t>(-magnitude);
}

void MotorSide::brake() {
    // TB6612FNG short-brake: both inputs high, PWM high.
    digitalWrite(_pinIn1, HIGH);
    digitalWrite(_pinIn2, HIGH);
    ledcWrite(ledcTarget(), Hw::PWM_MAX);
    _pwm = 0;
}

void MotorSide::coast() {
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);
    ledcWrite(ledcTarget(), 0);
    _pwm = 0;
}

int16_t MotorSide::currentPwm() const { return _pwm; }

uint8_t MotorSide::ledcTarget() const {
#if SYROBIX_LEDC_V3
    return _pinPwm;        // 3.x: ledcWrite(pin, duty)
#else
    return _ledcChannel;   // 2.x: ledcWrite(channel, duty)
#endif
}


// ============================================================================
//  RobotDrivetrain
// ============================================================================

RobotDrivetrain::RobotDrivetrain()
    : _left(Pins::MOTOR_PWM_LEFT,  Pins::MOTOR_LEFT_IN1,  Pins::MOTOR_LEFT_IN2,
            Hw::LEDC_CH_MOTOR_LEFT,  false),
      _right(Pins::MOTOR_PWM_RIGHT, Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2,
            Hw::LEDC_CH_MOTOR_RIGHT, true),   // mirrored mounting
      _encLeft(Pins::ENCODER_LEFT_A,  Pins::ENCODER_LEFT_B,  false),
      _encRight(Pins::ENCODER_RIGHT_A, Pins::ENCODER_RIGHT_B, true),
      _linePid(Tune::LINE_KP, Tune::LINE_KI, Tune::LINE_KD,
               -static_cast<float>(Hw::PWM_MAX), static_cast<float>(Hw::PWM_MAX),
               Tune::LINE_D_FILTER_ALPHA),
      _lineLastMs(0),
      _headingDeg(0.0f),
      _leftVelMmS(0.0f),
      _rightVelMmS(0.0f) {}

bool RobotDrivetrain::begin() {
    pinMode(Pins::MOTOR_STBY, OUTPUT);
    digitalWrite(Pins::MOTOR_STBY, LOW);   // stay in standby through bring-up

    _left.begin();
    _right.begin();
    _encLeft.begin();
    _encRight.begin();

    _linePid.reset();
    resetOdometry();

    digitalWrite(Pins::MOTOR_STBY, HIGH);  // drivers live
    return true;
}


// ----------------------------------------------------------------------------
//  Open-loop primitives — skid-steer kinematics
// ----------------------------------------------------------------------------

void RobotDrivetrain::tank(int16_t leftPwm, int16_t rightPwm) {
#if SYROBIX_DRY_RUN
    (void)leftPwm; (void)rightPwm;
    return;
#else
    _left.setPwm(leftPwm);
    _right.setPwm(rightPwm);
#endif
}

void RobotDrivetrain::pivot(int16_t pwm, TurnDirection dir) {
    const int16_t magnitude = (pwm > 0) ? pwm : static_cast<int16_t>(-pwm);

    switch (dir) {
        case TurnDirection::LEFT:
            // Zero-radius: right side forward, left side reverse. The chassis
            // rotates about its own centre and does not translate, which is
            // what the victim-alignment sequence depends on.
            tank(static_cast<int16_t>(-magnitude), magnitude);
            break;

        case TurnDirection::RIGHT:
            tank(magnitude, static_cast<int16_t>(-magnitude));
            break;

        case TurnDirection::U_TURN:
            // Direction is arbitrary for a 180; right-hand is chosen so the
            // behaviour is repeatable for the operator.
            tank(magnitude, static_cast<int16_t>(-magnitude));
            break;

        case TurnDirection::STRAIGHT:
            tank(magnitude, magnitude);
            break;

        case TurnDirection::NONE:
        default:
            brake();
            break;
    }
}

void RobotDrivetrain::brake() {
    _left.brake();
    _right.brake();
}

void RobotDrivetrain::coast() {
    _left.coast();
    _right.coast();
}

void RobotDrivetrain::emergencyStop() {
    // Cut the drivers at the hardware level first, then tidy the software
    // state. Order matters: STBY low is the part that satisfies the rulebook.
    digitalWrite(Pins::MOTOR_STBY, LOW);
    _left.coast();
    _right.coast();
    resetLinePid();
}


// ----------------------------------------------------------------------------
//  Closed-loop requests
// ----------------------------------------------------------------------------

// [A-C1] requestRotation/requestDistance/abortMotion removed — see header.


// ----------------------------------------------------------------------------
//  Line following — the PID entry point
// ----------------------------------------------------------------------------

void RobotDrivetrain::followLine(float normalisedError, int16_t baseSpeed) {
    // dt is measured, not assumed. The control task nominally ticks at
    // Rtos::PERIOD_CONTROL_MS, but a preempting higher-priority task will
    // stretch that, and a PID fed a nominal dt during a stretched tick
    // mis-scales both its integral and its derivative.
    // [A-C2] Member, not a function-local static: resetLinePid() must be able
    // to clear it, or the first tick after a pause sees dt clamped to 100 ms
    // against a stale _prevError and kicks the derivative.
    const uint32_t nowMs = millis();
    float dtSec = (_lineLastMs == 0) ? (Rtos::PERIOD_CONTROL_MS / 1000.0f)
                                     : (nowMs - _lineLastMs) / 1000.0f;
    _lineLastMs = nowMs;

    // Guard against a stall: if this task was starved for a long time, a huge
    // dt would produce an enormous integral step. Clamp rather than trust it.
    if (dtSec <= 0.0f)  dtSec = 1e-3f;
    if (dtSec > 0.100f) dtSec = 0.100f;

    float correction = _linePid.update(normalisedError, dtSec);

    // [A-C2] Cap the inner wheel's reverse effort. The correction may pull a
    // side below zero (needed on sharp corners) but never past
    // -LINE_MAX_REVERSE_PWM, so one saturated tick cannot slam a side into
    // full reverse.
    const float lim = static_cast<float>(baseSpeed) +
                      static_cast<float>(Tune::LINE_MAX_REVERSE_PWM);
    if (correction >  lim) correction =  lim;
    if (correction < -lim) correction = -lim;

    // Positive error means the line has drifted to the RIGHT of centre, so the
    // robot must turn right: slow the right side, speed the left.
    const int16_t leftPwm  = clampPwm(static_cast<float>(baseSpeed) + correction);
    const int16_t rightPwm = clampPwm(static_cast<float>(baseSpeed) - correction);

    tank(leftPwm, rightPwm);
}

void RobotDrivetrain::resetLinePid() {
    _linePid.reset();
    _lineLastMs = 0;
}

void RobotDrivetrain::setLineGains(float kp, float ki, float kd) {
    _linePid.setGains(kp, ki, kd);
}


// ----------------------------------------------------------------------------
//  Odometry
// ----------------------------------------------------------------------------

float RobotDrivetrain::distanceTravelledMm() const {
    return 0.5f * (_encLeft.distanceMm() + _encRight.distanceMm());
}

float RobotDrivetrain::headingDeg()      const { return _headingDeg; }
float RobotDrivetrain::leftVelocityMmS() const { return _leftVelMmS; }
float RobotDrivetrain::rightVelocityMmS()const { return _rightVelMmS; }

void RobotDrivetrain::resetOdometry() {
    _encLeft.reset();
    _encRight.reset();
    _headingDeg  = 0.0f;
    _leftVelMmS  = 0.0f;
    _rightVelMmS = 0.0f;
}


// ----------------------------------------------------------------------------
//  Service tick
// ----------------------------------------------------------------------------

void RobotDrivetrain::update(float dtSec) {
    integrateOdometry(dtSec);

    // [A-C1] No closed-loop request servicing any more: motor writes come only
    // from followLine() or the drive mailbox, both on this task.
}

void RobotDrivetrain::integrateOdometry(float dtSec) {
    // Sample both wheels exactly once per tick: velocityMmPerSec() consumes
    // the delta, so a second call in the same tick reads zero.
    _leftVelMmS  = _encLeft.velocityMmPerSec(dtSec);
    _rightVelMmS = _encRight.velocityMmPerSec(dtSec);

    const float dLeft  = _leftVelMmS  * dtSec;
    const float dRight = _rightVelMmS * dtSec;

    // Differential-drive heading: the arc-length difference between the two
    // tracks divided by the track width gives the yaw increment in radians.
    // @warning On a 4WD skid-steer the EFFECTIVE track width is 1.3-2x the
    // geometric one (lateral scrub), so this heading under-reports rotation
    // by 30-50 %. It is used only as a fallback when the IMU is unhealthy.
    const float dThetaRad = (dRight - dLeft) / Hw::TRACK_WIDTH_MM;
    _headingDeg = wrapDeg(_headingDeg + dThetaRad * 57.2957795f);
}

int16_t RobotDrivetrain::clampPwm(float value) {
    if (value >  static_cast<float>(Hw::PWM_MAX)) return  Hw::PWM_MAX;
    if (value < -static_cast<float>(Hw::PWM_MAX)) return -Hw::PWM_MAX;
    return static_cast<int16_t>(value);
}
