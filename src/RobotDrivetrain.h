/**
 * ============================================================================
 *  @file       RobotDrivetrain.h
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Skid-steer drivetrain: PWM output, quadrature odometry,
 *              velocity/heading PID, and closed-loop motion primitives.
 *
 *  @warning    Every method in this header is intended to be called from the
 *              Core-1 control task ONLY. The class performs no I2C, no dynamic
 *              allocation, and no blocking waits, so its update() is safe to
 *              run at 200 Hz. [A-C1] EVERY method that writes a motor is
 *              Core-1 control-task only. Other tasks command motion through
 *              the drive mailbox in TaskHandler.cpp. distanceTravelledMm() and
 *              headingDeg() are safe cross-core READS.
 * ============================================================================
 */

#pragma once

#include <stdint.h>
#include <esp_attr.h>   // [A-V4] IRAM_ATTR. Was missing: RobotDrivetrain.cpp
                         // and TaskHandler.cpp include this header BEFORE
                         // <Arduino.h>, so IRAM_ATTR was undefined there.
#include "Config.h"

// ============================================================================
//  PidController — reusable scalar PID with derivative filtering and clamping
// ============================================================================

/**
 * @brief Fixed-form PID with output clamping, integral anti-windup and a
 *        first-order low-pass on the derivative term.
 *
 * The derivative filter matters here: the TCR5000 array produces a quantised,
 * noisy position estimate, and an unfiltered D term at 200 Hz will make the
 * chassis chatter across the line rather than track it.
 */
class PidController {
public:
    PidController(float kp, float ki, float kd,
                  float outMin, float outMax,
                  float derivativeAlpha = 1.0f);

    /**
     * @param error    Setpoint minus measurement, in the caller's own units.
     * @param dtSec    Elapsed time since the previous call. Must be > 0.
     * @return         Clamped controller output.
     */
    float update(float error, float dtSec);

    /** Clears the integrator and derivative history. Call on every state change. */
    void  reset();

    /** Live retune, used by the telemetry task during field calibration. */
    void  setGains(float kp, float ki, float kd);

    float lastError() const;
    float integral()  const;

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax;
    float _alpha;            ///< Derivative low-pass coefficient (0..1]
    float _integral;
    float _prevError;
    float _filteredDeriv;
    bool  _primed;           ///< Suppresses the first derivative spike
};


// ============================================================================
//  QuadratureEncoder — ISR-backed counter for one instrumented wheel
// ============================================================================

/**
 * @brief x4-decoded quadrature counter.
 *
 * @details The count is written from an IRAM ISR and read from the control
 *          task, so it is declared volatile and read through a critical
 *          section (portENTER_CRITICAL) to avoid a torn 32-bit read on a
 *          preempting core.
 *
 *          Instances self-register in a static table so that the static ISR
 *          trampolines can dispatch to the right object without std::function
 *          or heap allocation.
 */
class QuadratureEncoder {
public:
    QuadratureEncoder(uint8_t pinA, uint8_t pinB, bool inverted = false);

    /** Configures pins, attaches both edge interrupts, zeroes the count. */
    void begin();

    /** @return Raw accumulated counts since the last reset. */
    int32_t counts() const;

    /** @return Distance travelled by this wheel, in millimetres. */
    float distanceMm() const;

    /**
     * @brief  Instantaneous wheel speed.
     * @param  dtSec Interval since the previous call to this method.
     * @return Millimetres per second, sign-carrying.
     */
    float velocityMmPerSec(float dtSec);

    void reset();

private:
    const uint8_t _pinA, _pinB;
    const bool    _inverted;
    volatile int32_t _count;
    int32_t _lastCount;

    /** @brief IRAM-resident edge handler. Kept branch-minimal by design. */
    void IRAM_ATTR handleEdge();

    // --- Static dispatch table (no heap, ISR-safe) --------------------------
    static QuadratureEncoder* _instances[4];
    static uint8_t            _instanceCount;
    static void IRAM_ATTR     isrTrampoline0();
    static void IRAM_ATTR     isrTrampoline1();
    static void IRAM_ATTR     isrTrampoline2();
    static void IRAM_ATTR     isrTrampoline3();
};


// ============================================================================
//  MotorSide — one electrically-ganged pair of TB6612FNG channels
// ============================================================================

/**
 * @brief Abstracts the front+rear motor pair on a single side of the chassis.
 *
 * @details Because the PWM and direction lines are shared between the two
 *          drivers (see the pin-budget note in Config.h), a side is the
 *          smallest independently commandable unit. This is intentional: in
 *          skid-steer, commanding co-side wheels differently only fights the
 *          ground.
 */
class MotorSide {
public:
    MotorSide(uint8_t pinPwm, uint8_t pinIn1, uint8_t pinIn2,
              uint8_t ledcChannel, bool inverted = false);

    void begin();

    /**
     * @brief Applies a signed effort.
     * @param pwm Signed PWM, clamped to +/- Hw::PWM_MAX. Magnitudes below
     *            Hw::PWM_MIN_MOVE are boosted to that floor (or zeroed if the
     *            request is itself zero) so the gearmotors never sit humming
     *            in stiction.
     */
    void setPwm(int16_t pwm);

    /** Active short-brake across the motor terminals (IN1 = IN2 = HIGH). */
    void brake();

    /** High-impedance coast (IN1 = IN2 = LOW). */
    void coast();

    int16_t currentPwm() const;

private:
    /** LEDC write target: channel on core 2.x, pin on core 3.x. */
    uint8_t ledcTarget() const;

private:
    const uint8_t _pinPwm, _pinIn1, _pinIn2, _ledcChannel;
    const bool    _inverted;
    int16_t       _pwm;
};


// ============================================================================
//  RobotDrivetrain — public motion API
// ============================================================================

/**
 * @brief Owns both motor sides, both encoders, the line PID and odometry.
 *
 * @details Only the control task calls the writing methods. The mission layer
 *          posts tank/brake/coast commands to the drive mailbox, which the
 *          control task applies; rotations are closed on IMU yaw in
 *          TaskHandler. Nothing in the mission layer spin-waits on a motor.
 */
class RobotDrivetrain {
public:
    RobotDrivetrain();

    /** Configures LEDC channels, encoder interrupts and the STBY line. */
    bool begin();

    // ------------------------------------------------------------------
    //  Open-loop primitives
    // ------------------------------------------------------------------

    /** @brief Direct tank command. Bypasses all PID. Use for line following. */
    void tank(int16_t leftPwm, int16_t rightPwm);

    /** @brief Equal-and-opposite sides — rotates about the chassis centre. */
    void pivot(int16_t pwm, TurnDirection dir);

    void brake();
    void coast();

    /** Drops STBY low. Mandated by the rulebook's emergency-stop requirement. */
    void emergencyStop();

    // ------------------------------------------------------------------
    //  [A-C1] Closed-loop primitives REMOVED
    // ------------------------------------------------------------------
    //  requestRotation()/requestDistance()/isMotionComplete()/abortMotion()
    //  had no callers and three latent defects: serviceDistance() steered
    //  with an INVERTED sign (positive feedback), it held absolute heading 0
    //  instead of the heading at request time, and requestRotation() wrote
    //  _motionType before its target from another core, which could strand
    //  _motionComplete == false forever. Rotations are now closed on the IMU
    //  in TaskHandler, and every motor write happens on the control task via
    //  the drive mailbox (TaskHandler.cpp).

    // ------------------------------------------------------------------
    //  Line following
    // ------------------------------------------------------------------

    /**
     * @brief Feeds the normalised lateral error into the line PID and drives.
     * @param normalisedError -1.0 (line hard left) .. +1.0 (line hard right)
     * @param baseSpeed       Forward PWM before differential correction.
     * @note  Called at Rtos::PERIOD_CONTROL_MS from the control task.
     */
    void followLine(float normalisedError, int16_t baseSpeed);

    /**
     * @brief [A-C2] Clears the line PID and its dt clock. The control task
     *        calls this on the rising edge of line following and whenever the
     *        line is re-acquired, so a stale _prevError can never produce a
     *        derivative kick. Control task only.
     */
    void resetLinePid();

    // ------------------------------------------------------------------
    //  Odometry
    // ------------------------------------------------------------------

    float distanceTravelledMm() const;  ///< Mean of both wheels since reset
    float headingDeg()          const;  ///< Integrated from wheel differential
    float leftVelocityMmS()     const;
    float rightVelocityMmS()    const;
    void  resetOdometry();

    // ------------------------------------------------------------------
    //  Service
    // ------------------------------------------------------------------

    /**
     * @brief Control-task tick. Samples encoders and integrates odometry.
     *        Writes no PWM (followLine()/tank()/brake() do that).
     * @param dtSec Measured loop period, not the nominal one.
     */
    void update(float dtSec);

    /** Live PID retune hook for the telemetry task. */
    void setLineGains(float kp, float ki, float kd);

private:
    MotorSide         _left;
    MotorSide         _right;
    QuadratureEncoder _encLeft;
    QuadratureEncoder _encRight;

    PidController     _linePid;
    uint32_t          _lineLastMs;   ///< [A-C2] was a function-local static

    // --- Odometry state ----------------------------------------------------
    float _headingDeg;
    float _leftVelMmS;
    float _rightVelMmS;

    // --- Internal helpers --------------------------------------------------
    void  integrateOdometry(float dtSec);
    static int16_t clampPwm(float value);
};
