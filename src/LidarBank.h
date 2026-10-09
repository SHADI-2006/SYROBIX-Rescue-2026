/**
 * ============================================================================
 *  @file       LidarBank.h
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Three VL53L1X time-of-flight rangers sharing one I2C bus.
 *
 *  @dependency pololu/VL53L1X @ ^1.3.1   (lib_deps in platformio.ini)
 *              REMOVE pololu/VL53L0X from lib_deps — both libraries define
 *              overlapping register enums and the old one is no longer used.
 *
 *  ---------------------------------------------------------------------------
 *  MIGRATION NOTES — VL53L0X -> VL53L1X
 *  ---------------------------------------------------------------------------
 *    - setVcselPulsePeriod() / setSignalRateLimit() do not exist on VL53L1X.
 *      Their job is now done by the DISTANCE MODE (SHORT/MEDIUM/LONG), which
 *      internally selects the VCSEL periods and phase windows.
 *    - The public timing API is in MILLISECONDS (setTimingBudgetMs()).
 *      Pololu's setMeasurementTimingBudget() takes MICROSECONDS; the
 *      conversion happens in exactly one place, LidarBank.cpp.
 *    - Continuous mode is TIMED (startContinuous(period_ms)), and reads are
 *      now strictly NON-BLOCKING: dataReady() is polled, and read(false) is
 *      only called when a fresh sample exists. The old VL53L0X path called
 *      readRangeContinuousMillimeters(), which BLOCKS until a conversion
 *      completes — with the 50 ms front budget focused at a 30 ms sensor tick
 *      it stalled the whole Core-0 sensor task (IMU + colour) by ~20 ms every
 *      tick, and a dead ranger stalled it for the full 200 ms I/O timeout.
 *    - Every sample carries a range_status. Only RangeValid and
 *      RangeValidMinRangeClipped are accepted; everything else (sigma/signal
 *      fail, wrap, out-of-bounds) publishes Tune::TOF_INVALID with a FRESH
 *      timestamp, i.e. "looked, nothing trustworthy there".
 *    - Removed dead API: readBlocking() and setTimingBudgetUs() had no callers.
 *
 *  ---------------------------------------------------------------------------
 *  THREADING CONTRACT
 *  ---------------------------------------------------------------------------
 *    begin()                         setup() only, before tasks start
 *    updateNext()                    Core-0 sensor task ONLY (owns the bus
 *                                    traffic and the sample arrays)
 *    rangeMm/isValid/obstacleAhead/  sensor task (via SensorsSystem::publish)
 *    isUp/upMask/errorCount/...      or setup(); plain reads
 *    setFocus/clearFocus             ANY task (single volatile int8 store)
 *    setDistanceMode/setTimingBudgetMs
 *                                    ANY task: they only POST a request; the
 *                                    sensor task applies it on its next tick,
 *                                    so no two tasks ever talk to a ranger.
 * ============================================================================
 */

#pragma once

#include <stdint.h>
#include "Config.h"

class LidarBank {
public:
    LidarBank();

    /**
     * @brief XSHUT re-addressing sequence, then per-unit configuration.
     *
     *   1. Assert every XSHUT (LOW)       -> all three held in hardware standby
     *   2. Probe 0x29 and the targets     -> anything answering now is a wiring
     *                                        fault (XSHUT open / stuck), abort
     *   3. Release XSHUT[i], wait boot    -> exactly one unit on 0x29
     *   4. init(), move it to its address -> it vacates 0x29
     *   5. Verify it answers the new one  -> then configure + start ranging
     *   6. Next unit
     *
     * A unit that fails is parked back in reset so it cannot squat on 0x29.
     *
     * @return true when the FRONT ranger is up. FRONT is mandatory (obstacle,
     *         victim scan, approach, grasp check); LEFT/RIGHT are advisory and
     *         currently consumed by no mission logic, so a dead side unit must
     *         not brick the run. Query upMask() for the full picture.
     * @note   Re-entrant: resets the driver objects, so a second call after a
     *         warm reset re-addresses cleanly.
     */
    bool begin();

    /**
     * @brief Non-blocking service tick. Collects every ranger that has a
     *        fresh sample (or only the focused one), and applies any pending
     *        reconfiguration request. Never waits on a conversion.
     *
     * @note  Name kept for source compatibility with SensorsSystem. Unlike
     *        the old round-robin it services ALL units each tick: a
     *        dataReady() poll is one ~0.1 ms register read, and a fresh
     *        sample only exists every Tof::*_PERIOD_MS anyway, so polling all
     *        three costs less than one old blocking read and removes the
     *        90 ms round-robin staleness entirely.
     */
    void updateNext();

    /**
     * @brief Restricts servicing to one ranger (the others are left ranging
     *        but not read, and correctly go stale -> isValid() == false).
     *        Used by the victim states so FRONT gets every bus slot.
     */
    void setFocus(LidarId id);
    void clearFocus();

    /** @return Last accepted range (mm, offset-corrected), or
     *          Tune::TOF_INVALID if stale, rejected, or the unit is down. */
    uint16_t rangeMm(LidarId id) const;

    /** @return millis() when the last sample was collected (0 = never).
     *          Stop-and-go callers use it to demand a post-settle sample. */
    uint32_t stampMs(LidarId id) const;

    /** @return true when the cached sample is fresh AND was accepted. */
    bool isValid(LidarId id) const;

    /** @brief Convenience predicate for the navigation layer. */
    bool obstacleAhead(uint16_t thresholdMm = Tune::TOF_OBSTACLE_MM) const;

    // --- Runtime reconfiguration (posted, applied by the sensor task) -------

    /**
     * @brief Requests a new ranging mode. If the current budget is below the
     *        new mode's legal floor (33 ms for MEDIUM/LONG), the budget is
     *        raised to that floor in the same request.
     * @return false if @p id is out of range or the unit is down.
     */
    bool setDistanceMode(LidarId id, TofDistanceMode mode);

    /**
     * @brief Requests a new timing budget in MILLISECONDS. The
     *        inter-measurement period is raised to budget + slack if needed.
     * @return false if the budget is illegal for the unit's (pending) mode,
     *         above Tof::MAX_BUDGET_MS, or the unit is down.
     */
    bool setTimingBudgetMs(LidarId id, uint16_t budgetMs);

    // --- Diagnostics -------------------------------------------------------

    bool     isUp(LidarId id) const;
    /** bit0 = FRONT, bit1 = LEFT, bit2 = RIGHT. */
    uint8_t  upMask() const;
    /** True if begin() found a device on 0x29 or on a target address while
     *  every XSHUT was asserted — i.e. an XSHUT line is not reaching a unit. */
    bool     busFault() const;
    /** Cumulative I2C errors seen while servicing this unit. */
    uint16_t errorCount(LidarId id) const;
    /** Raw VL53L1X RangeStatus of the last sample (255 = none yet). */
    uint8_t  lastRangeStatus(LidarId id) const;

private:
    static constexpr uint8_t N = static_cast<uint8_t>(LidarId::COUNT);

    // --- Sample state: written ONLY by the sensor task ---------------------
    uint16_t _range[N];
    uint32_t _stamp[N];
    uint8_t  _lastStatus[N];
    uint16_t _errors[N];
    uint32_t _lastPollMs[N];   ///< last serviceSensor() visit, for resume-discard
    bool     _up[N];
    bool     _busFault;

    // --- Active configuration: written ONLY by begin()/sensor task ---------
    TofDistanceMode _mode[N];
    uint16_t        _budgetMs[N];
    uint16_t        _periodMs[N];

    // --- Posted reconfiguration requests (any task -> sensor task) ---------
    //  Writer fills mode/budget/period, THEN sets the flag; the sensor task
    //  reads the flag, then the values, then clears it. All single-word
    //  volatile stores, so no lock and no torn values.
    volatile uint8_t  _reqMode[N];
    volatile uint16_t _reqBudgetMs[N];
    volatile uint16_t _reqPeriodMs[N];
    volatile bool     _reqPending[N];

    volatile int8_t   _focus;   ///< -1 = service all, else the pinned LidarId

    bool bringUpSensor(uint8_t idx);
    bool applyConfig(uint8_t idx, TofDistanceMode mode,
                     uint16_t budgetMs, uint16_t periodMs);
    void applyPendingConfig();
    void serviceSensor(uint8_t idx);
    uint32_t staleAfterMs(uint8_t idx) const;

    static bool indexOk(LidarId id);
};
