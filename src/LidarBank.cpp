/**
 * ============================================================================
 *  @file       LidarBank.cpp
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      VL53L1X bank: XSHUT re-addressing, distance-mode / timing-
 *              budget configuration, and non-blocking continuous acquisition.
 *
 *  @dependency pololu/VL53L1X @ ^1.3.1
 *
 *  ---------------------------------------------------------------------------
 *  BUS BUDGET PER CORE-0 SENSOR TICK (30 ms, 400 kHz I2C)
 *  ---------------------------------------------------------------------------
 *    dataReady() poll, per unit ...............................  ~0.10 ms
 *    read(false) when ready: 17-byte result burst + DSS update
 *      + interrupt clear ......................................  ~0.70 ms
 *    Worst case, all three ready on the same tick .............  ~2.4 ms
 *    Typical (focused FRONT, 38 ms period) ....................  ~0.1-0.8 ms
 *  Nothing here ever waits for a conversion. The old VL53L0X path did.
 * ============================================================================
 */

#include "LidarBank.h"

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

namespace {

constexpr uint8_t kCount = static_cast<uint8_t>(LidarId::COUNT);

/** Static wiring + profile per unit, indexed by LidarId. */
struct UnitCfg {
    uint8_t         xshutPin;
    uint8_t         address;
    TofDistanceMode mode;
    uint16_t        budgetMs;
    uint16_t        periodMs;
};

const UnitCfg kUnit[kCount] = {
    { Pins::TOF_XSHUT_FRONT, Hw::TOF_ADDR_FRONT,
      Tof::FRONT_MODE, Tof::FRONT_BUDGET_MS, Tof::FRONT_PERIOD_MS },
    { Pins::TOF_XSHUT_LEFT,  Hw::TOF_ADDR_LEFT,
      Tof::SIDE_MODE,  Tof::SIDE_BUDGET_MS,  Tof::SIDE_PERIOD_MS  },
    { Pins::TOF_XSHUT_RIGHT, Hw::TOF_ADDR_RIGHT,
      Tof::SIDE_MODE,  Tof::SIDE_BUDGET_MS,  Tof::SIDE_PERIOD_MS  },
};

/** Driver instances, file-static so LidarBank.h stays library-free. */
VL53L1X g_tof[kCount];

constexpr uint8_t STATUS_NONE = 255;

VL53L1X::DistanceMode toDriverMode(TofDistanceMode m) {
    switch (m) {
        case TofDistanceMode::SHORT:  return VL53L1X::Short;
        case TofDistanceMode::MEDIUM: return VL53L1X::Medium;
        case TofDistanceMode::LONG:
        default:                      return VL53L1X::Long;
    }
}

uint16_t minBudgetMs(TofDistanceMode m) {
    return (m == TofDistanceMode::SHORT) ? Tof::MIN_BUDGET_SHORT_MS
                                         : Tof::MIN_BUDGET_LONG_MS;
}

/** Hold a unit in hardware standby. Resets its I2C address to 0x29. */
void xshutAssert(uint8_t pin) {
    digitalWrite(pin, LOW);      // latch LOW before enabling the driver, so
    pinMode(pin, OUTPUT);        // the pin never glitches HIGH on the way
    digitalWrite(pin, LOW);
}

/** Let a unit boot. See SYROBIX_TOF_XSHUT_OPEN_DRAIN in Config.h. */
void xshutRelease(uint8_t pin) {
#if SYROBIX_TOF_XSHUT_OPEN_DRAIN
    pinMode(pin, INPUT);         // breakout pull-up raises XSHUT to 2.8 V
#else
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
#endif
}

/** @return true if any device ACKs @p addr. */
bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

/** RangeValid, or clipped-at-minimum (target right at the lens). */
bool statusAccepted(uint8_t st) {
    return st == VL53L1X::RangeValid ||
           st == VL53L1X::RangeValidMinRangeClipped;
}

} // anonymous namespace


// ============================================================================
//  Construction
// ============================================================================

LidarBank::LidarBank() : _busFault(false), _focus(-1) {
    for (uint8_t i = 0; i < N; ++i) {
        _range[i]       = Tune::TOF_INVALID;
        _stamp[i]       = 0;
        _lastStatus[i]  = STATUS_NONE;
        _errors[i]      = 0;
        _lastPollMs[i]  = 0;
        _up[i]          = false;
        _mode[i]        = kUnit[i].mode;
        _budgetMs[i]    = kUnit[i].budgetMs;
        _periodMs[i]    = kUnit[i].periodMs;
        _reqMode[i]     = static_cast<uint8_t>(kUnit[i].mode);
        _reqBudgetMs[i] = kUnit[i].budgetMs;
        _reqPeriodMs[i] = kUnit[i].periodMs;
        _reqPending[i]  = false;
    }
}

bool LidarBank::indexOk(LidarId id) {
    return static_cast<uint8_t>(id) < static_cast<uint8_t>(LidarId::COUNT);
}


// ============================================================================
//  Bring-up
// ============================================================================

bool LidarBank::begin() {
    _busFault = false;

    // Step 1: every unit into hardware standby. Until this is done all three
    // may be answering 0x29 at once (cold boot), or sitting on their old
    // addresses (warm reset) — XSHUT LOW puts every one of them back to a
    // known state: silent, and at 0x29 once released.
    for (uint8_t i = 0; i < N; ++i) {
        _up[i]    = false;
        _range[i] = Tune::TOF_INVALID;
        _stamp[i] = 0;
        xshutAssert(kUnit[i].xshutPin);
    }
    delay(Tof::RESET_HOLD_MS);

    // Step 2: with every XSHUT asserted, NOTHING may answer 0x29 or any of
    // our target addresses. If something does, an XSHUT line is not reaching
    // its unit (open wire, wrong pin, module without XSHUT broken out). Going
    // ahead would put two rangers on one address, which does not fail — it
    // returns plausible, wrong, interleaved ranges. Refuse instead.
    if (i2cProbe(Tof::DEFAULT_ADDR)) {
        _busFault = true;
    }
    for (uint8_t i = 0; i < N; ++i) {
        if (i2cProbe(kUnit[i].address)) {
            _busFault = true;
        }
    }
    if (_busFault) {
        return false;
    }

    // Steps 3-6: strictly one unit at a time.
    for (uint8_t i = 0; i < N; ++i) {
        bringUpSensor(i);   // failures are recorded in _up[] and parked
    }

    return _up[static_cast<uint8_t>(LidarId::FRONT)];
}

bool LidarBank::bringUpSensor(uint8_t idx) {
    const UnitCfg& u = kUnit[idx];
    VL53L1X& dev = g_tof[idx];

    // Fresh driver object: resets its cached address to 0x29 and its
    // calibration flags, so begin() is safe to call again after a warm reset.
    dev = VL53L1X();
    dev.setBus(&Wire);
    dev.setTimeout(Tof::IO_TIMEOUT_MS);   // non-zero, or init() can hang

    xshutRelease(u.xshutPin);
    delay(Tof::BOOT_WAIT_MS);

    bool ok = false;
    for (uint8_t attempt = 0; attempt < Tof::INIT_RETRIES && !ok; ++attempt) {
        ok = dev.init();                  // model-ID check + soft reset + boot
        if (!ok) delay(Tof::BOOT_WAIT_MS);
    }
    if (!ok) {
        xshutAssert(u.xshutPin);          // park it off the bus
        return false;
    }

    dev.setAddress(u.address);

    // Verify the move actually happened before releasing the next unit onto
    // 0x29. A silent setAddress() failure here is exactly the "two units, one
    // address" failure mode, one step later.
    if (!i2cProbe(u.address) || i2cProbe(Tof::DEFAULT_ADDR)) {
        xshutAssert(u.xshutPin);
        return false;
    }

    if (!applyConfig(idx, u.mode, u.budgetMs, u.periodMs)) {
        xshutAssert(u.xshutPin);
        return false;
    }

#if SYROBIX_TOF_USE_ROI
    if (idx == static_cast<uint8_t>(LidarId::FRONT)) {
        // ROI can only be changed while stopped.
        dev.stopContinuous();
        dev.setROISize(Tof::FRONT_ROI_W, Tof::FRONT_ROI_H);
        dev.startContinuous(u.periodMs);
    }
#endif

    _up[idx] = true;
    return true;
}

bool LidarBank::applyConfig(uint8_t idx, TofDistanceMode mode,
                            uint16_t budgetMs, uint16_t periodMs) {
    VL53L1X& dev = g_tof[idx];

    if (budgetMs < minBudgetMs(mode) || budgetMs > Tof::MAX_BUDGET_MS) {
        return false;
    }
    if (periodMs < budgetMs) {
        periodMs = static_cast<uint16_t>(budgetMs + Tof::PERIOD_SLACK_MS);
    }

    // Mode and budget are only writable while not ranging. stopContinuous()
    // on an idle unit is harmless (an abort of nothing).
    dev.stopContinuous();

    // ORDER MATTERS: the Pololu setDistanceMode() re-applies whatever budget
    // is currently programmed, so set the mode first, then the budget.
    if (!dev.setDistanceMode(toDriverMode(mode))) {
        return false;
    }
    // Pololu API is MICROSECONDS. This is the only ms->us conversion site.
    if (!dev.setMeasurementTimingBudget(static_cast<uint32_t>(budgetMs) * 1000UL)) {
        return false;
    }
    if (dev.last_status != 0) {
        return false;   // an I2C write inside the sequence NACKed
    }

    dev.startContinuous(periodMs);   // timed mode; clears any stale interrupt

    _mode[idx]     = mode;
    _budgetMs[idx] = budgetMs;
    _periodMs[idx] = periodMs;

    // A reconfigured unit's previous sample describes the old profile.
    _range[idx]      = Tune::TOF_INVALID;
    _stamp[idx]      = 0;
    _lastPollMs[idx] = 0;   // first sample after (re)start is discarded
    return true;
}


// ============================================================================
//  Runtime service — Core-0 sensor task only
// ============================================================================

void LidarBank::updateNext() {
    applyPendingConfig();

    const int8_t focus = _focus;   // single read: cannot change mid-decision
    if (focus >= 0 && focus < static_cast<int8_t>(N)) {
        serviceSensor(static_cast<uint8_t>(focus));
        return;
    }
    for (uint8_t i = 0; i < N; ++i) {
        serviceSensor(i);
    }
}

void LidarBank::serviceSensor(uint8_t idx) {
    if (!_up[idx]) {
        return;
    }
    VL53L1X& dev = g_tof[idx];

    // Resume guard. While another unit is focused, this one keeps ranging but
    // nobody clears its interrupt, so the result register can hold a sample
    // that is SECONDS old. Stamping that "now" would publish stale geometry as
    // fresh. If we have not visited this unit for longer than two periods,
    // the first ready sample is read (which clears the interrupt and re-arms
    // ranging) and thrown away.
    const uint32_t nowPoll = millis();
    const bool resumed = (_lastPollMs[idx] == 0) ||
                         ((nowPoll - _lastPollMs[idx]) >
                          2UL * static_cast<uint32_t>(_periodMs[idx]));
    _lastPollMs[idx] = nowPoll;

    // Non-blocking readiness check. dataReady() is one register read.
    const bool ready = dev.dataReady();
    if (dev.last_status != 0) {
        // NACK / bus error. Not fatal: the freshness window turns a unit that
        // keeps failing into isValid() == false on its own.
        if (_errors[idx] < 0xFFFF) ++_errors[idx];
        return;
    }
    if (!ready) {
        return;   // conversion still in flight; collect it next tick
    }

    // Data is ready, so read(false) never waits. It also runs the driver's
    // DSS update and clears the interrupt, which re-arms timed ranging.
    const uint16_t raw = dev.read(false);
    if (dev.last_status != 0) {
        if (_errors[idx] < 0xFFFF) ++_errors[idx];
        return;
    }

    if (resumed) {
        return;   // possibly stale; interrupt now cleared, next one is fresh
    }

    const uint8_t status = static_cast<uint8_t>(dev.ranging_data.range_status);
    _lastStatus[idx] = status;

    uint32_t now = millis();
    if (now == 0) now = 1;   // 0 is the "never sampled" sentinel

    if (statusAccepted(status)) {
        int32_t mm = static_cast<int32_t>(raw) + Tof::OFFSET_MM[idx];
        if (mm < 0) mm = 0;
        if (mm >= static_cast<int32_t>(Tune::TOF_INVALID)) {
            mm = Tune::TOF_INVALID - 1;
        }
        // Single writer (this task), and the only reader of these two arrays
        // is publish() on this same task — so no reader can land between the
        // stores. Range first, stamp second, regardless.
        _range[idx] = static_cast<uint16_t>(mm);
    } else {
        // Sigma/signal fail, wrap, out-of-bounds: "looked, nothing
        // trustworthy". Fresh stamp, invalid range.
        _range[idx] = Tune::TOF_INVALID;
    }
    _stamp[idx] = now;
}

void LidarBank::applyPendingConfig() {
    for (uint8_t i = 0; i < N; ++i) {
        if (!_reqPending[i]) continue;

        // Flag first, then values (the poster wrote values first, then flag).
        const TofDistanceMode mode = static_cast<TofDistanceMode>(_reqMode[i]);
        const uint16_t budget = _reqBudgetMs[i];
        const uint16_t period = _reqPeriodMs[i];
        _reqPending[i] = false;

        if (!_up[i]) continue;
        if (!applyConfig(i, mode, budget, period)) {
            // Rejected on the device. Restore the last known-good profile
            // rather than leave the unit stopped.
            applyConfig(i, _mode[i], _budgetMs[i], _periodMs[i]);
            if (_errors[i] < 0xFFFF) ++_errors[i];
        }
    }
}

uint32_t LidarBank::staleAfterMs(uint8_t idx) const {
    return static_cast<uint32_t>(Tof::STALE_PERIODS) * _periodMs[idx] +
           Tof::STALE_MARGIN_MS;
}


// ============================================================================
//  Focus
// ============================================================================

void LidarBank::setFocus(LidarId id) {
    if (!indexOk(id)) return;
    _focus = static_cast<int8_t>(static_cast<uint8_t>(id));
}

void LidarBank::clearFocus() {
    _focus = -1;
}


// ============================================================================
//  Reconfiguration requests — callable from any task
// ============================================================================

bool LidarBank::setDistanceMode(LidarId id, TofDistanceMode mode) {
    if (!indexOk(id)) return false;
    const uint8_t idx = static_cast<uint8_t>(id);
    if (!_up[idx]) return false;

    uint16_t budget = _reqPending[idx] ? _reqBudgetMs[idx] : _budgetMs[idx];
    if (budget < minBudgetMs(mode)) budget = minBudgetMs(mode);
    uint16_t period = _reqPending[idx] ? _reqPeriodMs[idx] : _periodMs[idx];
    if (period < budget + Tof::PERIOD_SLACK_MS) {
        period = static_cast<uint16_t>(budget + Tof::PERIOD_SLACK_MS);
    }

    _reqMode[idx]     = static_cast<uint8_t>(mode);
    _reqBudgetMs[idx] = budget;
    _reqPeriodMs[idx] = period;
    _reqPending[idx]  = true;    // publish LAST
    return true;
}

bool LidarBank::setTimingBudgetMs(LidarId id, uint16_t budgetMs) {
    if (!indexOk(id)) return false;
    const uint8_t idx = static_cast<uint8_t>(id);
    if (!_up[idx]) return false;

    const TofDistanceMode mode = _reqPending[idx]
        ? static_cast<TofDistanceMode>(_reqMode[idx]) : _mode[idx];
    if (budgetMs < minBudgetMs(mode) || budgetMs > Tof::MAX_BUDGET_MS) {
        return false;
    }

    uint16_t period = _reqPending[idx] ? _reqPeriodMs[idx] : _periodMs[idx];
    if (period < budgetMs + Tof::PERIOD_SLACK_MS) {
        period = static_cast<uint16_t>(budgetMs + Tof::PERIOD_SLACK_MS);
    }

    _reqMode[idx]     = static_cast<uint8_t>(mode);
    _reqBudgetMs[idx] = budgetMs;
    _reqPeriodMs[idx] = period;
    _reqPending[idx]  = true;    // publish LAST
    return true;
}


// ============================================================================
//  Accessors
// ============================================================================

uint16_t LidarBank::rangeMm(LidarId id) const {
    return isValid(id) ? _range[static_cast<uint8_t>(id)] : Tune::TOF_INVALID;
}

uint32_t LidarBank::stampMs(LidarId id) const {
    return indexOk(id) ? _stamp[static_cast<uint8_t>(id)] : 0;
}

bool LidarBank::isValid(LidarId id) const {
    if (!indexOk(id)) return false;
    const uint8_t idx = static_cast<uint8_t>(id);
    if (!_up[idx])                                   return false;
    if (_stamp[idx] == 0)                            return false;
    if (_range[idx] >= Tune::TOF_INVALID)            return false;
    if ((millis() - _stamp[idx]) > staleAfterMs(idx)) return false;
    return true;
}

bool LidarBank::obstacleAhead(uint16_t thresholdMm) const {
    return isValid(LidarId::FRONT) &&
           (rangeMm(LidarId::FRONT) < thresholdMm);
}

bool LidarBank::isUp(LidarId id) const {
    return indexOk(id) && _up[static_cast<uint8_t>(id)];
}

uint8_t LidarBank::upMask() const {
    uint8_t m = 0;
    for (uint8_t i = 0; i < N; ++i) {
        if (_up[i]) m |= static_cast<uint8_t>(1u << i);
    }
    return m;
}

bool LidarBank::busFault() const { return _busFault; }

uint16_t LidarBank::errorCount(LidarId id) const {
    return indexOk(id) ? _errors[static_cast<uint8_t>(id)] : 0;
}

uint8_t LidarBank::lastRangeStatus(LidarId id) const {
    return indexOk(id) ? _lastStatus[static_cast<uint8_t>(id)] : STATUS_NONE;
}
