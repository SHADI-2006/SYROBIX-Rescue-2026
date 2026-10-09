/**
 * ============================================================================
 *  @file       LineArray.cpp
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Implementation of LineArray: 8x TCR5000 on ADC1, split into a
 *              6-sensor FRONT row (weighted-centroid line PID) and a 2-sensor
 *              REAR pair (auxiliary — no centroid role), per-channel
 *              calibration, and the position estimate that feeds the line PID.
 *
 *  @note       Declared in LineArray.h (split out of SensorsSystem.h in
 *              Rev 3). Its own translation unit because it is the only sensor sampled on Core 1, and
 *              keeping it separate makes that ownership obvious at a glance.
 *
 *  ---------------------------------------------------------------------------
 *  POST-REVIEW REVISION — what changed in this file
 *  ---------------------------------------------------------------------------
 *  The centroid math was reviewed line by line and is CORRECT — see the
 *  derivation below, which now carries the worked numbers. Nothing in the
 *  weighted sum changed. Two things did:
 *
 *    [FIX 7] calibrateFinish() / _calComplete. read() runs at 200 Hz from the
 *            line task and recomputes _calibrated every sweep. During the
 *            calibration window that meant _calibrated could flip true from a
 *            mid-sweep transient — the operator's hand giving every front
 *            channel a >300 count spread half a second into a four-second
 *            sweep — and stepCalibrating() could then succeed on a
 *            half-calibrated array. _calibrated now also requires the sweep to
 *            have been explicitly finished.
 *
 *    [FIX 7] Reinforced the single-owner rule in the class doc. The other half
 *            of that fix lives in TaskHandler::lineLoop(), which no longer
 *            samples while the mission task owns the array.
 *
 *  ---------------------------------------------------------------------------
 *  6 FRONT + 2 REAR — WHY THE CENTROID ONLY SEES INDICES 0-5
 *  ---------------------------------------------------------------------------
 *  The hardware carries 8 TCR5000s on one ADC1 bus, same as before, but only
 *  the first 6 (indices 0-5) sit in the front row that steers the chassis.
 *  Indices 6-7 are a second, physically separate pair mounted at the rear.
 *  They are still sampled, calibrated and threshold-masked every tick like
 *  any other channel — nothing about the ADC sweep changes — but they never
 *  enter the weighted sum, and a rear channel failing calibration does not
 *  block a run the way a front-channel failure does (see read()).
 *
 *  Exposed instead through isRearOnLine() / rawRearLeft() / rawRearRight() /
 *  normalisedRearLeft() / normalisedRearRight(), for whatever the mission
 *  layer wants a "does the tail still see the line" signal for (e.g.
 *  confirming the chassis has fully cleared an intersection bar before
 *  committing to a turn). Wiring that decision up is a TaskHandler concern,
 *  not this file's — LineArray only reports what it sees.
 *
 *  ---------------------------------------------------------------------------
 *  THE +/-4000 SCALE, RE-DERIVED FOR 6 SENSORS  (verified, unchanged)
 *  ---------------------------------------------------------------------------
 *  The classic Pololu-style centroid for N sensors produces 0..(N-1)*1000,
 *  centred at (N-1)*1000/2. For the original 8-sensor row that was 0..7000
 *  centred at 3500. For this 6-sensor front row it is 0..5000 centred at
 *  2500 — CENTROID_CENTRE below is updated accordingly.
 *
 *  Worked end to end, so the next reader does not have to redo it:
 *
 *      weights           w_i = i * 1000, i in 0..5   ->  0,1000,...,5000
 *      centroid          c   = SUM(w_i * n_i) / SUM(n_i)      in [0, 5000]
 *      centred           c - 2500                             in [-2500, +2500]
 *      rescaled          (c - 2500) * (4000 / 2500)           in [-4000, +4000]
 *      published         that / 4000  ==  (c - 2500) / 2500   in [-1.0, +1.0]
 *
 *  So the +/-4000 intermediate cancels exactly and the public range is the
 *  same -1.0..+1.0 readPosition() has always promised. That is why
 *  Tune::LINE_KP / LINE_KD did NOT need retuning for the narrower array —
 *  only CENTROID_CENTRE moved. Overflow: max weightedSum is
 *  1000 * 5000 * 6 = 3.0e7, comfortably inside uint32_t.
 *
 *  ---------------------------------------------------------------------------
 *  SENSOR POLARITY — verify this before the first run
 *  ---------------------------------------------------------------------------
 *  This file assumes the TCR5000 phototransistors are wired so that a DARK
 *  surface produces a HIGHER ADC count. That is the common pull-down wiring.
 *  If your boards are wired the other way, the array will drive the robot off
 *  the line at full confidence rather than fail visibly. Flip INVERT_POLARITY
 *  below; do not try to compensate in the PID.
 *
 *  ---------------------------------------------------------------------------
 *  THREADING
 *  ---------------------------------------------------------------------------
 *  Exactly one task may touch an instance at a time. There is no internal
 *  locking and there deliberately is none: this runs inside the 200 Hz budget.
 *      during a run          -> Core 1 line task calls read()
 *      during CALIBRATING    -> Core 0 mission task calls calibrateSample() /
 *                               calibrateFinish() / read(), and the line task
 *                               is gated off (TaskHandler::lineLoop())
 *  Running both at once races on _calMin/_calMax and on the ADC itself.
 * ============================================================================
 */

#include "LineArray.h"

#include <Arduino.h>

namespace {

/** Set true if a dark surface reads LOWER than a bright one on your wiring. */
constexpr bool     INVERT_POLARITY = false;

/** ESP32-S3 ADC is 12-bit. */
constexpr uint16_t ADC_MAX         = 4095;

/** Normalised output range per channel: 0 = fully bright, 1000 = fully dark. */
constexpr uint16_t NORM_MAX        = 1000;

/** Number of channels in the FRONT row that feeds the weighted centroid. */
constexpr uint8_t  FRONT_COUNT     = 6;

/** Physical channel indices of the two REAR sensors. */
constexpr uint8_t  REAR_LEFT_INDEX  = 6;
constexpr uint8_t  REAR_RIGHT_INDEX = 7;

/** Half-width of the reported position, before normalisation to +/-1.0. */
constexpr float    POSITION_SPAN   = 4000.0f;

/** Centre of the raw 6-sensor FRONT centroid: (FRONT_COUNT - 1) * 1000 / 2. */
constexpr float    CENTROID_CENTRE = 2500.0f;

/**
 * A channel needs at least this much spread between its calibrated min and max
 * to be trusted. Less than this means the sensor never saw both a line and a
 * floor during calibration — usually a dead emitter or a bad ride height.
 */
constexpr uint16_t MIN_USABLE_SPREAD = 300;

// Compile-time restatement of the derivation in the file header. If anyone
// changes FRONT_COUNT without moving CENTROID_CENTRE, the build stops here
// instead of the robot quietly steering to one side of the line.
static_assert(CENTROID_CENTRE ==
                  static_cast<float>((FRONT_COUNT - 1) * 1000) / 2.0f,
              "CENTROID_CENTRE must be (FRONT_COUNT-1)*1000/2 or the centred "
              "error is biased and the robot tracks off-centre.");

} // anonymous namespace


// ============================================================================
//  Construction and bring-up
// ============================================================================

LineArray::LineArray()
    : _mask(0),
      _lastPosition(0.0f),
      _lineLost(false),
      _calibrated(false),
      _calComplete(false) {
    for (uint8_t i = 0; i < 8; ++i) {
        _raw[i]        = 0;
        _normalised[i] = 0;
        // Extrema start inverted so the first calibration sample overwrites
        // both ends rather than being averaged against a fake baseline.
        _calMin[i]     = ADC_MAX;
        _calMax[i]     = 0;
    }
}

void LineArray::begin() {
    analogReadResolution(12);

    for (uint8_t i = 0; i < 8; ++i) {
        pinMode(Pins::LINE_SENSOR[i], INPUT);
        // 11 dB attenuation gives the full ~0-3.3 V span. Without it the ADC
        // saturates around 1.1 V and every bright reading looks identical.
        analogSetPinAttenuation(Pins::LINE_SENSOR[i], ADC_11db);
    }
}


// ============================================================================
//  Calibration
// ============================================================================

void LineArray::calibrateReset() {
    for (uint8_t i = 0; i < 8; ++i) {
        _calMin[i] = ADC_MAX;
        _calMax[i] = 0;
    }
    _calibrated  = false;
    _calComplete = false;   // [FIX 7] a new sweep invalidates the old verdict
}

void LineArray::calibrateSample() {
    for (uint8_t i = 0; i < 8; ++i) {
        const uint16_t sample = static_cast<uint16_t>(analogRead(Pins::LINE_SENSOR[i]));
        if (sample < _calMin[i]) _calMin[i] = sample;
        if (sample > _calMax[i]) _calMax[i] = sample;
    }
}

void LineArray::calibrateFinish() {
    // [FIX 7] The sweep is over. From here read() is allowed to publish a
    // verdict on channel health; before this point it may not, no matter how
    // good the spreads look mid-sweep.
    _calComplete = true;
}

bool LineArray::isCalibrated() const {
    return _calibrated;
}


// ============================================================================
//  Acquisition
// ============================================================================

void LineArray::read() {
    // Re-evaluate calibration validity here rather than in endLineCalibration()
    // so that a sensor which dies mid-match is caught on the next sweep.
    // Scoped to the FRONT row only: the front 6 are what the line PID and the
    // intersection hypothesis depend on, so they gate _calibrated. A dead
    // REAR channel degrades isRearOnLine() gracefully (it just never reports
    // that side as active) rather than faulting out a run over a sensor that
    // has no say in navigation.
    bool frontChannelsUsable = true;

    uint32_t weightedSum = 0;   // sum of (position_weight * normalised), FRONT only
    uint32_t totalSum    = 0;   // sum of normalised, FRONT only
    uint8_t  frontActive = 0;
    _mask = 0;

    for (uint8_t i = 0; i < 8; ++i) {
        _raw[i] = static_cast<uint16_t>(analogRead(Pins::LINE_SENSOR[i]));

        const uint16_t lo = _calMin[i];
        const uint16_t hi = _calMax[i];

        if (hi <= lo || (hi - lo) < MIN_USABLE_SPREAD) {
            // Uncalibrated or dead channel. Contribute nothing rather than
            // contribute noise — an untrustworthy channel that still votes is
            // worse than one that abstains.
            if (i < FRONT_COUNT) {
                frontChannelsUsable = false;
            }
            _normalised[i] = 0;
            continue;
        }

        // Scale into 0..NORM_MAX against this channel's own observed range.
        int32_t scaled = (static_cast<int32_t>(_raw[i]) - lo) * NORM_MAX
                         / static_cast<int32_t>(hi - lo);
        if (scaled < 0)                scaled = 0;
        if (scaled > NORM_MAX)         scaled = NORM_MAX;
        if (INVERT_POLARITY)           scaled = NORM_MAX - scaled;

        _normalised[i] = static_cast<uint16_t>(scaled);

        if (_normalised[i] >= Tune::LINE_ON_THRESHOLD) {
            // The mask covers all 8 physical channels (front AND rear) so
            // that digitalMask() stays a useful raw diagnostic, and so the
            // rear accessors below can read their bits straight out of it.
            _mask |= static_cast<uint8_t>(1u << i);
            if (i < FRONT_COUNT) {
                ++frontActive;
            }
        }

        if (i < FRONT_COUNT) {
            // Weight by sensor index scaled to 1000 per step: 0, 1000, .. 5000.
            // Indices 6-7 (rear) are sampled and masked above like any other
            // channel but deliberately excluded here — they are a separate
            // 2-sensor array bolted on the back, not an extension of the
            // front line-following row.
            weightedSum += static_cast<uint32_t>(_normalised[i]) * (i * 1000u);
            totalSum    += _normalised[i];
        }
    }

    // [FIX 7] Health is only a verdict once the sweep has been declared over.
    _calibrated = _calComplete && frontChannelsUsable;

    if (frontActive == 0 || totalSum == 0) {
        // Line lost. Hold the last known position and let the PID keep steering
        // toward the side the line was last seen on — that is what recovers a
        // sharp corner. The mission layer decides when "lost" becomes "lack of
        // progress" via Tune::LINE_LOST_MAX_MM / LINE_LOST_STALL_MS. Rear-sensor activity has no
        // bearing on this: the rear pair can see the line while the front row
        // has already run off it (or vice versa), and neither case should be
        // read as "the front is still tracking".
        _lineLost = true;
        return;
    }

    _lineLost = false;

    const float centroid = static_cast<float>(weightedSum) / static_cast<float>(totalSum);

    // Centre, rescale to the +/-4000 convention, then normalise for the PID.
    // Algebraically this is (centroid - 2500) / 2500; the two constants are
    // kept explicit because the +/-4000 intermediate is the convention the
    // Tune::LINE_* gains were written against. See the file header.
    const float error4000 = (centroid - CENTROID_CENTRE) * (POSITION_SPAN / CENTROID_CENTRE);
    _lastPosition = error4000 / POSITION_SPAN;

    if (_lastPosition >  1.0f) _lastPosition =  1.0f;
    if (_lastPosition < -1.0f) _lastPosition = -1.0f;
}


// ============================================================================
//  Accessors
// ============================================================================

float LineArray::readPosition() const {
    // Deliberately returns the retained value when the line is lost. See read().
    return _lastPosition;
}

uint8_t LineArray::digitalMask() const {
    return _mask;
}

uint8_t LineArray::activeCount() const {
    // FRONT row only (indices 0-5). This is what isIntersectionCandidate()
    // and the mission layer's "how many sensors currently see a line" checks
    // (e.g. TaskHandler::stepTurning's post-turn reacquisition test) actually
    // care about; the rear pair has its own accessors below.
    uint8_t count = 0;
    for (uint8_t bit = 0; bit < FRONT_COUNT; ++bit) {
        if (_mask & (1u << bit)) ++count;
    }
    return count;
}

bool LineArray::isLineLost() const {
    return _lineLost;
}

bool LineArray::isIntersectionCandidate() const {
    // A hypothesis only, and FRONT-only: a full-width dark reading across the
    // 6-sensor front row is equally consistent with an intersection bar and
    // with the black strip where a speed bump crosses the line. The mission
    // layer corroborates with the colour sensors before committing a turn.
    // The rear pair never votes here — see isRearOnLine() for its own signal.
    return activeCount() >= Tune::INTERSECTION_MIN_HITS;
}

bool LineArray::isRearLeftActive() const {
    return (_mask & (1u << REAR_LEFT_INDEX)) != 0;
}

bool LineArray::isRearRightActive() const {
    return (_mask & (1u << REAR_RIGHT_INDEX)) != 0;
}

bool LineArray::isRearOnLine() const {
    // OR, not AND: the two rear sensors span a much narrower footprint than
    // the 6-sensor front row, so requiring both to be dark would miss the
    // line whenever the chassis tail is even slightly skewed off it. Either
    // one seeing it is enough to say "the tail is still over the line".
    return isRearLeftActive() || isRearRightActive();
}

uint16_t LineArray::rawRearLeft() const {
    return _raw[REAR_LEFT_INDEX];
}

uint16_t LineArray::rawRearRight() const {
    return _raw[REAR_RIGHT_INDEX];
}

uint16_t LineArray::normalisedRearLeft() const {
    return _normalised[REAR_LEFT_INDEX];
}

uint16_t LineArray::normalisedRearRight() const {
    return _normalised[REAR_RIGHT_INDEX];
}

uint16_t LineArray::rawChannel(uint8_t index) const {
    return (index < 8) ? _raw[index] : 0;
}

uint16_t LineArray::normalisedChannel(uint8_t index) const {
    return (index < 8) ? _normalised[index] : 0;
}
