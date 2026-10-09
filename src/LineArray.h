/**
 * ============================================================================
 *  @file       LineArray.h
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      8x TCR5000 reflectance array: 6-sensor FRONT row (weighted
 *              centroid) + 2-sensor REAR pair, with per-channel calibration.
 *
 *  @note       Rev 3: split out of SensorsSystem.h into its own header so the
 *              line driver has a self-contained interface. SensorsSystem.h
 *              includes this file; nothing about the class changed.
 * ============================================================================
 */

#pragma once

#include <stdint.h>
#include "Config.h"

// ============================================================================
//  LineArray — 8x TCR5000 on ADC1, split 6 FRONT (indices 0-5) + 2 REAR (6-7)
// ============================================================================

/**
 * @brief Reflectance array driver: a 6-sensor FRONT row feeding a weighted
 *        centroid position estimate, plus a 2-sensor REAR pair with its own
 *        independent accessors.
 *
 * @details Raw ADC counts are useless across venues: ambient light, floor
 *          gloss and sensor ride height all shift them. begin() therefore does
 *          nothing but configure pins; readPosition() is only meaningful after
 *          a calibration sweep has established per-channel min/max, which the
 *          rulebook explicitly permits (and clocks) before the scoring run.
 *
 *          The REAR pair (indices 6-7) is sampled and calibrated exactly like
 *          every other channel, but never contributes to the centroid — see
 *          the file-level note in LineArray.cpp for why the split sits at
 *          index 6 and how the centroid math was re-derived for 6 sensors.
 *
 * @warning NOT thread-safe. Exactly one task may touch an instance at a time.
 *          During a run that is the Core-1 line task; during CALIBRATING it is
 *          the Core-0 mission task, and the line task is gated off.
 */
class LineArray {
public:
    LineArray();

    void begin();

    /**
     * @brief Accumulates one calibration sample into the per-channel extrema.
     * @note  Call repeatedly (~1 kHz) while the operator sweeps the array
     *        across the line. Never call during a scoring run.
     */
    void calibrateSample();

    /** Wipes calibration extrema. Call once before a calibration sweep. */
    void calibrateReset();

    /**
     * @brief Declares the calibration sweep finished.                 [FIX 7]
     *
     * Until this is called, isCalibrated() stays false no matter how healthy
     * the channel spreads look. Without it, read() would flip _calibrated true
     * the instant the operator's hand gave every front channel a >300 count
     * spread — potentially a few hundred milliseconds into a four-second
     * sweep — and stepCalibrating() could succeed on a half-swept array.
     */
    void calibrateFinish();

    /** @return true if the sweep has been finished AND every FRONT channel has
     *          a usable dynamic range. A dead REAR channel does not affect
     *          this — see read(). */
    bool isCalibrated() const;

    /** @brief Samples all 8 physical channels and updates the cached readings. */
    void read();

    /**
     * @brief Normalised lateral line position from the 6-sensor FRONT row.
     * @return -1.0 (line at the leftmost front sensor) .. +1.0 (rightmost).
     *         Returns the last valid value if the line is currently lost.
     */
    float readPosition() const;

    /** @return Bitmask over all 8 physical channels, bit N set when sensor N
     *          reads darker than threshold (front AND rear). */
    uint8_t digitalMask() const;

    /** @return Count of FRONT channels (indices 0-5) currently over the line. */
    uint8_t activeCount() const;

    /** @return true when no FRONT channel sees the line. */
    bool isLineLost() const;

    /**
     * @brief Full-width dark reading across the 6-sensor FRONT row, consistent
     *        with an intersection bar or the black speed-bump crossing strip.
     * @note  This is a hypothesis, not a conclusion. The mission layer must
     *        corroborate it with the colour sensors before committing a turn.
     *        The REAR pair never contributes to this check.
     */
    bool isIntersectionCandidate() const;

    // --- Rear pair (indices 6-7) — auxiliary, no centroid role --------------

    /** @return true when the LEFT rear sensor (index 6) reads darker than threshold. */
    bool isRearLeftActive() const;

    /** @return true when the RIGHT rear sensor (index 7) reads darker than threshold. */
    bool isRearRightActive() const;

    /**
     * @return true when EITHER rear sensor sees the line.
     * @note  OR, not AND — the rear pair's footprint is much narrower than
     *        the front row's, so requiring both would miss the line under
     *        any real skew of the chassis tail.
     */
    bool isRearOnLine() const;

    uint16_t rawRearLeft()   const;
    uint16_t rawRearRight()  const;
    uint16_t normalisedRearLeft()  const;
    uint16_t normalisedRearRight() const;

    // --- Generic per-channel access (any of the 8 physical channels) -------

    uint16_t rawChannel(uint8_t index) const;
    uint16_t normalisedChannel(uint8_t index) const;

private:
    uint16_t _raw[8];
    uint16_t _normalised[8];   ///< 0 (bright) .. 1000 (dark)
    uint16_t _calMin[8];
    uint16_t _calMax[8];
    uint8_t  _mask;
    float    _lastPosition;
    bool     _lineLost;
    bool     _calibrated;
    bool     _calComplete;     ///< [FIX 7] set by calibrateFinish()
};
