#pragma once

#include <cstdint>
#include "angle_filter.h"
#include "../system/nvs_store.h"

/**
 * @brief Dual-antenna heading filter facade hosting a pluggable IAngleFilter strategy.
 *
 * Phase 1 uses CircularEmaFilter in Open-Loop mode with 3 NVS-persisted averaging levels
 * (Low, Medium, High). Phase 2 will allow automatic switching to a Closed-Loop Kalman
 * strategy when rotator control is active.
 */
class BearingFilter {
public:
    static constexpr const char* NVS_NAMESPACE = "compass";
    static constexpr const char* NVS_AVG_KEY = "avg_level";

    BearingFilter();

    // Loads persisted averaging level from NVS
    void begin();

    void reset();
    void seed(float headingDeg, unsigned long nowMs = 0);
    void push(float headingDeg, BearingQuality quality = BEARING_FIXED, unsigned long nowMs = 0);
    void predict(float angularVelDegPerSec, float dtSec, bool driveActive);

    // Returns smoothed heading with +270 deg orthogonal antenna boom offset applied
    double getRawHeading() const;

    // Returns smoothed heading with +270 deg boom offset and user calibration offset applied
    double getCalibratedHeading(float offsetDeg = 0.0f) const;

    bool hasSamples() const { return activeFilter_->hasSamples(); }
    uint8_t sampleCount() const { return activeFilter_->sampleCount(); }

    // Averaging depth & NVS persistence
    void setAveragingLevel(HeadingAveragingLevel level, bool persist = true);
    HeadingAveragingLevel getAveragingLevel() const { return level_; }
    double getTimeConstantSec() const { return CircularEmaFilter::timeConstantForLevel(level_); }

    // Pluggable strategy / operating mode selection (Phase 2 seam)
    void setOperatingMode(FilterOperatingMode mode);
    FilterOperatingMode getOperatingMode() const { return operatingMode_; }
    void setFilterStrategy(IAngleFilter *customFilter);
    const char* getStrategyName() const { return activeFilter_->getStrategyName(); }

    // Formatting helpers
    static const char* levelToString(HeadingAveragingLevel level);
    static bool parseLevelString(const char *str, HeadingAveragingLevel &outLevel);

private:
    CircularEmaFilter emaFilter_;
    IAngleFilter *activeFilter_;
    FilterOperatingMode operatingMode_;
    HeadingAveragingLevel level_;
};
