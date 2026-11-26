#pragma once

#include <cstdint>
#include "../types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Discrete operator-selectable smoothing levels for angular filtering.
 */
enum class HeadingAveragingLevel : uint8_t {
    Low = 0,     // tau = 0.0 s (raw / zero lag)
    Medium = 1,  // tau = 0.5 s (default, ~0.33 alpha @ 5 Hz)
    High = 2     // tau = 1.8 s (~0.11 alpha @ 5 Hz)
};

/**
 * @brief System control loop mode determining automatic filter strategy selection.
 */
enum class FilterOperatingMode : uint8_t {
    OpenLoop = 0,   // Circular EMA (Phase 1)
    ClosedLoop = 1  // 2-State Circular Kalman (Phase 2)
};

/**
 * @brief Pluggable circular angle filter strategy interface.
 *
 * Decoupled from dual-antenna boom offsets so the same filter abstraction
 * can be reused for both Azimuth and future Elevation axes.
 */
class IAngleFilter {
public:
    virtual ~IAngleFilter() = default;

    virtual void reset() = 0;
    virtual void seed(float angleDeg, unsigned long nowMs = 0) = 0;
    virtual void push(float angleDeg, BearingQuality quality = BEARING_FIXED, unsigned long nowMs = 0) = 0;
    virtual void predict(float angularVelDegPerSec, float dtSec, bool driveActive) = 0;

    virtual double getFilteredAngle() const = 0;
    virtual bool hasSamples() const = 0;
    virtual uint8_t sampleCount() const = 0;

    virtual void setAveragingLevel(HeadingAveragingLevel level) = 0;
    virtual HeadingAveragingLevel getAveragingLevel() const = 0;
    virtual const char* getStrategyName() const = 0;
};

/**
 * @brief Open-loop circular Exponential Moving Average (EMA) in (sin, cos) vector space.
 *
 * Uses elapsed sample time dt for rate-invariant time constants and gently increases
 * damping on BEARING_FLOAT samples while remaining completely invariant across 0/360 wrap-around.
 */
class CircularEmaFilter : public IAngleFilter {
public:
    static constexpr double TAU_LOW_SEC = 0.0;
    static constexpr double TAU_MEDIUM_SEC = 0.5;
    static constexpr double TAU_HIGH_SEC = 1.8;
    static constexpr double DEFAULT_DT_SEC = 0.2;
    static constexpr double MAX_DT_SEC = 5.0;
    static constexpr double FLOAT_TAU_MULTIPLIER = 1.5;

    CircularEmaFilter();

    void reset() override;
    void seed(float angleDeg, unsigned long nowMs = 0) override;
    void push(float angleDeg, BearingQuality quality = BEARING_FIXED, unsigned long nowMs = 0) override;
    void predict(float angularVelDegPerSec, float dtSec, bool driveActive) override;

    double getFilteredAngle() const override;
    bool hasSamples() const override { return sampleCount_ > 0; }
    uint8_t sampleCount() const override { return sampleCount_; }

    void setAveragingLevel(HeadingAveragingLevel level) override;
    HeadingAveragingLevel getAveragingLevel() const override { return level_; }
    const char* getStrategyName() const override { return "Circular EMA"; }

    static double timeConstantForLevel(HeadingAveragingLevel level);

private:
    double sinState_;
    double cosState_;
    unsigned long lastUpdateMs_;
    bool hasLastUpdateMs_;
    uint8_t sampleCount_;
    HeadingAveragingLevel level_;
};
