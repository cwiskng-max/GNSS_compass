#pragma once
#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Circular moving-window vector averager for dual-antenna GNSS heading smoothing
class BearingFilter {
public:
    static constexpr uint8_t WINDOW_SIZE = 5;

    BearingFilter();

    // Clears the buffer and resets running sums to zero
    void reset();

    // Pre-populates all window slots with a fixed heading (useful for simulation or cold start)
    void seed(float headingDeg);

    // Pushes a new raw GPS heading into the circular moving window
    void push(float headingDeg);

    // Returns the smoothed heading with the +270 deg orthogonal antenna boom offset applied
    double getRawHeading() const;

    // Returns the smoothed heading with both +270 deg boom offset and user calibration offset applied
    double getCalibratedHeading(float offsetDeg = 0.0f) const;

    // Returns true if at least one sample has been pushed
    bool hasSamples() const { return count > 0; }

    // Returns the current number of valid samples in the window (0 to WINDOW_SIZE)
    uint8_t sampleCount() const { return count; }

private:
    float sinBuffer[WINDOW_SIZE];
    float cosBuffer[WINDOW_SIZE];
    float sinSum;
    float cosSum;
    uint8_t index;
    uint8_t count;
};

