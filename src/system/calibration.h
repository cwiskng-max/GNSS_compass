#pragma once

#include <cstdint>
#include <cmath>

/**
 * @brief Manages antenna azimuth alignment calibration offset, staging, and NVS persistence.
 *
 * Provides non-volatile flash storage on ESP32-S3 via ESP-IDF Preferences under the "compass"
 * namespace with the "cal_offset" float key. Supports safe 2-stage offset adjustment (proposed vs
 * committed active) for modal UI tuning before writing to NOR flash.
 */
class Calibration {
public:
    Calibration();

    // Initializes calibration and loads stored offset from NVS
    void begin();

    // Active (committed) antenna offset
    float getOffset() const { return activeOffset_; }
    void setOffset(float offsetDeg, bool persist = true);
    void setActiveOffsetInMemory(float offsetDeg) { setOffset(offsetDeg, false); }
    void save(float offsetDeg) { setOffset(offsetDeg, true); }
    void reset() { setOffset(0.0f, true); }
    bool snap(double targetBearingDeg, double rawHeadingDeg);

    // Staging / Proposed offset (for Calibration Modal Dialog)
    float getProposedOffset() const { return proposedOffset_; }
    void setProposedOffset(float offsetDeg);
    void stageProposedOffset(float offsetDeg) { setProposedOffset(offsetDeg); }
    void adjustProposedOffset(float deltaDeg);
    void resetProposed() { proposedOffset_ = 0.0f; }
    void snapProposed(double targetBearingDeg, double rawHeadingDeg);
    void syncProposedFromActive() { proposedOffset_ = activeOffset_; }
    void commitProposed() { setOffset(proposedOffset_); }

    // Status & transformation helpers
    bool isCalibrated() const { return std::abs(activeOffset_) >= 0.05f; }
    double apply(double rawHeadingDeg) const;

    // Direct NVS storage operations
    void loadFromNvs();
    void saveToNvs(float offsetDeg);

#if !defined(ARDUINO)
    // Host test mock helpers
    static void resetHostStorage() { s_hostStorageOffset = 0.0f; }
    static float getHostStorage() { return s_hostStorageOffset; }
#endif

private:
    float activeOffset_;
    float proposedOffset_;

#if !defined(ARDUINO)
    static float s_hostStorageOffset;
#endif
};

