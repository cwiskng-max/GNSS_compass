#include "calibration.h"
#include "../geo/geo_math.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

Calibration::Calibration()
    : activeOffset_(0.0f), proposedOffset_(0.0f), cfgStore_("compass") {}

void Calibration::begin() {
    loadFromNvs();
    proposedOffset_ = activeOffset_;
}

void Calibration::loadFromNvs() {
    activeOffset_ = cfgStore_.getFloat("cal_offset", 0.0f);
#if defined(ARDUINO)
    Serial.printf("[NVS] Loaded antenna calibration offset: %+5.1f deg\r\n", activeOffset_);
#endif
}

void Calibration::saveToNvs(float offsetDeg) {
    cfgStore_.putFloat("cal_offset", offsetDeg);
#if defined(ARDUINO)
    Serial.printf("[NVS] Saved antenna calibration offset: %+5.1f deg\r\n", offsetDeg);
#endif
}

void Calibration::setOffset(float offsetDeg, bool persist) {
    offsetDeg = normalizeOffset(offsetDeg);
    if (std::abs(offsetDeg - activeOffset_) > 0.01f) {
        activeOffset_ = offsetDeg;
        if (persist) {
            saveToNvs(activeOffset_);
        }
    }
}

bool Calibration::snap(double targetBearingDeg, double rawHeadingDeg) {
    float newOffset = normalizeOffset((float)(targetBearingDeg - rawHeadingDeg));
    setOffset(newOffset);
    return true;
}

void Calibration::setProposedOffset(float offsetDeg) {
    proposedOffset_ = normalizeOffset(offsetDeg);
}

void Calibration::adjustProposedOffset(float deltaDeg) {
    proposedOffset_ = normalizeOffset(proposedOffset_ + deltaDeg);
}

void Calibration::snapProposed(double targetBearingDeg, double rawHeadingDeg) {
    proposedOffset_ = normalizeOffset((float)(targetBearingDeg - rawHeadingDeg));
}

double Calibration::apply(double rawHeadingDeg) const {
    return applyBearingOffset(rawHeadingDeg, activeOffset_);
}

