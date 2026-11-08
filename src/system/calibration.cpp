#include "calibration.h"
#include "../geo/geo_math.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <Preferences.h>
#else
#include <cstdio>
float Calibration::s_hostStorageOffset = 0.0f;
#endif

Calibration::Calibration()
    : activeOffset_(0.0f), proposedOffset_(0.0f) {}

void Calibration::begin() {
    loadFromNvs();
    proposedOffset_ = activeOffset_;
}

void Calibration::loadFromNvs() {
#if defined(ARDUINO)
    Preferences prefs;
    prefs.begin("compass", true);
    activeOffset_ = prefs.getFloat("cal_offset", 0.0f);
    prefs.end();
    Serial.printf("[NVS] Loaded antenna calibration offset: %+5.1f deg\r\n", activeOffset_);
#else
    activeOffset_ = s_hostStorageOffset;
#endif
}

void Calibration::saveToNvs(float offsetDeg) {
#if defined(ARDUINO)
    Preferences prefs;
    prefs.begin("compass", false);
    prefs.putFloat("cal_offset", offsetDeg);
    prefs.end();
    Serial.printf("[NVS] Saved antenna calibration offset: %+5.1f deg\r\n", offsetDeg);
#else
    s_hostStorageOffset = offsetDeg;
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

void Calibration::commitProposed() {
    setOffset(proposedOffset_);
}

double Calibration::apply(double rawHeadingDeg) const {
    return applyBearingOffset(rawHeadingDeg, activeOffset_);
}

