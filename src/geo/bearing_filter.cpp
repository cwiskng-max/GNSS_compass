#include "bearing_filter.h"
#include <cctype>
#include <cmath>
#include <cstring>

namespace {

bool eqIgnoreCase(const char *a, const char *b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (std::tolower(static_cast<unsigned char>(*a)) !=
            std::tolower(static_cast<unsigned char>(*b))) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

double wrapDegrees360(double deg) {
    double b = std::fmod(deg, 360.0);
    if (b < 0.0) {
        b += 360.0;
    }
    return b;
}

} // namespace

// ---------------------------------------------------------------------------
// CircularEmaFilter
// ---------------------------------------------------------------------------

CircularEmaFilter::CircularEmaFilter()
    : sinState_(0.0),
      cosState_(0.0),
      lastUpdateMs_(0),
      hasLastUpdateMs_(false),
      sampleCount_(0),
      level_(HeadingAveragingLevel::Medium) {
}

void CircularEmaFilter::reset() {
    sinState_ = 0.0;
    cosState_ = 0.0;
    lastUpdateMs_ = 0;
    hasLastUpdateMs_ = false;
    sampleCount_ = 0;
}

void CircularEmaFilter::seed(float angleDeg, unsigned long nowMs) {
    double rad = static_cast<double>(angleDeg) * M_PI / 180.0;
    sinState_ = std::sin(rad);
    cosState_ = std::cos(rad);
    lastUpdateMs_ = nowMs;
    hasLastUpdateMs_ = (nowMs > 0);
    sampleCount_ = 1;
}

void CircularEmaFilter::push(float angleDeg, BearingQuality quality, unsigned long nowMs) {
    double rad = static_cast<double>(angleDeg) * M_PI / 180.0;
    double s = std::sin(rad);
    double c = std::cos(rad);

    if (sampleCount_ == 0) {
        sinState_ = s;
        cosState_ = c;
        lastUpdateMs_ = nowMs;
        hasLastUpdateMs_ = (nowMs > 0);
        sampleCount_ = 1;
        return;
    }

    double tau = timeConstantForLevel(level_);
    if (tau > 0.0 && quality == BEARING_FLOAT) {
        tau *= FLOAT_TAU_MULTIPLIER;
    }

    double alpha = 1.0;
    if (tau > 0.0) {
        double dtSec = DEFAULT_DT_SEC;
        if (nowMs > 0 && hasLastUpdateMs_ && nowMs > lastUpdateMs_) {
            dtSec = static_cast<double>(nowMs - lastUpdateMs_) / 1000.0;
            if (dtSec > MAX_DT_SEC) {
                dtSec = MAX_DT_SEC;
            }
        }
        alpha = 1.0 - std::exp(-dtSec / tau);
    }

    sinState_ = (1.0 - alpha) * sinState_ + alpha * s;
    cosState_ = (1.0 - alpha) * cosState_ + alpha * c;

    double norm = std::hypot(sinState_, cosState_);
    if (norm > 1e-9) {
        sinState_ /= norm;
        cosState_ /= norm;
    } else {
        sinState_ = s;
        cosState_ = c;
    }

    if (nowMs > 0) {
        lastUpdateMs_ = nowMs;
        hasLastUpdateMs_ = true;
    }
    if (sampleCount_ < 255) {
        sampleCount_++;
    }
}

void CircularEmaFilter::predict(float /* angularVelDegPerSec */, float /* dtSec */, bool /* driveActive */) {
    // Open-loop EMA has no angular-rate prediction state; intentionally a no-op.
}

double CircularEmaFilter::getFilteredAngle() const {
    if (sampleCount_ == 0) {
        return 0.0;
    }
    return wrapDegrees360(std::atan2(sinState_, cosState_) * 180.0 / M_PI);
}

void CircularEmaFilter::setAveragingLevel(HeadingAveragingLevel level) {
    level_ = level;
}

double CircularEmaFilter::timeConstantForLevel(HeadingAveragingLevel level) {
    switch (level) {
        case HeadingAveragingLevel::Low:
            return TAU_LOW_SEC;
        case HeadingAveragingLevel::High:
            return TAU_HIGH_SEC;
        case HeadingAveragingLevel::Medium:
        default:
            return TAU_MEDIUM_SEC;
    }
}

// ---------------------------------------------------------------------------
// BearingFilter
// ---------------------------------------------------------------------------

BearingFilter::BearingFilter()
    : activeFilter_(&emaFilter_),
      operatingMode_(FilterOperatingMode::OpenLoop),
      level_(HeadingAveragingLevel::Medium) {
    activeFilter_->setAveragingLevel(level_);
}

void BearingFilter::begin() {
    uint16_t raw = NvsConfig(NVS_NAMESPACE).getUShort(
        NVS_AVG_KEY, static_cast<uint16_t>(HeadingAveragingLevel::Medium));
    HeadingAveragingLevel loaded = HeadingAveragingLevel::Medium;
    if (raw == static_cast<uint16_t>(HeadingAveragingLevel::Low)) {
        loaded = HeadingAveragingLevel::Low;
    } else if (raw == static_cast<uint16_t>(HeadingAveragingLevel::High)) {
        loaded = HeadingAveragingLevel::High;
    }
    setAveragingLevel(loaded, false);
}

void BearingFilter::reset() {
    activeFilter_->reset();
}

void BearingFilter::seed(float headingDeg, unsigned long nowMs) {
    activeFilter_->seed(headingDeg, nowMs);
}

void BearingFilter::push(float headingDeg, BearingQuality quality, unsigned long nowMs) {
    activeFilter_->push(headingDeg, quality, nowMs);
}

void BearingFilter::predict(float angularVelDegPerSec, float dtSec, bool driveActive) {
    activeFilter_->predict(angularVelDegPerSec, dtSec, driveActive);
}

double BearingFilter::getRawHeading() const {
    return getCalibratedHeading(0.0f);
}

double BearingFilter::getCalibratedHeading(float offsetDeg) const {
    return wrapDegrees360(activeFilter_->getFilteredAngle() + 270.0 + static_cast<double>(offsetDeg));
}

void BearingFilter::setAveragingLevel(HeadingAveragingLevel level, bool persist) {
    level_ = level;
    emaFilter_.setAveragingLevel(level);
    if (activeFilter_ != &emaFilter_) {
        activeFilter_->setAveragingLevel(level);
    }
    if (persist) {
        NvsConfig(NVS_NAMESPACE).putUShort(NVS_AVG_KEY, static_cast<uint16_t>(level));
    }
}

void BearingFilter::setOperatingMode(FilterOperatingMode mode) {
    operatingMode_ = mode;
    // Phase 1: OpenLoop uses CircularEmaFilter; ClosedLoop Kalman will be attached in Phase 2.
    if (mode == FilterOperatingMode::OpenLoop) {
        activeFilter_ = &emaFilter_;
        activeFilter_->setAveragingLevel(level_);
    }
}

void BearingFilter::setFilterStrategy(IAngleFilter *customFilter) {
    activeFilter_ = customFilter ? customFilter : &emaFilter_;
    activeFilter_->setAveragingLevel(level_);
}

const char* BearingFilter::levelToString(HeadingAveragingLevel level) {
    switch (level) {
        case HeadingAveragingLevel::Low:    return "LOW";
        case HeadingAveragingLevel::High:   return "HIGH";
        case HeadingAveragingLevel::Medium:
        default:                            return "MEDIUM";
    }
}

bool BearingFilter::parseLevelString(const char *str, HeadingAveragingLevel &outLevel) {
    if (!str) return false;
    while (*str == ' ' || *str == '\t') ++str;
    size_t len = std::strlen(str);
    while (len > 0 && (str[len - 1] == ' ' || str[len - 1] == '\t')) {
        --len;
    }
    if (len == 0 || len > 12) return false;

    char buf[16];
    std::memcpy(buf, str, len);
    buf[len] = '\0';

    if (eqIgnoreCase(buf, "low") || eqIgnoreCase(buf, "lo") ||
        eqIgnoreCase(buf, "raw") || eqIgnoreCase(buf, "off") ||
        eqIgnoreCase(buf, "fast")) {
        outLevel = HeadingAveragingLevel::Low;
        return true;
    }
    if (eqIgnoreCase(buf, "med") || eqIgnoreCase(buf, "medium") ||
        eqIgnoreCase(buf, "normal") || eqIgnoreCase(buf, "default")) {
        outLevel = HeadingAveragingLevel::Medium;
        return true;
    }
    if (eqIgnoreCase(buf, "high") || eqIgnoreCase(buf, "hi") ||
        eqIgnoreCase(buf, "smooth") || eqIgnoreCase(buf, "slow") ||
        eqIgnoreCase(buf, "max")) {
        outLevel = HeadingAveragingLevel::High;
        return true;
    }
    return false;
}
