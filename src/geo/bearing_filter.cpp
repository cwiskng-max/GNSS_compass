#include "bearing_filter.h"

BearingFilter::BearingFilter() {
    reset();
}

void BearingFilter::reset() {
    for (uint8_t i = 0; i < WINDOW_SIZE; ++i) {
        sinBuffer[i] = 0.0f;
        cosBuffer[i] = 0.0f;
    }
    sinSum = 0.0f;
    cosSum = 0.0f;
    index = 0;
    count = 0;
}

void BearingFilter::seed(float headingDeg) {
    float rad = headingDeg * (float)M_PI / 180.0f;
    float s = std::sin(rad);
    float c = std::cos(rad);

    for (uint8_t i = 0; i < WINDOW_SIZE; ++i) {
        sinBuffer[i] = s;
        cosBuffer[i] = c;
    }
    sinSum = s * (float)WINDOW_SIZE;
    cosSum = c * (float)WINDOW_SIZE;
    index = 0;
    count = WINDOW_SIZE;
}

void BearingFilter::push(float headingDeg) {
    float rad = headingDeg * (float)M_PI / 180.0f;
    float s = std::sin(rad);
    float c = std::cos(rad);

    sinSum -= sinBuffer[index];
    cosSum -= cosBuffer[index];

    sinBuffer[index] = s;
    cosBuffer[index] = c;

    sinSum += s;
    cosSum += c;

    index = (index + 1) % WINDOW_SIZE;
    if (count < WINDOW_SIZE) {
        count++;
    }
}

double BearingFilter::getRawHeading() const {
    return getCalibratedHeading(0.0f);
}

double BearingFilter::getCalibratedHeading(float offsetDeg) const {
    double b = std::fmod(
        (std::atan2((double)sinSum, (double)cosSum) * 180.0 / M_PI) + 270.0 + (double)offsetDeg,
        360.0
    );
    if (b < 0.0) {
        b += 360.0;
    }
    return b;
}

