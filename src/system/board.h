#pragma once

#include <cstdint>
#include "../config.h"
#include "nvs_store.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#endif

/**
 * @brief Manages motherboard hardware bus, expander pins, power, and reset sequencing.
 */
class Board {
public:
    Board();

    static constexpr uint8_t DEFAULT_BRIGHTNESS_PERCENT = 95;
    static uint8_t percentToPwm(uint8_t percent);

    void init();
    void setBacklight(uint8_t brightness = 250);
    void setBacklightPercent(uint8_t percent, bool persist = true);
    uint8_t getBacklightPercent() const { return brightnessPercent_; }
    uint8_t getLastPwmBrightness() const { return lastPwmBrightness_; }
    void restart();

#if defined(ARDUINO)
    PCF8574& getPcf8574() { return pcf8574_; }
    bool isEncoderSwPressed();
#else
    bool isEncoderSwPressed() const { return mockEncoderSwPressed_; }
    void setMockEncoderSwPressed(bool pressed) { mockEncoderSwPressed_ = pressed; }
#endif

private:
    uint8_t brightnessPercent_ = DEFAULT_BRIGHTNESS_PERCENT;
    uint8_t lastPwmBrightness_ = 255;
#if defined(ARDUINO)
    PCF8574 pcf8574_;
#else
    bool mockEncoderSwPressed_ = false;
#endif
};

