#pragma once

#include <cstdint>
#include "../config.h"

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

    void init();
    void setBacklight(uint8_t brightness = 250);

#if defined(ARDUINO)
    PCF8574& getPcf8574() { return pcf8574_; }
    bool isEncoderSwPressed();
#else
    bool isEncoderSwPressed() const { return mockEncoderSwPressed_; }
    void setMockEncoderSwPressed(bool pressed) { mockEncoderSwPressed_ = pressed; }
#endif

private:
#if defined(ARDUINO)
    PCF8574 pcf8574_;
#else
    bool mockEncoderSwPressed_ = false;
#endif
};

