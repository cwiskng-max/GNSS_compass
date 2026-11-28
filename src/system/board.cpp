#include "board.h"

uint8_t Board::percentToPwm(uint8_t percent) {
    if (percent > 100) percent = 100;
    return static_cast<uint8_t>((static_cast<uint16_t>(percent) * 255U + 50U) / 100U);
}

void Board::setBacklightPercent(uint8_t percent, bool persist) {
    if (percent > 100) percent = 100;
    brightnessPercent_ = percent;
    setBacklight(percentToPwm(percent));
    if (persist) {
        NvsConfig("compass").putUShort("bkl_pct", percent);
    }
}

#if defined(ARDUINO)
Board::Board() : pcf8574_(PCF8574_ADDR) {}

void Board::init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    pcf8574_.pinMode(P0, OUTPUT);        // tp RST
    pcf8574_.pinMode(P2, OUTPUT);        // tp INT
    pcf8574_.pinMode(P3, OUTPUT);        // lcd power
    pcf8574_.pinMode(P4, OUTPUT);        // lcd reset
    pcf8574_.pinMode(P5, INPUT_PULLUP);  // encoder SW

    if (!pcf8574_.begin()) {
        Serial.println(F("Can't init pcf8574"));
    }

    // LCD reset sequence
    pcf8574_.digitalWrite(P3, HIGH);
    delay(100);
    pcf8574_.digitalWrite(P4, HIGH);
    delay(100);
    pcf8574_.digitalWrite(P4, LOW);
    delay(120);
    pcf8574_.digitalWrite(P4, HIGH);
    delay(120);

    // Touchpad reset sequence
    pcf8574_.digitalWrite(P0, HIGH);
    delay(100);
    pcf8574_.digitalWrite(P0, LOW);
    delay(120);
    pcf8574_.digitalWrite(P0, HIGH);
    delay(120);
    pcf8574_.digitalWrite(P2, HIGH);
    delay(120);

    uint16_t savedPct = NvsConfig("compass").getUShort("bkl_pct", DEFAULT_BRIGHTNESS_PERCENT);
    if (savedPct > 100) savedPct = DEFAULT_BRIGHTNESS_PERCENT;
    setBacklightPercent(static_cast<uint8_t>(savedPct), false);
}

void Board::setBacklight(uint8_t brightness) {
    lastPwmBrightness_ = brightness;
    pinMode(BKL_PIN, OUTPUT);
    analogWrite(BKL_PIN, brightness);
}

bool Board::isEncoderSwPressed() {
    return pcf8574_.digitalRead(P5) == LOW;
}

void Board::restart() {
#if defined(ARDUINO)
    setBacklight(0);
    delay(100);
    esp_restart();
#endif
}

#else

Board::Board() {}
void Board::init() {
    uint16_t savedPct = NvsConfig("compass").getUShort("bkl_pct", DEFAULT_BRIGHTNESS_PERCENT);
    if (savedPct > 100) savedPct = DEFAULT_BRIGHTNESS_PERCENT;
    setBacklightPercent(static_cast<uint8_t>(savedPct), false);
}
void Board::setBacklight(uint8_t brightness) {
    lastPwmBrightness_ = brightness;
}
void Board::restart() {}

#endif

