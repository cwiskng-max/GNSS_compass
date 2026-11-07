#include "board.h"

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

    setBacklight(250);
}

void Board::setBacklight(uint8_t brightness) {
    pinMode(BKL_PIN, OUTPUT);
    analogWrite(BKL_PIN, brightness);
}

bool Board::isEncoderSwPressed() {
    return pcf8574_.digitalRead(P5) == LOW;
}

#else

Board::Board() {}
void Board::init() {}
void Board::setBacklight(uint8_t) {}

#endif

