#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "../config.h"
#include "../types.h"
#include "cal_modal.h"

#if defined(ARDUINO)
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <PCF8574.h>
#endif

class DisplayUi {
public:
    DisplayUi();

    void begin();
    void initHardware();
    void initBacklight();
    void initLcd();

    // Screen Renderers
    void renderMyGrid(const char *grid);
    void renderTargetGrid(const char *grid, uint8_t cursorCol, bool isEditing);
    void renderCurrentBearing(double heading, BearingQuality quality, bool hasHeading);
    void renderCalibrationPill(float offset, bool isCalibrated);
    void renderTargetBearing(double targetBearing);
    void renderReverseBearingAndDistance(double revBearing, double distanceMeters, bool isRoverAlertActive);
    void renderArrlDistance(const char *myGrid, const char *targetGrid);
    void renderUtcTime(int hours, int minutes, bool hasTime);
    void showFooterText(const char *msg, uint16_t color = 0x7BEF);
    void showStatusCard(const char *msg, uint16_t bgColor, uint16_t fgColor);

    void redrawMainScreen(const char *myGrid, const char *targetGrid, uint8_t cursorCol, bool isEditing,
                          double targetBearing, double revBearing, double distanceMeters, bool isRoverAlertActive,
                          double heading, BearingQuality quality, bool hasHeading,
                          float calOffset, bool isCalibrated,
                          int hours, int minutes, bool hasTime);

    CalModal &getCalModal() { return calModal_; }

#if defined(ARDUINO)
    PCF8574 &getPcf8574() { return pcf8574_; }
    Arduino_GFX *getGfx() const { return gfx_; }
#endif

    // Static text formatters (host testable)
    static void formatTimeString(int hours, int minutes, bool hasTime, char *outBuf, size_t bufSize);
    static void formatReverseBearingAndDistance(double revBearing, double distanceMeters, char *outBuf, size_t bufSize);
    static void formatArrlDistance(double arrlDistKm, char *outBuf, size_t bufSize);
    static void formatBearingString(double heading, bool hasHeading, char *outBuf, size_t bufSize);
    static void formatCalibrationPillText(float offset, bool isCalibrated, char *outBuf, size_t bufSize);
    static void formatSelfTestFailure(const int *failedTests, int count, char *outBuf, size_t bufSize);

private:
    CalModal calModal_;

#if defined(ARDUINO)
    PCF8574 pcf8574_;
    Arduino_ESP32RGBPanel *bus_ = nullptr;
    Arduino_ST7701_RGBPanel *gfx_ = nullptr;
#endif
};

