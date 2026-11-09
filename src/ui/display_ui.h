#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "../config.h"
#include "../types.h"
#include "cal_modal.h"
#include "rover_card.h"
#include "rover_history_modal.h"

#if defined(ARDUINO)
#include <Arduino_GFX_Library.h>
#endif

/**
 * @brief View model bundle representing the full display state of the main operational screen.
 */
struct MainScreenState {
    const char *myGrid = nullptr;
    const char *targetGrid = nullptr;
    uint8_t cursorColumn = 0;
    bool isEditing = false;
    double targetBearing = 0.0;
    double revBearing = 0.0;
    double distanceMeters = 0.0;
    bool isRoverMode = false;
    RoverState roverState = RoverState::InTransitUnder16k;
    double roverDisplacementKm = 0.0;
    double currentHeading = 0.0;
    BearingQuality bearingQuality = BearingQuality::None;
    bool hasHeading = false;
    float calOffset = 0.0f;
    bool isCalibrated = false;
    int hours = 0;
    int minutes = 0;
    bool hasTime = false;
};

class DisplayUi {
public:
    DisplayUi();

    void begin();
    void initLcd();
    void clearScreen(uint16_t color = 0x0000 /* BLACK */);

    // Screen Renderers
    void renderMyGrid(const char *grid);
    void renderTargetGrid(const char *grid, uint8_t cursorCol, bool isEditing);
    void renderCurrentBearing(double heading, BearingQuality quality, bool hasHeading);
    void renderCalibrationPill(float offset, bool isCalibrated);
    void renderTargetBearing(double targetBearing);
    void renderReverseBearingAndDistance(double revBearing, double distanceMeters);
    void renderRoverCard(RoverState state, double displacementKm);
    void clearRoverCard();
    void renderArrlDistance(const char *myGrid, const char *targetGrid);
    void renderUtcTime(int hours, int minutes, bool hasTime);
    void showFooterText(const char *msg, uint16_t color = 0x7BEF);
    void showStatusCard(const char *msg, uint16_t bgColor, uint16_t fgColor);

    void redrawMainScreen(const MainScreenState &state);

    CalModal &getCalModal() { return calModal_; }
    RoverHistoryModal &getRoverHistoryModal() { return roverHistoryModal_; }

#if defined(ARDUINO)
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
    RoverHistoryModal roverHistoryModal_;
    char lastRoverCardBuf_[24] = {0};
    RoverState lastRoverCardState_ = static_cast<RoverState>(-1);

#if defined(ARDUINO)
    Arduino_ESP32RGBPanel *bus_ = nullptr;
    Arduino_ST7701_RGBPanel *gfx_ = nullptr;
#endif
};

