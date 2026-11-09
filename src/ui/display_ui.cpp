#include "display_ui.h"
#include "../geo/geo_math.h"
#include <cstring>
#include <cmath>

void DisplayUi::formatTimeString(int hours, int minutes, bool hasTime, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (hasTime) {
        snprintf(outBuf, bufSize, "%02d:%02dZ", hours, minutes);
    } else {
        snprintf(outBuf, bufSize, "--:--Z");
    }
}

void DisplayUi::formatReverseBearingAndDistance(double revBearing, double distanceMeters, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    int rdist = static_cast<int>(std::floor(distanceMeters / 1000.0));
    snprintf(outBuf, bufSize, "%5.1f %4i k  ", revBearing, rdist);
}

void DisplayUi::formatArrlDistance(double arrlDistKm, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    snprintf(outBuf, bufSize, "ARRL %6.1f k  ", arrlDistKm);
}

void DisplayUi::formatBearingString(double heading, bool hasHeading, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (hasHeading) {
        snprintf(outBuf, bufSize, "%5.1f", heading);
    } else {
        snprintf(outBuf, bufSize, "---.-");
    }
}

void DisplayUi::formatCalibrationPillText(float offset, bool isCalibrated, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (!isCalibrated) {
        snprintf(outBuf, bufSize, "OFF");
    } else {
        snprintf(outBuf, bufSize, "%+4.1f", offset);
    }
}

void DisplayUi::formatSelfTestFailure(const int *failedTests, int count, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (!failedTests || count <= 0) {
        snprintf(outBuf, bufSize, "SELF-TEST PASSED");
        return;
    }
    if (count == 1) {
        snprintf(outBuf, bufSize, "FAILED: TEST #%d", failedTests[0]);
    } else {
        snprintf(outBuf, bufSize, "FAILED: #%d", failedTests[0]);
        for (int i = 1; i < count && strlen(outBuf) < bufSize - 5; i++) {
            char numBuf[8];
            snprintf(numBuf, sizeof(numBuf), ", #%d", failedTests[i]);
            strncat(outBuf, numBuf, bufSize - strlen(outBuf) - 1);
        }
    }
}

#if defined(ARDUINO)
DisplayUi::DisplayUi() {}

void DisplayUi::initLcd() {
    bus_ = new Arduino_ESP32RGBPanel(
        16 /* CS */, 2 /* SCK */, 1 /* SDA */,
        40 /* DE */, 7 /* VSYNC */, 15 /* HSYNC */, 41 /* PCLK */,
        46 /* R0 */, 3 /* R1 */, 8 /* R2 */, 18 /* R3 */, 17 /* R4 */,
        14 /* G0 */, 13 /* G1 */, 12 /* G2 */, 11 /* G3 */, 10 /* G4 */, 9 /* G5 */,
        5 /* B0 */, 45 /* B1 */, 48 /* B2 */, 47 /* B3 */, 21 /* B4 */
    );

    gfx_ = new Arduino_ST7701_RGBPanel(
        bus_,
        GFX_NOT_DEFINED,  // RST pin (not used, reset via PCF8574)
        0,                // rotation
        false,            // IPS
        SCREEN_WIDTH, SCREEN_HEIGHT,
        st7701_type5_init_operations,
        sizeof(st7701_type5_init_operations),
        true,       // BGR
        10, 4, 20,  // hsync front porch, pulse width, back porch
        10, 4, 20   // vsync front porch, pulse width, back porch
    );

    gfx_->begin();
    gfx_->fillScreen(BLACK);
    gfx_->setTextColor(WHITE, BLACK);

    calModal_.setGfx(gfx_);
    roverHistoryModal_.setGfx(gfx_);
}

void DisplayUi::begin() {
    initLcd();
}

void DisplayUi::renderMyGrid(const char *grid) {
    if (!gfx_ || !grid) return;
    gfx_->setTextSize(LOC_SZ);
    gfx_->setCursor(LOC_X, LOC_Y);
    gfx_->setTextColor(WHITE, BLACK);
    gfx_->print(grid);
}

void DisplayUi::renderTargetGrid(const char *grid, uint8_t cursorCol, bool isEditing) {
    if (!gfx_ || !grid) return;

    gfx_->setTextSize(REMSIZE);
    for (int i = 0; i < 6; i++) {
        if (isEditing && i == cursorCol) {
            gfx_->setTextColor(RICH_YELLOW, BLACK);
        } else {
            gfx_->setTextColor(WHITE, BLACK);
        }
        gfx_->setCursor(REMSTARTX + i * CHAR_W(REMSIZE), REMSTARTY);
        gfx_->print(grid[i]);
    }

    gfx_->setTextColor(RICH_YELLOW, BLACK);
    gfx_->setCursor(REMSTARTX, REMSTARTY + CHAR_H(REMSIZE));
    gfx_->print("      ");
    if (isEditing) {
        gfx_->setCursor(REMSTARTX + cursorCol * CHAR_W(REMSIZE), REMSTARTY + CHAR_H(REMSIZE));
        gfx_->print("^");
    }

    gfx_->setTextColor(WHITE, BLACK);
}

void DisplayUi::renderCurrentBearing(double heading, BearingQuality quality, bool hasHeading) {
    if (!gfx_) return;

    uint16_t color = WHITE;
    if (quality == BEARING_FLOAT) {
        color = RICH_YELLOW;
    } else if (quality == BEARING_LOST) {
        color = RED;
    }

    gfx_->setTextSize(BEARSZ);
    gfx_->setCursor(BEARX, BEARY);
    gfx_->setTextColor(color, BLACK);

    char str[16];
    formatBearingString(heading, hasHeading, str, sizeof(str));
    gfx_->print(str);
    gfx_->setTextColor(WHITE, BLACK);
}

void DisplayUi::renderCalibrationPill(float offset, bool isCalibrated) {
    if (!gfx_) return;

    // Clear pill region
    gfx_->fillRect(CAL_PILL_X - 2, CAL_PILL_Y - 2, CAL_PILL_W + 4, CAL_PILL_H + 4, BLACK);

    uint16_t borderColor = DARK_GRAY_BORDER;
    uint16_t textColor = DIM_GRAY_TEXT;

    if (isCalibrated) {
        borderColor = CYAN;
        textColor = CYAN;
    }

    // Draw rounded pill frame
    gfx_->drawRoundRect(CAL_PILL_X, CAL_PILL_Y, CAL_PILL_W, CAL_PILL_H, 6, borderColor);

    // Line 1: "CAL"
    gfx_->setTextSize(2);
    gfx_->setTextColor(textColor, BLACK);
    gfx_->setCursor(CAL_PILL_X + 22, CAL_PILL_Y + 4);
    gfx_->print("CAL");

    // Line 2: "OFF" or "+4.2"
    char valStr[12];
    formatCalibrationPillText(offset, isCalibrated, valStr, sizeof(valStr));
    int txtW = strlen(valStr) * CHAR_W(2);
    gfx_->setCursor(CAL_PILL_X + (CAL_PILL_W - txtW) / 2, CAL_PILL_Y + 24);
    gfx_->print(valStr);
    gfx_->setTextColor(WHITE, BLACK);
}

void DisplayUi::renderTargetBearing(double targetBearing) {
    if (!gfx_) return;
    char buf[16];
    snprintf(buf, sizeof(buf), "%5.1f", targetBearing);
    gfx_->setTextSize(BEARSZ);
    gfx_->setCursor(TGTBEARX, BEARY);
    gfx_->setTextColor(WHITE, BLACK);
    gfx_->print(buf);
}

void DisplayUi::renderReverseBearingAndDistance(double revBearing, double distanceMeters) {
    if (!gfx_) return;

    char buf[24];
    formatReverseBearingAndDistance(revBearing, distanceMeters, buf, sizeof(buf));
    gfx_->setTextColor(WHITE, BLACK);
    gfx_->setTextSize(REVBEAR_SZ);
    gfx_->setCursor(REVBEAR_X, REVBEAR_Y);
    gfx_->print(buf);
}

void DisplayUi::renderRoverCard(RoverState state, double displacementKm) {
    if (!gfx_) return;

    char buf[20];
    formatRoverCard(state, displacementKm, buf, sizeof(buf));

    if (state == lastRoverCardState_ && std::strcmp(buf, lastRoverCardBuf_) == 0) {
        return;
    }
    lastRoverCardState_ = state;
    std::strncpy(lastRoverCardBuf_, buf, sizeof(lastRoverCardBuf_) - 1);
    lastRoverCardBuf_[sizeof(lastRoverCardBuf_) - 1] = '\0';

    gfx_->fillRoundRect(ROVER_CARD_X, ROVER_CARD_Y, ROVER_CARD_W, ROVER_CARD_H, ROVER_CARD_R, WHITE);

    uint16_t textColor = getRoverStateTextColor(state);
    gfx_->setTextColor(textColor, WHITE);
    gfx_->setTextSize(ROVER_CARD_SZ);
    gfx_->setCursor(ROVER_CARD_TEXT_X, ROVER_CARD_TEXT_Y);
    gfx_->print(buf);
    gfx_->setTextColor(WHITE, BLACK);
}

void DisplayUi::clearRoverCard() {
    if (!gfx_) return;
    lastRoverCardState_ = static_cast<RoverState>(-1);
    lastRoverCardBuf_[0] = '\0';
    gfx_->fillRect(ROVER_CARD_X - 2, ROVER_CARD_Y - 2, ROVER_CARD_W + 4, ROVER_CARD_H + 4, BLACK);
}

void DisplayUi::renderArrlDistance(const char *myGrid, const char *targetGrid) {
#if ENABLE_ARRL_DIST
    if (!gfx_ || !myGrid || !targetGrid) return;
    double arrl_dist = calculateArrlContestDistance(myGrid, targetGrid);
    char arrl_str[24];
    formatArrlDistance(arrl_dist, arrl_str, sizeof(arrl_str));
    gfx_->setTextSize(ARRL_SZ);
    gfx_->setCursor(ARRL_X, ARRL_Y);
    gfx_->setTextColor(WHITE, BLACK);
    gfx_->print(arrl_str);
#else
    (void)myGrid;
    (void)targetGrid;
#endif
}

void DisplayUi::renderUtcTime(int hours, int minutes, bool hasTime) {
    if (!gfx_) return;
    char tbuf[12];
    formatTimeString(hours, minutes, hasTime, tbuf, sizeof(tbuf));
    gfx_->setTextSize(TIME_SZ);
    gfx_->setCursor(TIME_X, TIME_Y);
    gfx_->setTextColor(WHITE, BLACK);
    gfx_->print(tbuf);
}

void DisplayUi::showFooterText(const char *msg, uint16_t color) {
    if (!gfx_ || !msg) return;
    gfx_->fillRect(0, VER_Y, SCREEN_WIDTH, CHAR_H(VER_SZ) + 4, BLACK);
    int w = strlen(msg) * CHAR_W(VER_SZ);
    gfx_->setTextSize(VER_SZ);
    gfx_->setTextColor(color, BLACK);
    gfx_->setCursor((SCREEN_WIDTH - w) / 2, VER_Y);
    gfx_->print(msg);
    gfx_->setTextColor(WHITE, BLACK);
}

void DisplayUi::showStatusCard(const char *msg, uint16_t bgColor, uint16_t fgColor) {
    if (!gfx_ || !msg) return;
    int w = strlen(msg) * CHAR_W(2);
    gfx_->fillRoundRect(40, 215, 400, 52, 8, bgColor);
    gfx_->setTextColor(fgColor, bgColor);
    gfx_->setTextSize(2);
    gfx_->setCursor((SCREEN_WIDTH - w) / 2, 233);
    gfx_->print(msg);
    gfx_->setTextColor(WHITE, BLACK);
}

void DisplayUi::clearScreen(uint16_t color) {
    if (!gfx_) return;
    gfx_->fillScreen(color);
}

void DisplayUi::redrawMainScreen(const MainScreenState &s) {
    if (!gfx_) return;
    clearScreen(BLACK);
    renderMyGrid(s.myGrid);
    renderTargetGrid(s.targetGrid, s.cursorColumn, s.isEditing);
    renderCurrentBearing(s.currentHeading, s.bearingQuality, s.hasHeading);
    renderCalibrationPill(s.calOffset, s.isCalibrated);
    renderTargetBearing(s.targetBearing);
    if (s.isRoverMode) {
        renderRoverCard(s.roverState, s.roverDisplacementKm);
    } else {
        renderReverseBearingAndDistance(s.revBearing, s.distanceMeters);
    }
    renderArrlDistance(s.myGrid, s.targetGrid);
    renderUtcTime(s.hours, s.minutes, s.hasTime);
    showFooterText(FW_VERSION, DIM_GRAY_TEXT);
}
#else
DisplayUi::DisplayUi() = default;
void DisplayUi::begin() {}
void DisplayUi::initLcd() {}
void DisplayUi::clearScreen(uint16_t) {}
void DisplayUi::renderMyGrid(const char *) {}
void DisplayUi::renderTargetGrid(const char *, uint8_t, bool) {}
void DisplayUi::renderCurrentBearing(double, BearingQuality, bool) {}
void DisplayUi::renderCalibrationPill(float, bool) {}
void DisplayUi::renderTargetBearing(double) {}
void DisplayUi::renderReverseBearingAndDistance(double, double) {}
void DisplayUi::renderRoverCard(RoverState, double) {}
void DisplayUi::clearRoverCard() {}
void DisplayUi::renderArrlDistance(const char *, const char *) {}
void DisplayUi::renderUtcTime(int, int, bool) {}
void DisplayUi::showFooterText(const char *, uint16_t) {}
void DisplayUi::showStatusCard(const char *, uint16_t, uint16_t) {}
void DisplayUi::redrawMainScreen(const MainScreenState &) {}
#endif

