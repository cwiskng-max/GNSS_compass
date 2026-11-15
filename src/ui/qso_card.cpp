#include "qso_card.h"
#include <cstring>
#include <cstdio>

void QsoCard::formatHeader(size_t qsoNumber, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (qsoNumber > 0) {
        snprintf(outBuf, bufSize, "*** QSO LOGGED #%zu ***", qsoNumber);
    } else {
        snprintf(outBuf, bufSize, "*** QSO LOGGED ***");
    }
}

void QsoCard::formatCallsign(const char *callsign, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    snprintf(outBuf, bufSize, "%s", callsign ? callsign : "");
}

void QsoCard::formatBandFreq(const char *band, uint32_t freqKhz, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    const char *b = (band && band[0]) ? band : "10 GHz";
    if (freqKhz > 0) {
        snprintf(outBuf, bufSize, "%s (%.3f MHz)", b, (double)freqKhz / 1000.0);
    } else {
        snprintf(outBuf, bufSize, "%s", b);
    }
}

void QsoCard::formatGrids(const char *myGrid, const char *hisGrid, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    const char *mg = (myGrid && myGrid[0]) ? myGrid : "------";
    const char *hg = (hisGrid && hisGrid[0]) ? hisGrid : "------";
    snprintf(outBuf, bufSize, "%s -> %s", mg, hg);
}

void QsoCard::formatDistanceBearing(float distanceKm, float bearingDeg, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    snprintf(outBuf, bufSize, "%.1f km @ %03.0f deg", distanceKm, bearingDeg);
}

void QsoCard::formatTime(uint32_t timestamp, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (timestamp > 0) {
        int h = (timestamp / 3600) % 24;
        int m = (timestamp / 60) % 60;
        snprintf(outBuf, bufSize, "Time: %02d:%02d UTC", h, m);
    } else {
        snprintf(outBuf, bufSize, "Time: --:-- UTC");
    }
}

bool QsoCard::isOkButtonTouched(int16_t x, int16_t y) {
    return (x >= QSO_TOUCH_OK_X_MIN && x <= QSO_TOUCH_OK_X_MAX &&
            y >= QSO_TOUCH_OK_Y_MIN && y <= QSO_TOUCH_OK_Y_MAX);
}

void QsoCard::show(const QsoEntry &qso, size_t qsoIndex, unsigned long nowMs) {
    visible_ = true;
    activeQso_ = qso;
    activeQsoIndex_ = qsoIndex;
    openedMs_ = nowMs;
}

void QsoCard::dismiss() {
    visible_ = false;
}

bool QsoCard::checkTimeout(unsigned long nowMs) {
    if (visible_ && (nowMs >= openedMs_) && ((nowMs - openedMs_) >= QSO_MODAL_TIMEOUT_MS)) {
        visible_ = false;
        return true;
    }
    return false;
}

#if defined(ARDUINO)
void QsoCard::drawModal(const QsoEntry &qso, size_t qsoIndex) {
    if (!gfx_) return;

    // Outer modal card with green visual confirmation border
    gfx_->fillRoundRect(QSO_MODAL_X, QSO_MODAL_Y, QSO_MODAL_W, QSO_MODAL_H, 12, MODAL_BG_COLOR);
    gfx_->drawRoundRect(QSO_MODAL_X, QSO_MODAL_Y, QSO_MODAL_W, QSO_MODAL_H, 12, COLOR_ROVER_QUALIFIED_GREEN);
    gfx_->drawRoundRect(QSO_MODAL_X + 1, QSO_MODAL_Y + 1, QSO_MODAL_W - 2, QSO_MODAL_H - 2, 11, COLOR_ROVER_QUALIFIED_GREEN);

    // Title / Status Badge
    char headerBuf[40];
    formatHeader(qsoIndex, headerBuf, sizeof(headerBuf));
    gfx_->setTextSize(2);
    gfx_->setTextColor(COLOR_ROVER_QUALIFIED_GREEN, MODAL_BG_COLOR);
    int hw = strlen(headerBuf) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - hw) / 2, 80);
    gfx_->print(headerBuf);

    // Separator line
    gfx_->drawFastHLine(60, 102, 360, DARK_GRAY_BORDER);

    // Prominent Callsign Header
    char callBuf[24];
    formatCallsign(qso.callsign, callBuf, sizeof(callBuf));
    int sz = (strlen(callBuf) > 7) ? 3 : 4;
    gfx_->setTextSize(sz);
    gfx_->setTextColor(WHITE, MODAL_BG_COLOR);
    int cw = strlen(callBuf) * CHAR_W(sz);
    gfx_->setCursor((SCREEN_WIDTH - cw) / 2, 116);
    gfx_->print(callBuf);

    // Separator line
    gfx_->drawFastHLine(60, 156, 360, DARK_GRAY_BORDER);

    // Band & Frequency
    char bandBuf[48];
    formatBandFreq(qso.band, qso.freq_khz, bandBuf, sizeof(bandBuf));
    gfx_->setTextSize(2);
    gfx_->setTextColor(CYAN, MODAL_BG_COLOR);
    int bw = strlen(bandBuf) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - bw) / 2, 170);
    gfx_->print(bandBuf);

    // Target Grid & My Grid
    char gridBuf[32];
    formatGrids(qso.my_grid, qso.his_grid, gridBuf, sizeof(gridBuf));
    gfx_->setTextSize(2);
    gfx_->setTextColor(WHITE, MODAL_BG_COLOR);
    int gw = strlen(gridBuf) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - gw) / 2, 204);
    gfx_->print(gridBuf);

    // Great-circle Distance & Bearing
    char distBuf[36];
    formatDistanceBearing(qso.distance_km, qso.bearing_deg, distBuf, sizeof(distBuf));
    gfx_->setTextSize(2);
    gfx_->setTextColor(RICH_YELLOW, MODAL_BG_COLOR);
    int dw = strlen(distBuf) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - dw) / 2, 238);
    gfx_->print(distBuf);

    // UTC Time
    char timeBuf[24];
    formatTime(qso.timestamp, timeBuf, sizeof(timeBuf));
    gfx_->setTextSize(2);
    gfx_->setTextColor(DIM_GRAY_TEXT, MODAL_BG_COLOR);
    int tw = strlen(timeBuf) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - tw) / 2, 272);
    gfx_->print(timeBuf);

    // Manual Dismiss OK Button
    drawOkButton(false);

    gfx_->setTextColor(WHITE, BLACK);
}

void QsoCard::drawOkButton(bool highlighted) {
    if (!gfx_) return;
    uint16_t bg = highlighted ? COLOR_ROVER_QUALIFIED_GREEN : BUTTON_OK_IDLE_BG;
    uint16_t border = COLOR_ROVER_QUALIFIED_GREEN;

    gfx_->fillRoundRect(QSO_BTN_OK_X, QSO_BTN_OK_Y, QSO_BTN_OK_W, QSO_BTN_OK_H, 8, bg);
    gfx_->drawRoundRect(QSO_BTN_OK_X, QSO_BTN_OK_Y, QSO_BTN_OK_W, QSO_BTN_OK_H, 8, border);

    const char *lbl = "OK (Auto 10s)";
    int lw = strlen(lbl) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(WHITE, bg);
    gfx_->setCursor(QSO_BTN_OK_X + (QSO_BTN_OK_W - lw) / 2, QSO_BTN_OK_Y + (QSO_BTN_OK_H - CHAR_H(2)) / 2);
    gfx_->print(lbl);
}
#else
void QsoCard::drawModal(const QsoEntry &, size_t) {}
void QsoCard::drawOkButton(bool) {}
#endif
