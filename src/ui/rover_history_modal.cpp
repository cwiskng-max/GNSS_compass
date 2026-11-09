#include "rover_history_modal.h"
#include <cstring>
#include <cstdio>

void RoverHistoryModal::formatLastStop(size_t stopNumber, const RoverStop *stop, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (!stop || stopNumber == 0) {
        snprintf(outBuf, bufSize, "Last Stop: None");
        return;
    }
    int h = (stop->timestamp / 3600) % 24;
    int m = (stop->timestamp / 60) % 60;
    snprintf(outBuf, bufSize, "Last: #%zu - %s (%02d:%02dZ)", stopNumber, stop->grid, h, m);
}

void RoverHistoryModal::formatDisplacementStatus(const RoverStop *stop, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (!stop) {
        snprintf(outBuf, bufSize, "Displacement: 0.0 km (NONE)");
        return;
    }
    const char *status = "UNDER 16k";
    if (stop->displacement_km == 0.0f && !stop->qualified_16km) {
        status = "ORIGIN";
    } else if (stop->qualified_16km) {
        status = "QUALIFIED";
    }
    snprintf(outBuf, bufSize, "Disp: %5.1f km (%s)", stop->displacement_km, status);
}

void RoverHistoryModal::formatTotalStops(size_t totalStops, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    snprintf(outBuf, bufSize, "Total Logged Stops: %zu", totalStops);
}

#if defined(ARDUINO)
void RoverHistoryModal::drawModal(const RoverStop *lastStop, size_t totalStops, size_t lastStopIndex) {
    if (!gfx_) return;

    // Outer modal card
    gfx_->fillRoundRect(ROVER_MODAL_X, ROVER_MODAL_Y, ROVER_MODAL_W, ROVER_MODAL_H, 12, MODAL_BG_COLOR);
    gfx_->drawRoundRect(ROVER_MODAL_X, ROVER_MODAL_Y, ROVER_MODAL_W, ROVER_MODAL_H, 12, RED);
    gfx_->drawRoundRect(ROVER_MODAL_X + 1, ROVER_MODAL_Y + 1, ROVER_MODAL_W - 2, ROVER_MODAL_H - 2, 11, RED);

    // Title: "ROVER CONTEST HISTORY"
    gfx_->setTextSize(2);
    gfx_->setTextColor(WHITE, MODAL_BG_COLOR);
    const char *title = "ROVER CONTEST HISTORY";
    int tw = strlen(title) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - tw) / 2, 92);
    gfx_->print(title);

    // Separator line
    gfx_->drawFastHLine(55, 118, 370, DARK_GRAY_BORDER);

    // Last Stop Details
    char buf[40];
    formatLastStop(lastStopIndex, lastStop, buf, sizeof(buf));
    gfx_->setTextColor(WHITE, MODAL_BG_COLOR);
    gfx_->setCursor(55, 134);
    gfx_->print(buf);

    formatDisplacementStatus(lastStop, buf, sizeof(buf));
    uint16_t dispColor = (lastStop && lastStop->qualified_16km) ? COLOR_ROVER_QUALIFIED_GREEN :
                         (lastStop && lastStop->displacement_km == 0.0f) ? CYAN : COLOR_ROVER_TRANSIT_RED;
    gfx_->setTextColor(dispColor, MODAL_BG_COLOR);
    gfx_->setCursor(55, 168);
    gfx_->print(buf);

    formatTotalStops(totalStops, buf, sizeof(buf));
    gfx_->setTextColor(RICH_YELLOW, MODAL_BG_COLOR);
    gfx_->setCursor(55, 202);
    gfx_->print(buf);

    // Buttons
    drawDiscardButton(false);
    drawCancelButton(false);

    gfx_->setTextColor(WHITE, BLACK);
}

void RoverHistoryModal::drawDiscardButton(bool highlighted) {
    if (!gfx_) return;
    uint16_t bg = highlighted ? RED : BUTTON_RESET_BG;
    uint16_t fg = WHITE;
    uint16_t border = highlighted ? WHITE : RED;

    gfx_->fillRoundRect(ROVER_BTN_DISCARD_X, ROVER_BTN_DISCARD_Y, ROVER_BTN_DISCARD_W, ROVER_BTN_DISCARD_H, 8, bg);
    gfx_->drawRoundRect(ROVER_BTN_DISCARD_X, ROVER_BTN_DISCARD_Y, ROVER_BTN_DISCARD_W, ROVER_BTN_DISCARD_H, 8, border);

    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    const char *lbl = "[ DISCARD LAST STOP ]";
    int lw = strlen(lbl) * CHAR_W(2);
    gfx_->setCursor(ROVER_BTN_DISCARD_X + (ROVER_BTN_DISCARD_W - lw) / 2, ROVER_BTN_DISCARD_Y + 15);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}

void RoverHistoryModal::drawCancelButton(bool highlighted) {
    if (!gfx_) return;
    uint16_t bg = highlighted ? BUTTON_BLUE_BG : BUTTON_CANCEL_BG;
    uint16_t fg = highlighted ? CYAN : WHITE;

    gfx_->fillRoundRect(ROVER_BTN_CANCEL_X, ROVER_BTN_CANCEL_Y, ROVER_BTN_CANCEL_W, ROVER_BTN_CANCEL_H, 8, bg);
    gfx_->drawRoundRect(ROVER_BTN_CANCEL_X, ROVER_BTN_CANCEL_Y, ROVER_BTN_CANCEL_W, ROVER_BTN_CANCEL_H, 8, DARK_GRAY_BORDER);

    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    const char *lbl = "[ CANCEL ]";
    int lw = strlen(lbl) * CHAR_W(2);
    gfx_->setCursor(ROVER_BTN_CANCEL_X + (ROVER_BTN_CANCEL_W - lw) / 2, ROVER_BTN_CANCEL_Y + 12);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}
#else
void RoverHistoryModal::drawModal(const RoverStop *, size_t, size_t) {}
void RoverHistoryModal::drawDiscardButton(bool) {}
void RoverHistoryModal::drawCancelButton(bool) {}
#endif

