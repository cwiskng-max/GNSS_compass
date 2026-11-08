#include "cal_modal.h"
#include <cstring>
#include <cstdio>

void CalModal::formatTargetBearing(double bearing, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    snprintf(outBuf, bufSize, "Target Bearing : %5.1f", bearing);
}

void CalModal::formatRawHeading(double rawHeading, bool hasHeading, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (hasHeading) {
        snprintf(outBuf, bufSize, "Raw GNSS Hdg   : %5.1f", rawHeading);
    } else {
        snprintf(outBuf, bufSize, "Raw GNSS Hdg   : ---.-");
    }
}

void CalModal::formatCurrentOffset(float offset, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    snprintf(outBuf, bufSize, "Current Offset : %+5.1f", offset);
}

void CalModal::formatProposedOffset(float proposedOffset, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    snprintf(outBuf, bufSize, "Proposed Offset: %+5.1f", proposedOffset);
}

#if defined(ARDUINO)
void CalModal::drawModal(double targetBearing, double rawHeading, bool hasHeading, float currentOffset, float proposedOffset) {
    if (!gfx_) return;

    // Semi-modal dark card in center
    gfx_->fillRoundRect(CAL_MODAL_X, CAL_MODAL_Y, CAL_MODAL_W, CAL_MODAL_H, 12, MODAL_BG_COLOR);
    gfx_->drawRoundRect(CAL_MODAL_X, CAL_MODAL_Y, CAL_MODAL_W, CAL_MODAL_H, 12, CYAN);
    gfx_->drawRoundRect(CAL_MODAL_X + 1, CAL_MODAL_Y + 1, CAL_MODAL_W - 2, CAL_MODAL_H - 2, 11, CYAN);

    // Title: "ANTENNA CALIBRATION"
    gfx_->setTextSize(2);
    gfx_->setTextColor(CYAN, MODAL_BG_COLOR);
    const char *title = "ANTENNA CALIBRATION";
    int tw = strlen(title) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - tw) / 2, 88);
    gfx_->print(title);

    // Separator line
    gfx_->drawFastHLine(55, 112, 370, DARK_GRAY_BORDER);

    // Target Bearing
    char buf[36];
    formatTargetBearing(targetBearing, buf, sizeof(buf));
    gfx_->setTextColor(WHITE, MODAL_BG_COLOR);
    gfx_->setCursor(55, 124);
    gfx_->print(buf);

    // Raw GNSS Heading
    drawRawHeading(rawHeading, hasHeading);

    // Current Stored Offset
    formatCurrentOffset(currentOffset, buf, sizeof(buf));
    gfx_->setCursor(55, 172);
    gfx_->print(buf);

    // Proposed Offset
    drawProposedOffsetRow(proposedOffset);

    // Buttons
    drawSnapButton(false, hasHeading);
    drawResetButton(false);
    drawSaveButton(false, 0.0f);
    drawCancelButton(false);

    // Instruction footer
    gfx_->setTextColor(DIM_GRAY_TEXT, MODAL_BG_COLOR);
    const char *hint = "(Turn knob to nudge +-0.1)";
    int htw = strlen(hint) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - htw) / 2, 386);
    gfx_->print(hint);

    gfx_->setTextColor(WHITE, BLACK);
}

void CalModal::drawRawHeading(double rawHeading, bool hasHeading) {
    if (!gfx_) return;
    gfx_->fillRect(55, 146, 370, 22, MODAL_BG_COLOR);
    gfx_->setTextSize(2);
    gfx_->setTextColor(WHITE, MODAL_BG_COLOR);
    gfx_->setCursor(55, 148);
    char buf[36];
    formatRawHeading(rawHeading, hasHeading, buf, sizeof(buf));
    gfx_->print(buf);
    gfx_->setTextColor(WHITE, BLACK);
}

void CalModal::drawProposedOffsetRow(float proposedOffset) {
    if (!gfx_) return;
    gfx_->fillRect(50, 198, 380, 26, MODAL_BG_COLOR);
    gfx_->setTextSize(2);
    gfx_->setTextColor(RICH_YELLOW, MODAL_BG_COLOR);
    char buf[36];
    formatProposedOffset(proposedOffset, buf, sizeof(buf));
    gfx_->setCursor(55, 202);
    gfx_->print(buf);
    gfx_->setTextColor(WHITE, BLACK);
}

void CalModal::drawSnapButton(bool highlighted, bool hasHeading) {
    if (!gfx_) return;

    uint16_t snapBg, snapBorder, snapFg;
    if (!hasHeading) {
        snapBg = BUTTON_MUTED_BG;
        snapBorder = 0x4208;
        snapFg = DIM_GRAY_TEXT;
    } else if (highlighted) {
        snapBg = CYAN;
        snapBorder = WHITE;
        snapFg = BLACK;
    } else {
        snapBg = BUTTON_BLUE_BG;
        snapBorder = BUTTON_CYAN_BORDER;
        snapFg = WHITE;
    }

    gfx_->fillRoundRect(CAL_BTN_SNAP_X, CAL_BTN_SNAP_Y, CAL_BTN_SNAP_W, CAL_BTN_SNAP_H, 8, snapBg);
    gfx_->drawRoundRect(CAL_BTN_SNAP_X, CAL_BTN_SNAP_Y, CAL_BTN_SNAP_W, CAL_BTN_SNAP_H, 8, snapBorder);
    const char *snapText = "SNAP TARGET";
    int stw = strlen(snapText) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(snapFg, snapBg);
    gfx_->setCursor(CAL_BTN_SNAP_X + (CAL_BTN_SNAP_W - stw) / 2, 246);
    gfx_->print(snapText);
    gfx_->setTextColor(WHITE, BLACK);
}

void CalModal::drawResetButton(bool highlighted) {
    if (!gfx_) return;

    uint16_t rstBg = highlighted ? RED : BUTTON_RESET_BG;
    uint16_t rstBorder = highlighted ? WHITE : RED;
    uint16_t rstFg = WHITE;

    gfx_->fillRoundRect(CAL_BTN_RESET_X, CAL_BTN_RESET_Y, CAL_BTN_RESET_W, CAL_BTN_RESET_H, 8, rstBg);
    gfx_->drawRoundRect(CAL_BTN_RESET_X, CAL_BTN_RESET_Y, CAL_BTN_RESET_W, CAL_BTN_RESET_H, 8, rstBorder);
    const char *rstText = "RESET (0.0)";
    int rtw = strlen(rstText) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(rstFg, rstBg);
    gfx_->setCursor(CAL_BTN_RESET_X + (CAL_BTN_RESET_W - rtw) / 2, 246);
    gfx_->print(rstText);
    gfx_->setTextColor(WHITE, BLACK);
}

void CalModal::drawSaveButton(bool isHolding, float progress) {
    if (!gfx_) return;

    uint16_t bg = isHolding ? BUTTON_SAVE_HOLD_BG : BUTTON_SAVE_IDLE_BG;
    gfx_->fillRoundRect(CAL_BTN_SAVE_X, CAL_BTN_SAVE_Y, CAL_BTN_SAVE_W, CAL_BTN_SAVE_H, 8, bg);
    gfx_->drawRoundRect(CAL_BTN_SAVE_X, CAL_BTN_SAVE_Y, CAL_BTN_SAVE_W, CAL_BTN_SAVE_H, 8, GREEN);

    if (isHolding && progress > 0.0f) {
        int fillW = static_cast<int>(366.0f * progress);
        if (fillW > 366) fillW = 366;
        gfx_->fillRoundRect(CAL_BTN_SAVE_X + 2, CAL_BTN_SAVE_Y + 2, fillW, CAL_BTN_SAVE_H - 4, 6, GREEN);
    }

    const char *saveText = isHolding ? "HOLDING TO SAVE..." : "HOLD TO SAVE OFFSET";
    int stw = strlen(saveText) * CHAR_W(2);
    gfx_->setTextColor(WHITE, (isHolding && progress > 0.5f) ? GREEN : bg);
    gfx_->setTextSize(2);
    gfx_->setCursor((SCREEN_WIDTH - stw) / 2, 300);
    gfx_->print(saveText);
    gfx_->setTextColor(WHITE, BLACK);
}

void CalModal::drawCancelButton(bool highlighted) {
    if (!gfx_) return;

    uint16_t canBg = highlighted ? WHITE : BUTTON_CANCEL_BG;
    uint16_t canBorder = WHITE;
    uint16_t canFg = highlighted ? BLACK : WHITE;

    gfx_->fillRoundRect(CAL_BTN_CANCEL_X, CAL_BTN_CANCEL_Y, CAL_BTN_CANCEL_W, CAL_BTN_CANCEL_H, 8, canBg);
    gfx_->drawRoundRect(CAL_BTN_CANCEL_X, CAL_BTN_CANCEL_Y, CAL_BTN_CANCEL_W, CAL_BTN_CANCEL_H, 8, canBorder);
    const char *canText = "CANCEL";
    int ctw = strlen(canText) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(canFg, canBg);
    gfx_->setCursor(CAL_BTN_CANCEL_X + (CAL_BTN_CANCEL_W - ctw) / 2, 352);
    gfx_->print(canText);
    gfx_->setTextColor(WHITE, BLACK);
}
#else
void CalModal::drawModal(double, double, bool, float, float) {}
void CalModal::drawRawHeading(double, bool) {}
void CalModal::drawProposedOffsetRow(float) {}
void CalModal::drawSnapButton(bool, bool) {}
void CalModal::drawResetButton(bool) {}
void CalModal::drawSaveButton(bool, float) {}
void CalModal::drawCancelButton(bool) {}
#endif

