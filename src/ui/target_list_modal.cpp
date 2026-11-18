#include "target_list_modal.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include <cmath>
#include <cstring>

void TargetListModal::formatHeader(size_t count, size_t maxTargets, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    std::snprintf(outBuf, bufSize, "SAVED TARGET GRIDS (%zu/%zu)", count, maxTargets);
}

void TargetListModal::formatPresetRow(size_t index,
                                      const TargetPreset &preset,
                                      const GeoPoint &currentPos,
                                      bool hasGps,
                                      char *outBuf,
                                      size_t bufSize) {
    if (!outBuf || bufSize == 0) return;

    const char *lbl = preset.label;
    size_t idxNum = index + 1;
    int idxChars = (idxNum >= 100) ? 5 : ((idxNum >= 10) ? 4 : 3); // "<N>. "

    GeoPoint tgtPt;
    if (hasGps && gridToGeoPoint(preset.grid, tgtPt)) {
        double distMeters = 0.0;
        double fwdAzimuth = 0.0;
        calculateWgs84(currentPos, tgtPt, distMeters, fwdAzimuth);

        int azDeg = static_cast<int>(std::round(fwdAzimuth)) % 360;
        if (azDeg < 0) azDeg += 360;
        int distKm = static_cast<int>(std::round(distMeters / 1000.0));
        if (distKm < 0) distKm = 0;

        int distChars = (distKm >= 10000) ? 6 : ((distKm >= 1000) ? 5 : 4); // "%3dk"
        int lblW = 27 - (idxChars + 7 + 5 + distChars + 1);
        if (lblW < 3) lblW = 3;
        if (lblW > 7) lblW = 7;

        std::snprintf(outBuf, bufSize, "%zu. %s %-*.*s %03d\xf7 %3dk",
                      idxNum, preset.grid, lblW, lblW, lbl, azDeg, distKm);
    } else {
        int lblW = 27 - (idxChars + 7 + 5 + 4 + 1);
        if (lblW < 3) lblW = 3;
        if (lblW > 7) lblW = 7;

        std::snprintf(outBuf, bufSize, "%zu. %s %-*.*s ---\xf7 ---k",
                      idxNum, preset.grid, lblW, lblW, lbl);
    }
}

#if defined(ARDUINO)
void TargetListModal::drawModal(const TargetPreset *presets,
                                size_t count,
                                size_t maxTargets,
                                size_t selectedIndex,
                                size_t scrollOffset,
                                const GeoPoint &currentPos,
                                bool hasGps) {
    if (!gfx_) return;

    // Outer modal container (centered inside 480x480 round bezel)
    gfx_->fillRoundRect(TARGET_MODAL_X, TARGET_MODAL_Y, TARGET_MODAL_W, TARGET_MODAL_H, 12, MODAL_BG_COLOR);
    gfx_->drawRoundRect(TARGET_MODAL_X, TARGET_MODAL_Y, TARGET_MODAL_W, TARGET_MODAL_H, 12, CYAN);
    gfx_->drawRoundRect(TARGET_MODAL_X + 1, TARGET_MODAL_Y + 1, TARGET_MODAL_W - 2, TARGET_MODAL_H - 2, 11, CYAN);

    // Header bar
    char hdr[40];
    formatHeader(count, maxTargets, hdr, sizeof(hdr));
    gfx_->setTextSize(2);
    gfx_->setTextColor(YELLOW, MODAL_BG_COLOR);
    int hw = static_cast<int>(std::strlen(hdr)) * CHAR_W(2);
    gfx_->setCursor((SCREEN_WIDTH - hw) / 2, TARGET_MODAL_HEADER_Y + 6);
    gfx_->print(hdr);

    // Separator line
    gfx_->drawFastHLine(TARGET_MODAL_HEADER_X, TARGET_MODAL_HEADER_Y + TARGET_MODAL_HEADER_H + 2,
                        TARGET_MODAL_HEADER_W, DARK_GRAY_BORDER);

    // Preset rows & scroll track
    drawPresetList(presets, count, selectedIndex, scrollOffset, currentPos, hasGps);

    // Action buttons
    bool hasItems = (count > 0);
    drawLoadButton(false, hasItems);
    drawDeleteButton(false, hasItems);
    drawSaveButton(false);
    drawClearButton(false, 0.0f, hasItems);
    drawCancelButton(false);

    gfx_->setTextColor(WHITE, BLACK);
}

void TargetListModal::drawPresetList(const TargetPreset *presets,
                                     size_t count,
                                     size_t selectedIndex,
                                     size_t scrollOffset,
                                     const GeoPoint &currentPos,
                                     bool hasGps) {
    if (!gfx_) return;

    // Clear list area
    gfx_->fillRect(TARGET_LIST_X, TARGET_LIST_Y, TARGET_LIST_ROW_W,
                   TARGET_LIST_VISIBLE_ROWS * TARGET_LIST_ROW_PITCH, MODAL_BG_COLOR);

    if (count == 0 || !presets) {
        gfx_->drawRoundRect(TARGET_LIST_X, TARGET_LIST_Y, TARGET_LIST_ROW_W,
                            TARGET_LIST_VISIBLE_ROWS * TARGET_LIST_ROW_PITCH - 6, 6, DARK_GRAY_BORDER);
        const char *emptyMsg = "Empty List";
        int ew = static_cast<int>(std::strlen(emptyMsg)) * CHAR_W(2);
        gfx_->setTextSize(2);
        gfx_->setTextColor(DIM_GRAY_TEXT, MODAL_BG_COLOR);
        gfx_->setCursor(TARGET_LIST_X + (TARGET_LIST_ROW_W - ew) / 2, TARGET_LIST_Y + 64);
        gfx_->print(emptyMsg);
        drawScrollTrack(count, scrollOffset);
        gfx_->setTextColor(WHITE, BLACK);
        return;
    }

    for (size_t row = 0; row < TARGET_LIST_VISIBLE_ROWS; ++row) {
        size_t idx = scrollOffset + row;
        int yPos = TARGET_LIST_Y + static_cast<int>(row) * TARGET_LIST_ROW_PITCH;
        if (idx < count) {
            drawPresetRow(idx, presets[idx], currentPos, hasGps, idx == selectedIndex, yPos);
        }
    }

    drawScrollTrack(count, scrollOffset);
    gfx_->setTextColor(WHITE, BLACK);
}

void TargetListModal::drawPresetRow(size_t index,
                                    const TargetPreset &preset,
                                    const GeoPoint &currentPos,
                                    bool hasGps,
                                    bool isSelected,
                                    int yPos) {
    if (!gfx_) return;

    char lineBuf[40];
    formatPresetRow(index, preset, currentPos, hasGps, lineBuf, sizeof(lineBuf));

    uint16_t bg = isSelected ? 0x03E0 : 0x18E3;
    uint16_t fg = isSelected ? WHITE : 0xDEFB;
    uint16_t border = isSelected ? CYAN : DARK_GRAY_BORDER;

    gfx_->fillRoundRect(TARGET_LIST_X, yPos, TARGET_LIST_ROW_W, TARGET_LIST_ROW_H, 6, bg);
    gfx_->drawRoundRect(TARGET_LIST_X, yPos, TARGET_LIST_ROW_W, TARGET_LIST_ROW_H, 6, border);

    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    gfx_->setCursor(TARGET_LIST_X + 8, yPos + 14);
    gfx_->print(lineBuf);
    gfx_->setTextColor(WHITE, BLACK);
}

void TargetListModal::drawScrollTrack(size_t count, size_t scrollOffset) {
    if (!gfx_) return;

    gfx_->fillRoundRect(TARGET_SCROLL_X, TARGET_SCROLL_Y, TARGET_SCROLL_W, TARGET_SCROLL_H, 3, 0x2104);

    if (count == 0) return;

    int thumbH = TARGET_SCROLL_H;
    int thumbY = TARGET_SCROLL_Y;
    if (count > TARGET_LIST_VISIBLE_ROWS) {
        thumbH = static_cast<int>((TARGET_SCROLL_H * TARGET_LIST_VISIBLE_ROWS) / count);
        if (thumbH < 20) thumbH = 20;
        size_t maxScroll = count - TARGET_LIST_VISIBLE_ROWS;
        size_t clampedOffset = (scrollOffset > maxScroll) ? maxScroll : scrollOffset;
        thumbY = TARGET_SCROLL_Y + static_cast<int>(((TARGET_SCROLL_H - thumbH) * clampedOffset) / maxScroll);
    }

    gfx_->fillRoundRect(TARGET_SCROLL_X, thumbY, TARGET_SCROLL_W, thumbH, 3, WHITE);
}

void TargetListModal::drawLoadButton(bool highlighted, bool enabled) {
    if (!gfx_) return;
    uint16_t bg = !enabled ? BUTTON_MUTED_BG : (highlighted ? BUTTON_BLUE_BG : BUTTON_SAVE_HOLD_BG);
    uint16_t fg = !enabled ? DIM_GRAY_TEXT : (highlighted ? CYAN : WHITE);
    uint16_t border = !enabled ? DARK_GRAY_BORDER : CYAN;

    gfx_->fillRoundRect(TARGET_BTN_LOAD_X, TARGET_BTN_LOAD_Y, TARGET_BTN_LOAD_W, TARGET_BTN_LOAD_H, 8, bg);
    gfx_->drawRoundRect(TARGET_BTN_LOAD_X, TARGET_BTN_LOAD_Y, TARGET_BTN_LOAD_W, TARGET_BTN_LOAD_H, 8, border);

    const char *lbl = "[ USE ]";
    int lw = static_cast<int>(std::strlen(lbl)) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    gfx_->setCursor(TARGET_BTN_LOAD_X + (TARGET_BTN_LOAD_W - lw) / 2, TARGET_BTN_LOAD_Y + 14);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}

void TargetListModal::drawDeleteButton(bool highlighted, bool enabled) {
    if (!gfx_) return;
    uint16_t bg = !enabled ? BUTTON_MUTED_BG : (highlighted ? BUTTON_DISCARD_HOLD_BG : 0x9800);
    uint16_t fg = !enabled ? DIM_GRAY_TEXT : WHITE;
    uint16_t border = !enabled ? DARK_GRAY_BORDER : RED;

    gfx_->fillRoundRect(TARGET_BTN_DEL_X, TARGET_BTN_DEL_Y, TARGET_BTN_DEL_W, TARGET_BTN_DEL_H, 8, bg);
    gfx_->drawRoundRect(TARGET_BTN_DEL_X, TARGET_BTN_DEL_Y, TARGET_BTN_DEL_W, TARGET_BTN_DEL_H, 8, border);

    const char *lbl = "[ DEL ]";
    int lw = static_cast<int>(std::strlen(lbl)) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    gfx_->setCursor(TARGET_BTN_DEL_X + (TARGET_BTN_DEL_W - lw) / 2, TARGET_BTN_DEL_Y + 14);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}

void TargetListModal::drawSaveButton(bool highlighted) {
    if (!gfx_) return;
    uint16_t bg = highlighted ? BUTTON_BLUE_BG : BUTTON_CANCEL_BG;
    uint16_t fg = highlighted ? CYAN : WHITE;

    gfx_->fillRoundRect(TARGET_BTN_SAVE_X, TARGET_BTN_SAVE_Y, TARGET_BTN_SAVE_W, TARGET_BTN_SAVE_H, 8, bg);
    gfx_->drawRoundRect(TARGET_BTN_SAVE_X, TARGET_BTN_SAVE_Y, TARGET_BTN_SAVE_W, TARGET_BTN_SAVE_H, 8, BUTTON_CYAN_BORDER);

    const char *lbl = "[ + SAVE ]";
    int lw = static_cast<int>(std::strlen(lbl)) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    gfx_->setCursor(TARGET_BTN_SAVE_X + (TARGET_BTN_SAVE_W - lw) / 2, TARGET_BTN_SAVE_Y + 14);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}

void TargetListModal::drawClearButton(bool isHolding, float progress, bool enabled) {
    if (!gfx_) return;
    uint16_t bg = !enabled ? BUTTON_MUTED_BG : (isHolding ? BUTTON_DISCARD_HOLD_BG : BUTTON_RESET_BG);
    uint16_t border = !enabled ? DARK_GRAY_BORDER : RED;

    gfx_->fillRoundRect(TARGET_BTN_CLEAR_X, TARGET_BTN_CLEAR_Y, TARGET_BTN_CLEAR_W, TARGET_BTN_CLEAR_H, 8, bg);
    gfx_->drawRoundRect(TARGET_BTN_CLEAR_X, TARGET_BTN_CLEAR_Y, TARGET_BTN_CLEAR_W, TARGET_BTN_CLEAR_H, 8, border);

    if (enabled && isHolding && progress > 0.0f) {
        if (progress > 1.0f) progress = 1.0f;
        int fillW = static_cast<int>((TARGET_BTN_CLEAR_W - 4) * progress);
        if (fillW > TARGET_BTN_CLEAR_W - 4) fillW = TARGET_BTN_CLEAR_W - 4;
        if (fillW > 0) {
            gfx_->fillRoundRect(TARGET_BTN_CLEAR_X + 2, TARGET_BTN_CLEAR_Y + 2, fillW, TARGET_BTN_CLEAR_H - 4, 6, YELLOW);
        }
    }

    const char *lbl = (enabled && isHolding) ? "HOLDING TO CLEAR..." : "[ HOLD TO CLEAR ALL ]";
    int lw = static_cast<int>(std::strlen(lbl)) * CHAR_W(2);
    uint16_t fg = !enabled ? DIM_GRAY_TEXT : ((isHolding && progress > 0.5f) ? BLACK : WHITE);
    uint16_t textBg = (enabled && isHolding && progress > 0.5f) ? YELLOW : bg;
    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, textBg);
    gfx_->setCursor(TARGET_BTN_CLEAR_X + (TARGET_BTN_CLEAR_W - lw) / 2, TARGET_BTN_CLEAR_Y + 14);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}

void TargetListModal::drawCancelButton(bool highlighted) {
    if (!gfx_) return;
    uint16_t bg = highlighted ? BUTTON_BLUE_BG : 0x3186;
    uint16_t fg = highlighted ? CYAN : WHITE;

    gfx_->fillRoundRect(TARGET_BTN_CANCEL_X, TARGET_BTN_CANCEL_Y, TARGET_BTN_CANCEL_W, TARGET_BTN_CANCEL_H, 8, bg);
    gfx_->drawRoundRect(TARGET_BTN_CANCEL_X, TARGET_BTN_CANCEL_Y, TARGET_BTN_CANCEL_W, TARGET_BTN_CANCEL_H, 8, DARK_GRAY_BORDER);

    const char *lbl = "[ CLOSE ]";
    int lw = static_cast<int>(std::strlen(lbl)) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    gfx_->setCursor(TARGET_BTN_CANCEL_X + (TARGET_BTN_CANCEL_W - lw) / 2, TARGET_BTN_CANCEL_Y + 8);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}
#else
void TargetListModal::drawModal(const TargetPreset *, size_t, size_t, size_t, size_t, const GeoPoint &, bool) {}
void TargetListModal::drawPresetList(const TargetPreset *, size_t, size_t, size_t, const GeoPoint &, bool) {}
void TargetListModal::drawPresetRow(size_t, const TargetPreset &, const GeoPoint &, bool, bool, int) {}
void TargetListModal::drawScrollTrack(size_t, size_t) {}
void TargetListModal::drawLoadButton(bool, bool) {}
void TargetListModal::drawDeleteButton(bool, bool) {}
void TargetListModal::drawSaveButton(bool) {}
void TargetListModal::drawClearButton(bool, float, bool) {}
void TargetListModal::drawCancelButton(bool) {}
#endif
