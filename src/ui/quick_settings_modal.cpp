#include "quick_settings_modal.h"
#include <cstdio>
#include <cstring>

void QuickSettingsModal::formatBrightnessText(uint8_t percent, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize == 0) return;
    if (percent > 100) percent = 100;
    snprintf(outBuf, bufSize, "%3u%%", static_cast<unsigned int>(percent));
}

int16_t QuickSettingsModal::getSliderFillWidth(uint8_t percent, int16_t maxWidth) {
    if (maxWidth <= 0) return 0;
    if (percent > 100) percent = 100;
    return static_cast<int16_t>((static_cast<int32_t>(percent) * maxWidth + 50) / 100);
}

bool QuickSettingsModal::isOkButtonTouched(int16_t x, int16_t y) {
    return (x >= QUICK_SETTINGS_OK_X && x <= (QUICK_SETTINGS_OK_X + QUICK_SETTINGS_OK_W) &&
            y >= QUICK_SETTINGS_OK_Y && y <= (QUICK_SETTINGS_OK_Y + QUICK_SETTINGS_OK_H));
}

int QuickSettingsModal::getBrightnessFromTouch(int16_t x, int16_t y) {
    if (y < (QUICK_SETTINGS_BAR_Y - 14) || y > (QUICK_SETTINGS_BAR_Y + QUICK_SETTINGS_BAR_H + 14)) {
        return -1;
    }
    if (x < (QUICK_SETTINGS_BAR_X - 10) || x > (QUICK_SETTINGS_BAR_X + QUICK_SETTINGS_BAR_W + 10)) {
        return -1;
    }
    int rel = x - QUICK_SETTINGS_BAR_X;
    if (rel < 0) rel = 0;
    if (rel > QUICK_SETTINGS_BAR_W) rel = QUICK_SETTINGS_BAR_W;
    int pct = (rel * 100 + QUICK_SETTINGS_BAR_W / 2) / QUICK_SETTINGS_BAR_W;
    // Snap to 5% increments for clean touch/bezel parity
    pct = ((pct + 2) / 5) * 5;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

#if defined(ARDUINO)
void QuickSettingsModal::drawModal(uint8_t brightnessPercent) {
    if (!gfx_) return;

    gfx_->fillRoundRect(QUICK_SETTINGS_MODAL_X, QUICK_SETTINGS_MODAL_Y,
                        QUICK_SETTINGS_MODAL_W, QUICK_SETTINGS_MODAL_H, 12, MODAL_BG_COLOR);
    gfx_->drawRoundRect(QUICK_SETTINGS_MODAL_X, QUICK_SETTINGS_MODAL_Y,
                        QUICK_SETTINGS_MODAL_W, QUICK_SETTINGS_MODAL_H, 12, BUTTON_CYAN_BORDER);

    const char *title = "QUICK SETTINGS";
    int tw = static_cast<int>(std::strlen(title)) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(CYAN, MODAL_BG_COLOR);
    gfx_->setCursor(QUICK_SETTINGS_MODAL_X + (QUICK_SETTINGS_MODAL_W - tw) / 2, QUICK_SETTINGS_MODAL_Y + 18);
    gfx_->print(title);

    gfx_->drawFastHLine(QUICK_SETTINGS_MODAL_X + 24, QUICK_SETTINGS_MODAL_Y + 44,
                        QUICK_SETTINGS_MODAL_W - 48, DARK_GRAY_BORDER);

    const char *label = "BRIGHTNESS";
    int lw = static_cast<int>(std::strlen(label)) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(WHITE, MODAL_BG_COLOR);
    gfx_->setCursor(QUICK_SETTINGS_MODAL_X + (QUICK_SETTINGS_MODAL_W - lw) / 2, QUICK_SETTINGS_MODAL_Y + 56);
    gfx_->print(label);

    drawBrightnessValue(brightnessPercent);
    drawOkButton(false);
}

void QuickSettingsModal::drawBrightnessValue(uint8_t brightnessPercent) {
    if (!gfx_) return;

    char buf[8];
    formatBrightnessText(brightnessPercent, buf, sizeof(buf));
    int vw = static_cast<int>(std::strlen(buf)) * CHAR_W(3);
    int vx = QUICK_SETTINGS_MODAL_X + (QUICK_SETTINGS_MODAL_W - vw) / 2;
    int vy = QUICK_SETTINGS_MODAL_Y + 78;

    gfx_->fillRect(QUICK_SETTINGS_MODAL_X + 40, vy, QUICK_SETTINGS_MODAL_W - 80, CHAR_H(3), MODAL_BG_COLOR);
    gfx_->setTextSize(3);
    gfx_->setTextColor(RICH_YELLOW, MODAL_BG_COLOR);
    gfx_->setCursor(vx, vy);
    gfx_->print(buf);

    // Slider bar track & fill
    gfx_->fillRoundRect(QUICK_SETTINGS_BAR_X, QUICK_SETTINGS_BAR_Y,
                        QUICK_SETTINGS_BAR_W, QUICK_SETTINGS_BAR_H, 6, MODAL_CARD_BG);
    gfx_->drawRoundRect(QUICK_SETTINGS_BAR_X, QUICK_SETTINGS_BAR_Y,
                        QUICK_SETTINGS_BAR_W, QUICK_SETTINGS_BAR_H, 6, DARK_GRAY_BORDER);

    int16_t maxFillW = QUICK_SETTINGS_BAR_W - 4;
    int16_t fillW = getSliderFillWidth(brightnessPercent, maxFillW);
    if (fillW > 0) {
        gfx_->fillRoundRect(QUICK_SETTINGS_BAR_X + 2, QUICK_SETTINGS_BAR_Y + 2,
                            fillW, QUICK_SETTINGS_BAR_H - 4, 4, BUTTON_CYAN_BORDER);
    }

    gfx_->setTextColor(WHITE, BLACK);
}

void QuickSettingsModal::drawOkButton(bool highlighted) {
    if (!gfx_) return;
    uint16_t bg = highlighted ? BUTTON_CYAN_BORDER : BUTTON_OK_IDLE_BG;
    uint16_t fg = highlighted ? BLACK : WHITE;

    gfx_->fillRoundRect(QUICK_SETTINGS_OK_X, QUICK_SETTINGS_OK_Y,
                        QUICK_SETTINGS_OK_W, QUICK_SETTINGS_OK_H, 8, bg);
    gfx_->drawRoundRect(QUICK_SETTINGS_OK_X, QUICK_SETTINGS_OK_Y,
                        QUICK_SETTINGS_OK_W, QUICK_SETTINGS_OK_H, 8, BUTTON_CYAN_BORDER);

    const char *lbl = "OK";
    int lw = static_cast<int>(std::strlen(lbl)) * CHAR_W(2);
    gfx_->setTextSize(2);
    gfx_->setTextColor(fg, bg);
    gfx_->setCursor(QUICK_SETTINGS_OK_X + (QUICK_SETTINGS_OK_W - lw) / 2, QUICK_SETTINGS_OK_Y + 14);
    gfx_->print(lbl);
    gfx_->setTextColor(WHITE, BLACK);
}
#else
void QuickSettingsModal::drawModal(uint8_t) {}
void QuickSettingsModal::drawBrightnessValue(uint8_t) {}
void QuickSettingsModal::drawOkButton(bool) {}
#endif
