#pragma once

#include <cstdint>
#include <cstring>
#include "../config.h"

#if defined(ARDUINO)
#include <Arduino_GFX_Library.h>

namespace ui_widgets {

inline void drawHoldProgressButton(Arduino_GFX *gfx,
                                   int16_t x, int16_t y, int16_t w, int16_t h,
                                   int16_t textY,
                                   uint16_t bg, uint16_t border, uint16_t fill, uint16_t fg,
                                   const char *label,
                                   bool isHolding, float progress, bool enabled = true) {
    if (!gfx || !label) return;

    gfx->fillRoundRect(x, y, w, h, 8, bg);
    gfx->drawRoundRect(x, y, w, h, 8, border);

    if (enabled && isHolding && progress > 0.0f) {
        if (progress > 1.0f) progress = 1.0f;
        int16_t maxFillW = w - 4;
        int16_t fillW = static_cast<int16_t>(static_cast<float>(maxFillW) * progress);
        if (fillW > maxFillW) fillW = maxFillW;
        if (fillW > 0) {
            gfx->fillRoundRect(x + 2, y + 2, fillW, h - 4, 6, fill);
        }
    }

    int lw = static_cast<int>(std::strlen(label)) * CHAR_W(2);
    uint16_t textBg = (enabled && isHolding && progress > 0.5f) ? fill : bg;
    gfx->setTextSize(2);
    gfx->setTextColor(fg, textBg);
    gfx->setCursor(x + (w - lw) / 2, textY);
    gfx->print(label);
    gfx->setTextColor(WHITE, BLACK);
}

} // namespace ui_widgets
#endif
