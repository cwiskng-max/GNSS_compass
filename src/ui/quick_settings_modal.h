#pragma once

#include <cstdint>
#include <cstddef>
#include "../config.h"

#if defined(ARDUINO)
#include <Arduino_GFX_Library.h>
#endif

/**
 * @brief Quick Settings modal dialog (triggered by long-pressing the rotary bezel).
 *
 * Currently hosts Display Brightness (0-100%) with live preview while rotating the bezel
 * or touching the slider bar, committing to NVS on OK/press or auto-dismiss timeout.
 */
class QuickSettingsModal {
public:
#if defined(ARDUINO)
    explicit QuickSettingsModal(Arduino_GFX *gfx = nullptr) : gfx_(gfx) {}
    void setGfx(Arduino_GFX *gfx) { gfx_ = gfx; }
    Arduino_GFX *getGfx() const { return gfx_; }
#else
    QuickSettingsModal() = default;
#endif

    void drawModal(uint8_t brightnessPercent);
    void drawBrightnessValue(uint8_t brightnessPercent);
    void drawOkButton(bool highlighted);

    // Host-testable formatters & geometry helpers
    static void formatBrightnessText(uint8_t percent, char *outBuf, size_t bufSize);
    static int16_t getSliderFillWidth(uint8_t percent, int16_t maxWidth);
    static bool isOkButtonTouched(int16_t x, int16_t y);
    static int getBrightnessFromTouch(int16_t x, int16_t y);

private:
#if defined(ARDUINO)
    Arduino_GFX *gfx_ = nullptr;
#endif
};
