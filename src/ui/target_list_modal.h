#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "../config.h"
#include "../types.h"
#include "../system/target_preset_manager.h"

#if defined(ARDUINO)
#include <Arduino_GFX_Library.h>
#endif

/**
 * @brief Modal dialog for browsing, selecting, saving, deleting, and clearing
 *        non-volatile Target Maidenhead Grid presets on the 480x480 round display.
 */
class TargetListModal {
public:
#if defined(ARDUINO)
    explicit TargetListModal(Arduino_GFX *gfx = nullptr) : gfx_(gfx) {}
    void setGfx(Arduino_GFX *gfx) { gfx_ = gfx; }
    Arduino_GFX *getGfx() const { return gfx_; }
#else
    TargetListModal() = default;
#endif

    // Selection & scroll state management
    void resetSelection() { selectedIndex_ = 0; scrollOffset_ = 0; }
    void setSelectedIndex(size_t idx, size_t count) { selectedIndex_ = idx; clampSelection(count); }
    bool moveSelection(int delta, size_t count);
    void clampSelection(size_t count);
    size_t getSelectedIndex() const { return selectedIndex_; }
    size_t getScrollOffset() const { return scrollOffset_; }

    // Modal drawing methods
    void drawModal(const TargetPreset *presets,
                   size_t count,
                   size_t maxTargets,
                   size_t selectedIndex,
                   size_t scrollOffset,
                   const GeoPoint &currentPos,
                   bool hasGps);
    void drawPresetList(const TargetPreset *presets,
                        size_t count,
                        size_t selectedIndex,
                        size_t scrollOffset,
                        const GeoPoint &currentPos,
                        bool hasGps);
    void drawPresetRow(size_t index,
                       const TargetPreset &preset,
                       const GeoPoint &currentPos,
                       bool hasGps,
                       bool isSelected,
                       int yPos);
    void drawScrollTrack(size_t count, size_t scrollOffset);
    void drawLoadButton(bool highlighted, bool enabled);
    void drawDeleteButton(bool highlighted, bool enabled);
    void drawSaveButton(bool highlighted);
    void drawClearButton(bool isHolding, float progress, bool enabled);
    void drawCancelButton(bool highlighted);

    // Static text formatters (host testable)
    static void formatHeader(size_t count, size_t maxTargets, char *outBuf, size_t bufSize);
    static void formatPresetRow(size_t index,
                                const TargetPreset &preset,
                                const GeoPoint &currentPos,
                                bool hasGps,
                                char *outBuf,
                                size_t bufSize);

private:
    size_t selectedIndex_ = 0;
    size_t scrollOffset_ = 0;
#if defined(ARDUINO)
    Arduino_GFX *gfx_ = nullptr;
#endif
};
