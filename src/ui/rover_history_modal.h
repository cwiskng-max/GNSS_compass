#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "../config.h"
#include "../system/rover_log_manager.h"

#if defined(ARDUINO)
#include <Arduino_GFX_Library.h>
#endif

/**
 * @brief Modal dialog presenting contest location history and 1-touch undo / discard.
 */
class RoverHistoryModal {
public:
#if defined(ARDUINO)
    explicit RoverHistoryModal(Arduino_GFX *gfx = nullptr) : gfx_(gfx) {}
    void setGfx(Arduino_GFX *gfx) { gfx_ = gfx; }
    Arduino_GFX *getGfx() const { return gfx_; }
#else
    RoverHistoryModal() = default;
#endif

    // Modal drawing methods
    void drawModal(const RoverStop *lastStop, size_t totalStops, size_t lastStopIndex);
    void drawDiscardButton(bool highlighted);
    void drawCancelButton(bool highlighted);

    // Static text formatters (host testable)
    static void formatLastStop(size_t stopNumber, const RoverStop *stop, char *outBuf, size_t bufSize);
    static void formatDisplacementStatus(const RoverStop *stop, char *outBuf, size_t bufSize);
    static void formatTotalStops(size_t totalStops, char *outBuf, size_t bufSize);

private:
#if defined(ARDUINO)
    Arduino_GFX *gfx_ = nullptr;
#endif
};

