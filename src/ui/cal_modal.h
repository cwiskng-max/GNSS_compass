#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "../config.h"

#if defined(ARDUINO)
#include <Arduino_GFX_Library.h>
#endif

class CalModal {
public:
#if defined(ARDUINO)
    explicit CalModal(Arduino_GFX *gfx = nullptr) : gfx_(gfx) {}
    void setGfx(Arduino_GFX *gfx) { gfx_ = gfx; }
    Arduino_GFX *getGfx() const { return gfx_; }
#else
    CalModal() = default;
#endif

    // Modal drawing methods
    void drawModal(double targetBearing, double rawHeading, bool hasHeading, float currentOffset, float proposedOffset);
    void drawRawHeading(double rawHeading, bool hasHeading);
    void drawProposedOffsetRow(float proposedOffset);
    void drawSnapButton(bool highlighted, bool hasHeading);
    void drawResetButton(bool highlighted);
    void drawSaveButton(bool isHolding, float progress);
    void drawCancelButton(bool highlighted);

    // Static text formatters (host testable)
    static void formatTargetBearing(double bearing, char *outBuf, size_t bufSize);
    static void formatRawHeading(double rawHeading, bool hasHeading, char *outBuf, size_t bufSize);
    static void formatCurrentOffset(float offset, char *outBuf, size_t bufSize);
    static void formatProposedOffset(float proposedOffset, char *outBuf, size_t bufSize);

private:
#if defined(ARDUINO)
    Arduino_GFX *gfx_ = nullptr;
#endif
};

