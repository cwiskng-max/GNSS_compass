#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "../config.h"
#include "../system/qso_entry.h"

#if defined(ARDUINO)
#include <Arduino_GFX_Library.h>
#endif

/**
 * @brief Modal dialog presenting the temporary QSO confirmation card on the 480x480 round screen.
 *
 * Displays logged callsign, band & frequency, Maidenhead grids, great-circle distance & bearing,
 * UTC time, and visual confirmation checkmark/badge with a 10-second non-blocking auto-dismiss timer.
 */
class QsoCard {
public:
#if defined(ARDUINO)
    explicit QsoCard(Arduino_GFX *gfx = nullptr) : gfx_(gfx) {}
    void setGfx(Arduino_GFX *gfx) { gfx_ = gfx; }
    Arduino_GFX *getGfx() const { return gfx_; }
#else
    QsoCard() = default;
#endif

    // Modal drawing methods
    void drawModal(const QsoEntry &qso, size_t qsoIndex = 0);
    void drawOkButton(bool highlighted);

    // Static text formatters (host testable)
    static void formatHeader(size_t qsoNumber, char *outBuf, size_t bufSize);
    static void formatCallsign(const char *callsign, char *outBuf, size_t bufSize);
    static void formatBandFreq(const char *band, uint32_t freqKhz, char *outBuf, size_t bufSize);
    static void formatGrids(const char *myGrid, const char *hisGrid, char *outBuf, size_t bufSize);
    static void formatDistanceBearing(float distanceKm, float bearingDeg, char *outBuf, size_t bufSize);
    static void formatTime(uint32_t timestamp, char *outBuf, size_t bufSize);

    // Static button hit-testing helper
    static bool isOkButtonTouched(int16_t x, int16_t y);

    // Active card state tracking (host testable)
    bool isVisible() const { return visible_; }
    void show(const QsoEntry &qso, size_t qsoIndex, unsigned long nowMs);
    void dismiss();
    bool checkTimeout(unsigned long nowMs);
    unsigned long getOpenedMs() const { return openedMs_; }
    const QsoEntry& getActiveQso() const { return activeQso_; }
    size_t getActiveQsoIndex() const { return activeQsoIndex_; }

private:
    bool visible_ = false;
    unsigned long openedMs_ = 0;
    QsoEntry activeQso_{};
    size_t activeQsoIndex_ = 0;

#if defined(ARDUINO)
    Arduino_GFX *gfx_ = nullptr;
#endif
};

