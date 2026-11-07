#pragma once

#include <cstdint>
#include <cstddef>
#include "../types.h"

// Callback signature for delivering injected sentences to downstream parsers
typedef void (*GpsSentenceConsumer)(const char *sentence);

/**
 * @brief Manages GPS simulation state, virtual rover movements, and synthetic sentence generation.
 *
 * Allows injecting virtual NMEA and Unicore KSXT sentences to exercise the parser,
 * bearing filter, geodesy math, and rover proximity alerts without live hardware.
 */
class GpsSimulator {
public:
    GpsSimulator();

    // Simulation state accessors
    bool isActive() const { return active_; }
    void setActive(bool active) { active_ = active; }

    uint32_t getPacketCount() const { return packetCount_; }
    uint32_t getLastInjectionMillis() const { return lastInjectionMillis_; }

    void recordInjection(size_t byteCount = 0, uint32_t nowMs = 0);
    void reset();

    // Sentence injection: marks simulation active, updates metrics, and dispatches to consumer
    void injectSentence(const char *sentence, GpsSentenceConsumer consumer = nullptr, uint32_t nowMs = 0);

    // Standard NMEA 0183 8-bit XOR checksum calculator
    static uint8_t computeChecksum(const char *sentence);

    // Synthetic sentence builders
    static bool generateKsxt(char *outBuffer, size_t bufferSize,
                            const char *utcTime, double lat, double lon,
                            double headingDeg, BearingQuality quality,
                            double altitude = 2100.0, double pitch = 0.0, double roll = 0.0,
                            int sats = 28);

    static bool generateGll(char *outBuffer, size_t bufferSize,
                           const GeoPoint &point, int hours = 0, int minutes = 0,
                           bool valid = true);

    static bool generateHdt(char *outBuffer, size_t bufferSize,
                           double headingDeg);

    // Pure 64-bit double geodesic rover displacement calculation
    static GeoPoint calculateRoverDisplacement(const GeoPoint &origin,
                                               double distKm,
                                               double bearingDeg = 45.0);

private:
    bool active_;
    uint32_t packetCount_;
    uint32_t lastInjectionMillis_;
};

