#include "simulator.h"
#include "../geo/geo_math.h"
#include <cstdio>
#include <cmath>
#include <cstring>

GpsSimulator::GpsSimulator()
    : active_(false), packetCount_(0), lastInjectionMillis_(0) {}

void GpsSimulator::recordInjection(size_t /* byteCount */, uint32_t nowMs) {
    active_ = true;
    packetCount_++;
    lastInjectionMillis_ = nowMs;
}

void GpsSimulator::reset() {
    active_ = false;
    packetCount_ = 0;
    lastInjectionMillis_ = 0;
}

void GpsSimulator::injectSentence(const char *sentence, GpsSentenceConsumer consumer, uint32_t nowMs) {
    if (!sentence) return;
    active_ = true;
    packetCount_++;
    lastInjectionMillis_ = nowMs;
    if (consumer) {
        consumer(sentence);
    }
}

uint8_t GpsSimulator::computeChecksum(const char *sentence) {
    if (!sentence) return 0;
    const char *p = sentence;
    if (*p == '$') p++;
    uint8_t csum = 0;
    while (*p && *p != '*' && *p != '\r' && *p != '\n') {
        csum ^= (uint8_t)(*p++);
    }
    return csum;
}

bool GpsSimulator::generateKsxt(char *outBuffer, size_t bufferSize,
                                const char *utcTime, double lat, double lon,
                                double headingDeg, BearingQuality quality,
                                double altitude, double pitch, double roll,
                                int sats) {
    if (!outBuffer || bufferSize < 64) return false;

    int posQuality = 1;
    int hdgQuality = 3; // Fixed
    if (quality == BEARING_FLOAT) {
        hdgQuality = 2; // Float
    } else if (quality == BEARING_LOST || quality == BEARING_NONE) {
        hdgQuality = 0; // Lost / Invalid
    }

    const char *timeStr = (utcTime && *utcTime) ? utcTime : "20261106180000.00";
    char payload[192];
    int len = snprintf(payload, sizeof(payload),
                       "$KSXT,%s,%.8f,%.8f,%.2f,%.2f,%.2f,%.2f,0.022,,%d,%d,%d,28",
                       timeStr, lon, lat, altitude, headingDeg, pitch, roll,
                       posQuality, hdgQuality, sats);
    if (len < 0 || (size_t)len >= sizeof(payload)) return false;

    uint8_t csum = computeChecksum(payload);
    int fullLen = snprintf(outBuffer, bufferSize, "%s*%02X\r\n", payload, csum);
    return (fullLen > 0 && (size_t)fullLen < bufferSize);
}

bool GpsSimulator::generateGll(char *outBuffer, size_t bufferSize,
                              const GeoPoint &point, int hours, int minutes,
                              bool valid) {
    if (!outBuffer || bufferSize < 64) return false;

    double absLat = std::abs(point.latitude);
    int latDeg = (int)absLat;
    double latMin = (absLat - latDeg) * 60.0;

    double absLon = std::abs(point.longitude);
    int lonDeg = (int)absLon;
    double lonMin = (absLon - lonDeg) * 60.0;

    char latStr[24];
    snprintf(latStr, sizeof(latStr), "%s%02d%07.4f",
             (point.latitude < 0.0 ? "-" : ""), latDeg, latMin);

    char lonStr[24];
    snprintf(lonStr, sizeof(lonStr), "%s%03d%07.4f",
             (point.longitude < 0.0 ? "-" : ""), lonDeg, lonMin);

    char statusChar = valid ? 'A' : 'V';
    char payload[128];
    int len = snprintf(payload, sizeof(payload),
                       "$GNGLL,%s,%c,%s,%c,%02d%02d00.00,%c,%c",
                       latStr, (point.latitude >= 0.0 ? 'N' : 'S'),
                       lonStr, (point.longitude >= 0.0 ? 'E' : 'W'),
                       hours, minutes, statusChar, statusChar);
    if (len < 0 || (size_t)len >= sizeof(payload)) return false;

    uint8_t csum = computeChecksum(payload);
    int fullLen = snprintf(outBuffer, bufferSize, "%s*%02X\r\n", payload, csum);
    return (fullLen > 0 && (size_t)fullLen < bufferSize);
}

bool GpsSimulator::generateHdt(char *outBuffer, size_t bufferSize,
                              double headingDeg) {
    if (!outBuffer || bufferSize < 32) return false;

    char payload[64];
    int len = snprintf(payload, sizeof(payload), "$GNHDT,%.1f,T", headingDeg);
    if (len < 0 || (size_t)len >= sizeof(payload)) return false;

    uint8_t csum = computeChecksum(payload);
    int fullLen = snprintf(outBuffer, bufferSize, "%s*%02X\r\n", payload, csum);
    return (fullLen > 0 && (size_t)fullLen < bufferSize);
}

GeoPoint GpsSimulator::calculateRoverDisplacement(const GeoPoint &origin,
                                                  double distKm,
                                                  double bearingDeg) {
    double radBearing = toRad(bearingDeg);
    double latOffset = (distKm * 1000.0 * std::cos(radBearing)) / 111132.954;
    double lonOffset = (distKm * 1000.0 * std::sin(radBearing)) / (111132.954 * std::cos(toRad(origin.latitude)));
    return GeoPoint(origin.latitude + latOffset, origin.longitude + lonOffset, true);
}

