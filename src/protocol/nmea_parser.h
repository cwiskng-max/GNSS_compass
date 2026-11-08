#pragma once
#include <cstdint>
#include "../types.h"

// Extracts a pointer to the nth comma-separated field in an NMEA sentence
const char* getNthField(const char* str, int n);

// Parses legacy GLL degree/minute format into decimal degrees (full double precision)
void parseLegacyLatLon(char *buf, double *lat, double *lon, char **rptr);
inline void parseLegacyLatLon(char *buf, float *lat, float *lon, char **rptr) {
    double dLat = 0.0, dLon = 0.0;
    parseLegacyLatLon(buf, &dLat, &dLon, rptr);
    if (lat) *lat = (float)dLat;
    if (lon) *lon = (float)dLon;
}

// Unicore KSXT sentence decoders
bool parseKSXT(const char *sentence, float *heading_out, BearingQuality *quality_out);
inline bool parseKsxtHeading(const char *sentence, float &headingOut, BearingQuality &qualityOut) {
    return parseKSXT(sentence, &headingOut, &qualityOut);
}

bool parseKSXTTime(const char *sentence, int *hours_out, int *minutes_out);
inline bool parseKsxtTime(const char *sentence, int &hoursOut, int &minutesOut) {
    return parseKSXTTime(sentence, &hoursOut, &minutesOut);
}

bool parseKSXTLocation(const char *sentence, double *lat_out, double *lon_out);
inline bool parseKSXTLocation(const char *sentence, float *lat_out, float *lon_out) {
    double dLat = 0.0, dLon = 0.0;
    if (!parseKSXTLocation(sentence, &dLat, &dLon)) return false;
    if (lat_out) *lat_out = (float)dLat;
    if (lon_out) *lon_out = (float)dLon;
    return true;
}

bool parseKsxtLocation(const char *sentence, GeoPoint &pointOut);

// Standard NMEA HDT / THS heading decoders
bool parseHDT(const char *sentence, float *heading_out, BearingQuality *quality_out);
bool parseTHS(const char *sentence, float *heading_out, BearingQuality *quality_out);
inline bool parseHdtHeading(const char *sentence, float &headingOut, BearingQuality &qualityOut) {
    return parseHDT(sentence, &headingOut, &qualityOut);
}
inline bool parseThsHeading(const char *sentence, float &headingOut, BearingQuality &qualityOut) {
    return parseTHS(sentence, &headingOut, &qualityOut);
}

// Unified heading parser (KSXT, GNTHS/GPTHS, GNHDT/GPHDT)
bool parseHeadingSentence(const char *sentence, float *heading_out, BearingQuality *quality_out);

// Evaluates stream health and determines GpsLinkVerdict
GpsLinkVerdict evaluateGpsStream(
    unsigned long now,
    unsigned long totalBytes,
    unsigned long lastByteTime,
    unsigned int gllCount,
    unsigned int ksxtCount,
    unsigned int hdtCount,
    bool hasLoc,
    bool hasHdg,
    BearingQuality quality
);
