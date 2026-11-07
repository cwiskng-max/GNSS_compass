#include "nmea_parser.h"
#include <cstdlib>
#include <cctype>
#include <cstring>

const char* getNthField(const char* str, int n) {
    if (!str) return nullptr;
    int count = 0;
    while (*str) {
        if (count == n) return str;
        if (*str == ',') count++;
        str++;
    }
    if (count == n) return str;
    return nullptr;
}

void parseLegacyLatLon(char *buf, double *lat, double *lon, char **rptr) {
    if (!buf || !lat || !lon) return;

    char *ptr = &buf[0];
    int neg = 0;

    if (ptr[0] == '-') { neg = 1; ptr++; }
    *lat = (ptr[0] - '0') * 10.0 + (ptr[1] - '0');
    *lat += strtod(&ptr[2], &ptr) / 60.0;
    if (neg) { neg = 0; *lat = -*lat; }

    if (ptr[3] == '-') { neg = 1; ptr++; }
    *lon = (ptr[3] - '0') * 100.0 + (ptr[4] - '0') * 10.0 + (ptr[5] - '0');
    *lon += strtod(&ptr[6], &ptr) / 60.0;
    if (neg) { neg = 0; *lon = -*lon; }

    if (rptr) {
        *rptr = &ptr[3];
    }
}

bool parseKSXT(const char *sentence, float *heading_out, BearingQuality *quality_out) {
    if (!sentence || !heading_out || !quality_out) return false;

    const char *h_field = getNthField(sentence, 5);
    if (h_field == nullptr || *h_field == ',' || *h_field == '*' || *h_field == '\0' || *h_field == '\r' || *h_field == '\n') {
        *quality_out = BEARING_LOST;
        return false;
    }

    const char *q_field = getNthField(sentence, 11);
    int qual = 0;
    if (q_field != nullptr && *q_field >= '0' && *q_field <= '9') {
        qual = *q_field - '0';
    }

    if (qual == 3 || qual == 4) {
        *quality_out = BEARING_FIXED;
    } else if (qual == 2 || qual == 5) {
        *quality_out = BEARING_FLOAT;
    } else {
        *quality_out = BEARING_LOST;
        return false;
    }

    *heading_out = strtof(h_field, nullptr);
    return true;
}

bool parseKSXTTime(const char *sentence, int *hours_out, int *minutes_out) {
    if (!sentence || !hours_out || !minutes_out) return false;

    const char *utc_field = getNthField(sentence, 1);
    if (utc_field == nullptr || *utc_field == ',' || *utc_field == '*' || *utc_field == '\0') {
        return false;
    }

    int len = 0;
    while (isdigit((unsigned char)utc_field[len])) {
        len++;
    }

    int h = -1, m = -1;
    if (len >= 12) {
        // Format: yyyymmddhhmmss (14) or yyyymmddhhmm (12)
        h = (utc_field[8] - '0') * 10 + (utc_field[9] - '0');
        m = (utc_field[10] - '0') * 10 + (utc_field[11] - '0');
    } else if (len == 6 || len == 4) {
        // Format: hhmmss (6) or hhmm (4)
        h = (utc_field[0] - '0') * 10 + (utc_field[1] - '0');
        m = (utc_field[2] - '0') * 10 + (utc_field[3] - '0');
    } else {
        return false;
    }

    if (h >= 0 && h < 24 && m >= 0 && m < 60) {
        *hours_out = h;
        *minutes_out = m;
        return true;
    }
    return false;
}

bool parseKSXTLocation(const char *sentence, double *lat_out, double *lon_out) {
    if (!sentence || !lat_out || !lon_out) return false;

    const char *q_field = getNthField(sentence, 10);
    if (q_field == nullptr || *q_field <= '0' || *q_field > '9') {
        return false;
    }

    const char *lon_field = getNthField(sentence, 2);
    const char *lat_field = getNthField(sentence, 3);
    if (lon_field == nullptr || *lon_field == ',' || *lon_field == '*' ||
        lat_field == nullptr || *lat_field == ',' || *lat_field == '*') {
        return false;
    }

    double lon = strtod(lon_field, nullptr);
    double lat = strtod(lat_field, nullptr);
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || (lat == 0.0 && lon == 0.0)) {
        return false;
    }

    *lat_out = lat;
    *lon_out = lon;
    return true;
}

bool parseKsxtLocation(const char *sentence, GeoPoint &pointOut) {
    double lat = 0.0, lon = 0.0;
    if (parseKSXTLocation(sentence, &lat, &lon)) {
        pointOut.latitude = lat;
        pointOut.longitude = lon;
        pointOut.isValid = true;
        return true;
    }
    pointOut.isValid = false;
    return false;
}

bool parseHDT(const char *sentence, float *heading_out, BearingQuality *quality_out) {
    if (!sentence || !heading_out || !quality_out) return false;

    const char *h_field = getNthField(sentence, 1);
    if (h_field == nullptr || *h_field == ',' || *h_field == '*' || *h_field == '\0' || *h_field == '\r' || *h_field == '\n') {
        *quality_out = BEARING_LOST;
        return false;
    }

    *heading_out = strtof(h_field, nullptr);
    *quality_out = BEARING_FIXED;
    return true;
}

bool parseTHS(const char *sentence, float *heading_out, BearingQuality *quality_out) {
    if (!sentence || !heading_out || !quality_out) return false;

    const char *h_field = getNthField(sentence, 1);
    if (h_field == nullptr || *h_field == ',' || *h_field == '*' || *h_field == '\0' || *h_field == '\r' || *h_field == '\n') {
        *quality_out = BEARING_LOST;
        return false;
    }

    const char *m_field = getNthField(sentence, 2);
    if (m_field == nullptr || *m_field == 'V' || *m_field == 'v' || *m_field == ',' || *m_field == '*') {
        *quality_out = BEARING_LOST;
        return false;
    }

    *heading_out = strtof(h_field, nullptr);
    *quality_out = BEARING_FIXED;
    return true;
}

bool parseHeadingSentence(const char *sentence, float *heading_out, BearingQuality *quality_out) {
    if (!sentence || !heading_out || !quality_out) return false;

    if (strncmp(sentence, "$KSXT", 5) == 0) {
        return parseKSXT(sentence, heading_out, quality_out);
    } else if (strncmp(sentence, "$GNTHS", 6) == 0 || strncmp(sentence, "$GPTHS", 6) == 0) {
        return parseTHS(sentence, heading_out, quality_out);
    } else if (strncmp(sentence, "$GNHDT", 6) == 0 || strncmp(sentence, "$GPHDT", 6) == 0) {
        return parseHDT(sentence, heading_out, quality_out);
    }
    return false;
}

GpsLinkVerdict evaluateGpsStream(
    unsigned long now,
    unsigned long totalBytes,
    unsigned long lastByteTime,
    unsigned int gllCount,
    unsigned int ksxtCount,
    unsigned int hdtCount,
    bool hasLoc,
    bool hasHdg,
    BearingQuality quality)
{
    if (totalBytes == 0 || (now - lastByteTime > 3000)) {
        return GPS_LINK_DISCONNECTED;
    }
    if (gllCount == 0 && ksxtCount == 0 && hdtCount == 0) {
        return GPS_LINK_NOISY_CORRUPT;
    }
    if (hasHdg) {
        if (quality == BEARING_FIXED) return GPS_LINK_OK_FIXED;
        if (quality == BEARING_FLOAT) return GPS_LINK_HEADING_FLOAT;
    }
    if (hasLoc) {
        return GPS_LINK_POSITION_ONLY;
    }
    return GPS_LINK_ACQUIRING;
}
