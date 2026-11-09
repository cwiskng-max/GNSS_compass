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
    if (rptr) *rptr = nullptr;
    if (!buf || !lat || !lon) return;

    size_t len = strlen(buf);
    if (len < 10) return;

    char *ptr = buf;
    int neg = 0;

    if (*ptr == '-') { neg = 1; ptr++; }
    if (!isdigit((unsigned char)ptr[0]) || !isdigit((unsigned char)ptr[1])) return;
    *lat = (ptr[0] - '0') * 10.0 + (ptr[1] - '0');
    char *endPtr = nullptr;
    *lat += strtod(&ptr[2], &endPtr) / 60.0;
    if (!endPtr || *endPtr != ',' || endPtr[1] == '\0' || endPtr[2] != ',') {
        *lat = 0.0;
        return;
    }
    if (endPtr[1] == 'S' || endPtr[1] == 's') neg = 1;
    if (neg) { neg = 0; *lat = -*lat; }

    ptr = &endPtr[3]; // start of longitude
    if (*ptr == '-') { neg = 1; ptr++; }
    if (!isdigit((unsigned char)ptr[0]) || !isdigit((unsigned char)ptr[1]) || !isdigit((unsigned char)ptr[2])) {
        *lat = 0.0;
        return;
    }
    *lon = (ptr[0] - '0') * 100.0 + (ptr[1] - '0') * 10.0 + (ptr[2] - '0');
    *lon += strtod(&ptr[3], &endPtr) / 60.0;
    if (!endPtr || *endPtr != ',' || endPtr[1] == '\0') {
        *lat = 0.0;
        *lon = 0.0;
        return;
    }
    if (endPtr[1] == 'W' || endPtr[1] == 'w') neg = 1;
    if (neg) { neg = 0; *lon = -*lon; }

    if (rptr) {
        if (endPtr[2] == ',') {
            *rptr = &endPtr[3];
        } else {
            *rptr = &endPtr[2];
        }
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
    if (q_field == nullptr || *q_field == '*' || *q_field == '\0' || *q_field == '\r' || *q_field == '\n') {
        // Backward compatibility: If KSXT sentence does not include the RTK quality field
        // (e.g. legacy/compact KSXT configurations), default to FIXED (consistent with $GNHDT).
        *heading_out = strtof(h_field, nullptr);
        *quality_out = BEARING_FIXED;
        return true;
    }

    int qual = 0;
    if (*q_field >= '0' && *q_field <= '9') {
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

uint32_t utcCalendarToEpoch(int year, int month, int day, int hour, int min, int sec) {
    if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 60) {
        return 0;
    }
    static const uint16_t kDaysBeforeMonth[13] = {
        0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };
    auto isLeap = [](int y) {
        return (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
    };
    int y = year - 1970;
    int leapDays = (year - 1969) / 4 - (year - 1901) / 100 + (year - 1601) / 400;
    uint32_t days = static_cast<uint32_t>(y * 365 + leapDays + kDaysBeforeMonth[month] + (day - 1));
    if (month > 2 && isLeap(year)) {
        days += 1;
    }
    return days * 86400UL + static_cast<uint32_t>(hour * 3600 + min * 60 + sec);
}

bool parseKsxtDateTime(const char *sentence, int &yearOut, int &monthOut, int &dayOut, int &hoursOut, int &minutesOut, int &secondsOut) {
    if (!sentence) return false;

    const char *utc_field = getNthField(sentence, 1);
    if (utc_field == nullptr || *utc_field == ',' || *utc_field == '*' || *utc_field == '\0') {
        return false;
    }

    int len = 0;
    while (isdigit((unsigned char)utc_field[len])) {
        len++;
    }

    if (len >= 12) {
        // Format: yyyymmddhhmmss (14) or yyyymmddhhmm (12)
        yearOut = (utc_field[0] - '0') * 1000 + (utc_field[1] - '0') * 100 + (utc_field[2] - '0') * 10 + (utc_field[3] - '0');
        monthOut = (utc_field[4] - '0') * 10 + (utc_field[5] - '0');
        dayOut = (utc_field[6] - '0') * 10 + (utc_field[7] - '0');
        hoursOut = (utc_field[8] - '0') * 10 + (utc_field[9] - '0');
        minutesOut = (utc_field[10] - '0') * 10 + (utc_field[11] - '0');
        secondsOut = (len >= 14) ? ((utc_field[12] - '0') * 10 + (utc_field[13] - '0')) : 0;

        return (yearOut >= 1970 && monthOut >= 1 && monthOut <= 12 && dayOut >= 1 && dayOut <= 31 &&
                hoursOut >= 0 && hoursOut < 24 && minutesOut >= 0 && minutesOut < 60 && secondsOut >= 0 && secondsOut < 60);
    }
    return false;
}

bool parseKsxtEpochTime(const char *sentence, uint32_t &epochOut) {
    int y = 0, m = 0, d = 0, h = 0, min = 0, s = 0;
    if (!parseKsxtDateTime(sentence, y, m, d, h, min, s)) {
        return false;
    }
    epochOut = utcCalendarToEpoch(y, m, d, h, min, s);
    return epochOut > 0;
}

bool parseKSXTLocation(const char *sentence, double *lat_out, double *lon_out) {
    if (!sentence || !lat_out || !lon_out) return false;

    const char *q_field = getNthField(sentence, 10);
    // If field 10 exists, require valid fix status (> '0')
    if (q_field != nullptr && *q_field != '*' && (*q_field <= '0' || *q_field > '9')) {
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
