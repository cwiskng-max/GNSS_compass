#include "maidenhead.h"
#include <cctype>
#include <cstring>

bool gridToGeoPoint(const char *grid, double *lat, double *lon) {
    if (!grid || !lat || !lon) return false;
    size_t len = std::strlen(grid);
    if (len < 4) return false;

    *lon = (toupper((unsigned char)grid[0]) - 'A') * 20.0 + (grid[2] - '0') * 2.0 - 180.0;
    *lat = (toupper((unsigned char)grid[1]) - 'A') * 10.0 + (grid[3] - '0') * 1.0 - 90.0;

    if (len >= 6) {
        *lon += (toupper((unsigned char)grid[4]) - 'A') * (5.0 / 60.0);
        *lat += (toupper((unsigned char)grid[5]) - 'A') * (2.5 / 60.0);
        if (len >= 8 && grid[6] >= '0' && grid[6] <= '9' && grid[7] >= '0' && grid[7] <= '9') {
            *lon += (grid[6] - '0') * (0.5 / 60.0) + (0.25 / 60.0);
            *lat += (grid[7] - '0') * (0.25 / 60.0) + (0.125 / 60.0);
        } else {
            *lon += (2.5 / 60.0);
            *lat += (1.25 / 60.0);
        }
    } else {
        *lon += 1.0;
        *lat += 0.5;
    }
    return (*lat >= -90.0 && *lat <= 90.0 && *lon >= -180.0 && *lon <= 180.0);
}

bool gridToGeoPoint(const char *grid, GeoPoint &outPoint) {
    double lat = 0.0, lon = 0.0;
    if (!gridToGeoPoint(grid, &lat, &lon)) {
        outPoint.isValid = false;
        return false;
    }
    outPoint.latitude = lat;
    outPoint.longitude = lon;
    outPoint.isValid = true;
    return true;
}

bool geoPointToGrid(double lat, double lon, char *outGrid, uint8_t length) {
    if (!outGrid || length < 4 || length > 8 || (length % 2 != 0)) return false;

    // Clamp latitude to valid Maidenhead domain [-90.0, +90.0]
    // +90.0 is clamped just below +90 to avoid out-of-bounds field 'S' (max field is 'R')
    if (lat >= 90.0) lat = 89.999999;
    if (lat < -90.0) lat = -90.0;

    // Normalize longitude to [-180.0, +180.0)
    while (lon >= 180.0) lon -= 360.0;
    while (lon < -180.0) lon += 360.0;
    if (lon >= 180.0) lon = 179.999999;
    if (lon < -180.0) lon = -180.0;

    int o1, o2, o3, o4;
    int a1, a2, a3, a4;
    double remainder;

    // longitude
    remainder = lon + 180.0;
    o1 = (int)(remainder / 20.0);
    if (o1 > 17) o1 = 17;
    remainder = remainder - (double)o1 * 20.0;
    o2 = (int)(remainder / 2.0);
    if (o2 > 9) o2 = 9;
    remainder = remainder - 2.0 * (double)o2;
    o3 = (int)(12.0 * remainder);
    if (o3 > 23) o3 = 23;
    remainder = 12.0 * remainder - (double)o3;
    o4 = (int)(10.0 * remainder);
    if (o4 > 9) o4 = 9;

    // latitude
    remainder = lat + 90.0;
    a1 = (int)(remainder / 10.0);
    if (a1 > 17) a1 = 17;
    remainder = remainder - (double)a1 * 10.0;
    a2 = (int)(remainder);
    if (a2 > 9) a2 = 9;
    remainder = remainder - (double)a2;
    a3 = (int)(24.0 * remainder);
    if (a3 > 23) a3 = 23;
    remainder = 24.0 * remainder - (double)a3;
    a4 = (int)(10.0 * remainder);
    if (a4 > 9) a4 = 9;

    char full[9];
    full[0] = (char)o1 + 'A';
    full[1] = (char)a1 + 'A';
    full[2] = (char)o2 + '0';
    full[3] = (char)a2 + '0';
    full[4] = (char)o3 + 'A';
    full[5] = (char)a3 + 'A';
    full[6] = (char)o4 + '0';
    full[7] = (char)a4 + '0';
    full[8] = '\0';

    for (uint8_t i = 0; i < length; ++i) {
        outGrid[i] = full[i];
    }
    outGrid[length] = '\0';
    return true;
}

bool geoPointToGrid(const GeoPoint &point, char *outGrid, uint8_t length) {
    return geoPointToGrid(point.latitude, point.longitude, outGrid, length);
}
