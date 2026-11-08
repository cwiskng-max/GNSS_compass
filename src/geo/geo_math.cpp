#include "geo_math.h"
#include "maidenhead.h"
#include <cstring>
#include <cctype>

double toRad(double degree) {
    return degree * M_PI / 180.0;
}

double toDeg(double rad) {
    return rad * 180.0 / M_PI;
}

void calculateWgs84(double lat1, double lon1, double lat2, double lon2, double &distance, double &bearing) {
    const double R = 6378137.0; // WGS84 semi-major axis in meters

    double dLat = toRad(lat2 - lat1);
    double dLon = toRad(lon2 - lon1);

    double rLat1 = toRad(lat1);
    double rLat2 = toRad(lat2);

    // Haversine Distance Calculation
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(rLat1) * cos(rLat2) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    if (a > 1.0) a = 1.0;
    if (a < 0.0) a = 0.0;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    distance = R * c;

    // Initial Bearing Calculation
    double y = sin(dLon) * cos(rLat2);
    double x = cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon);
    bearing = toDeg(atan2(y, x));

    // Normalize bearing to 0-360 degrees
    if (bearing < 0.0) {
        bearing += 360.0;
    }
}

void calculateWgs84(const GeoPoint &from, const GeoPoint &to, double &distanceMeters, double &bearingDeg) {
    calculateWgs84(from.latitude, from.longitude, to.latitude, to.longitude, distanceMeters, bearingDeg);
}

double calculateReverseBearing(const GeoPoint &from, const GeoPoint &to) {
    double dist = 0.0, revBearing = 0.0;
    calculateWgs84(to.latitude, to.longitude, from.latitude, from.longitude, dist, revBearing);
    return revBearing;
}

double calculateArrlContestDistance(const char *gridA, const char *gridB) {
    if (!gridA || !gridB) return 1.0;
    bool match = true;
    for (int i = 0; i < 6; ++i) {
        if (!gridA[i] || !gridB[i] || std::toupper((unsigned char)gridA[i]) != std::toupper((unsigned char)gridB[i])) {
            match = false;
            break;
        }
    }
    if (match) {
        return 1.0;
    }
    double lat1 = 0.0, lon1 = 0.0, lat2 = 0.0, lon2 = 0.0;
    gridToGeoPoint(gridA, &lat1, &lon1);
    gridToGeoPoint(gridB, &lat2, &lon2);

    const double R = 6371.0; // Mean Earth radius in km (ARRL contest standard)
    double dLat = toRad(lat2 - lat1);
    double dLon = toRad(lon2 - lon1);
    double rLat1 = toRad(lat1);
    double rLat2 = toRad(lat2);

    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(rLat1) * cos(rLat2) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    if (a > 1.0) a = 1.0;
    if (a < 0.0) a = 0.0;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double d = R * c;

    return (d < 1.0) ? 1.0 : d;
}

bool isRoverTransitInsideAlert(const GeoPoint &currentPos, const GeoPoint &originPos, double thresholdKm) {
    double distMeters = 0.0, bearingDeg = 0.0;
    calculateWgs84(originPos, currentPos, distMeters, bearingDeg);
    int distKm = (int)std::floor(distMeters / 1000.0);
    return (distKm < (int)thresholdKm);
}

float normalizeHeading(float headingDeg) {
    float h = fmodf(headingDeg, 360.0f);
    if (h < 0.0f) h += 360.0f;
    return h;
}

float normalizeOffset(float deltaDeg) {
    while (deltaDeg > 180.0f) deltaDeg -= 360.0f;
    while (deltaDeg < -180.0f) deltaDeg += 360.0f;
    return deltaDeg;
}

double applyBearingOffset(double uncalibratedBearing, float offsetDeg) {
    double b = fmod(uncalibratedBearing + (double)offsetDeg, 360.0);
    if (b < 0.0) b += 360.0;
    return b;
}

double calcCurrentBearing(float sinSum, float cosSum, float calOffset) {
    double b = fmod(toDeg(atan2((double)sinSum / (double)BSZ, (double)cosSum / (double)BSZ)) + 270.0 + (double)calOffset, 360.0);
    if (b < 0.0) b += 360.0;
    return b;
}
