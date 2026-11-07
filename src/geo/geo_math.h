#pragma once
#include <cmath>
#include "../types.h"
#include "bearing_filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef BSZ
#define BSZ 5
#endif

// Angle conversion utilities
double toRad(double degree);
double toDeg(double rad);
inline double toRadians(double degrees) { return toRad(degrees); }
inline double toDegrees(double radians) { return toDeg(radians); }

// WGS84 Geodesic Distance (meters) and Initial Bearing (degrees)
void calculateWgs84(double lat1, double lon1, double lat2, double lon2, double &distance, double &bearing);
void calculateWgs84(const GeoPoint &from, const GeoPoint &to, double &distanceMeters, double &bearingDeg);

// Reverse Bearing from target back to station
double calculateReverseBearing(const GeoPoint &from, const GeoPoint &to);

// ARRL Contest Distance (Spherical Earth R = 6371.0 km, 1.0 km minimum floor)
double calculateArrlContestDistance(const char *gridA, const char *gridB);

// Rover transit alert threshold check (< 16 km rule)
bool isRoverTransitInsideAlert(const GeoPoint &currentPos, const GeoPoint &originPos, double thresholdKm = 16.0);

// Compass Heading & Calibration Offset Math
float normalizeOffset(float deltaDeg);
double applyBearingOffset(double uncalibratedBearing, float offsetDeg);
double calcCurrentBearing(float sinSum, float cosSum, float calOffset = 0.0f);
