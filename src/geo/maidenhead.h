#pragma once
#include <cstdint>
#include "../types.h"

// Convert 4-, 6-, or 8-character Maidenhead grid string to center geographic coordinates
bool gridToGeoPoint(const char *grid, GeoPoint &outPoint);
bool gridToGeoPoint(const char *grid, double *lat, double *lon);

// Convert geographic coordinates to 8-character Maidenhead grid (or given length)
bool geoPointToGrid(const GeoPoint &point, char *outGrid, uint8_t length = 8);
bool geoPointToGrid(double lat, double lon, char *outGrid, uint8_t length = 8);
