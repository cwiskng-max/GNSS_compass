#pragma once
#include <cstdint>

// High-precision geographic coordinate (Source of physical ground truth)
struct GeoPoint {
    double latitude = 0.0;   // Degrees: -90.0 to +90.0 (North positive)
    double longitude = 0.0;  // Degrees: -180.0 to +180.0 (East positive)
    bool isValid = false;

    GeoPoint() : latitude(0.0), longitude(0.0), isValid(false) {}
    GeoPoint(double lat, double lon, bool valid = true)
        : latitude(lat), longitude(lon), isValid(valid) {}
};

// Bearing Quality indicators
enum BearingQuality : uint8_t {
    BEARING_NONE = 0,    // Fresh start, never acquired -> WHITE (---.-)
    BEARING_FIXED = 1,   // High precision (RTK Fixed) -> WHITE
    BEARING_FLOAT = 2,   // Reduced quality (RTK Float) -> RICH_YELLOW
    BEARING_LOST = 3     // Signal lost / stale timeout -> RED
};

// GPS Link Diagnostic Health State
enum GpsLinkVerdict : uint8_t {
    GPS_LINK_DISCONNECTED = 0,   // No bytes received (>3s) or total bytes == 0
    GPS_LINK_NOISY_CORRUPT = 1,  // Raw bytes received, but no valid NMEA/KSXT sentences
    GPS_LINK_ACQUIRING = 2,      // Valid sentences streaming, waiting for satellites
    GPS_LINK_POSITION_ONLY = 3,  // Position valid, waiting for dual-antenna heading
    GPS_LINK_HEADING_FLOAT = 4,  // Dual-antenna heading active in Float mode (Yellow)
    GPS_LINK_OK_FIXED = 5        // Dual-antenna heading active in Fixed mode (White)
};

