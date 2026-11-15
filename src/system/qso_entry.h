#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>

/**
 * @brief Represents a persistent contest QSO entry.
 *
 * Stored in ESP32-S3 NVS flash as a packed 54-byte record.
 */
#pragma pack(push, 1)
struct QsoEntry {
    uint32_t timestamp;        // GPS UTC epoch timestamp (seconds)
    char     callsign[12];     // Target callsign (e.g. "K6MG/R\0")
    char     band[8];          // Band identifier (e.g. "10 GHz\0")
    uint32_t freq_khz;         // Frequency in kHz (e.g. 10368100), 0 if band-only
    char     my_grid[9];       // Operator Maidenhead grid at QSO time (e.g. "CM87wa\0")
    char     his_grid[9];      // Target Maidenhead grid at QSO time (e.g. "CM88qk\0")
    float    distance_km;      // Geodesic path distance in km
    float    bearing_deg;      // Geodesic true bearing in degrees

    QsoEntry() {
        std::memset(this, 0, sizeof(*this));
    }

    QsoEntry(uint32_t ts, const char *call, const char *bnd, uint32_t freqKhz,
             const char *myGrid, const char *hisGrid, float distKm, float brgDeg) {
        std::memset(this, 0, sizeof(*this));
        timestamp = ts;
        freq_khz = freqKhz;
        distance_km = distKm;
        bearing_deg = brgDeg;
        setCallsign(call);
        setBand(bnd);
        setMyGrid(myGrid);
        setHisGrid(hisGrid);
    }

    void setCallsign(const char *call) {
        if (call) {
            size_t len = std::strlen(call);
            if (len > sizeof(callsign) - 1) len = sizeof(callsign) - 1;
            std::memcpy(callsign, call, len);
            callsign[len] = '\0';
        } else {
            std::memset(callsign, 0, sizeof(callsign));
        }
    }

    void setBand(const char *bnd) {
        if (bnd) {
            size_t len = std::strlen(bnd);
            if (len > sizeof(band) - 1) len = sizeof(band) - 1;
            std::memcpy(band, bnd, len);
            band[len] = '\0';
        } else {
            std::memset(band, 0, sizeof(band));
        }
    }

    void setMyGrid(const char *grid) {
        if (grid) {
            size_t len = std::strlen(grid);
            if (len > sizeof(my_grid) - 1) len = sizeof(my_grid) - 1;
            std::memcpy(my_grid, grid, len);
            my_grid[len] = '\0';
        } else {
            std::memset(my_grid, 0, sizeof(my_grid));
        }
    }

    void setHisGrid(const char *grid) {
        if (grid) {
            size_t len = std::strlen(grid);
            if (len > sizeof(his_grid) - 1) len = sizeof(his_grid) - 1;
            std::memcpy(his_grid, grid, len);
            his_grid[len] = '\0';
        } else {
            std::memset(his_grid, 0, sizeof(his_grid));
        }
    }

    bool operator==(const QsoEntry &other) const {
        return timestamp == other.timestamp &&
               std::strncmp(callsign, other.callsign, sizeof(callsign)) == 0 &&
               std::strncmp(band, other.band, sizeof(band)) == 0 &&
               freq_khz == other.freq_khz &&
               std::strncmp(my_grid, other.my_grid, sizeof(my_grid)) == 0 &&
               std::strncmp(his_grid, other.his_grid, sizeof(his_grid)) == 0 &&
               std::abs(distance_km - other.distance_km) < 1e-3f &&
               std::abs(bearing_deg - other.bearing_deg) < 1e-3f;
    }

    bool operator!=(const QsoEntry &other) const {
        return !(*this == other);
    }
};
#pragma pack(pop)

