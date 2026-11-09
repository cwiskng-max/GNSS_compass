#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include <cmath>
#include <cctype>
#include "../config.h"

/**
 * @brief Represents a persistent contest operating site / transit stop.
 */
#pragma pack(push, 1)
struct RoverStop {
    uint32_t timestamp;        // GPS epoch timestamp (seconds)
    double   latitude;         // Decimal latitude of operating site
    double   longitude;        // Decimal longitude of operating site
    char     grid[9];          // 8-character Maidenhead locator (e.g. "CM87wa22"), null-terminated
    float    displacement_km;  // Verified straight-line km from immediate prior stop
    bool     qualified_16km;   // Whether the move met the 16 km requirement

    RoverStop() {
        std::memset(this, 0, sizeof(*this));
    }

    RoverStop(uint32_t ts, double lat, double lon, const char *g, float disp, bool qual) {
        std::memset(this, 0, sizeof(*this));
        timestamp = ts;
        latitude = lat;
        longitude = lon;
        displacement_km = disp;
        qualified_16km = qual;
        setGrid(g);
    }

    void setGrid(const char *g) {
        if (g) {
            size_t len = std::strlen(g);
            if (len > sizeof(grid) - 1) len = sizeof(grid) - 1;
            std::memcpy(grid, g, len);
            grid[len] = '\0';
        } else {
            std::memset(grid, 0, sizeof(grid));
        }
    }

    bool operator==(const RoverStop &other) const {
        return timestamp == other.timestamp &&
               std::abs(latitude - other.latitude) < 1e-7 &&
               std::abs(longitude - other.longitude) < 1e-7 &&
               std::strncmp(grid, other.grid, sizeof(grid)) == 0 &&
               std::abs(displacement_km - other.displacement_km) < 1e-4f &&
               qualified_16km == other.qualified_16km;
    }

    bool operator!=(const RoverStop &other) const {
        return !(*this == other);
    }
};
#pragma pack(pop)

/**
 * @brief Manages the persistent multi-stop rover contest location history stack in NVS.
 *
 * Supports sequential undo (popping last stop and restoring prior origin), bounds management
 * (default 100 stops, clamped 10-500), capacity rollover (oldest unpinned entries drop on overflow),
 * and duplicate 6-character subsquare lookup to avoid duplicate QSO penalties.
 */
class RoverLogManager {
public:
    RoverLogManager();

    // Initializes manager and loads stored history and capacity from NVS
    void begin();

    // Stack operations
    bool appendStop(const RoverStop &stop);
    bool popLastStop();
    void clearHistory();

    // Stack inspection & accessors
    size_t getStopCount() const { return stops_.size(); }
    bool isEmpty() const { return stops_.empty(); }

    const RoverStop* getStop(size_t index) const;
    bool getStop(size_t index, RoverStop &out) const;

    const RoverStop* getLastStop() const;
    bool getLastStop(RoverStop &out) const;

    const RoverStop* getPriorStop() const;
    bool getPriorStop(RoverStop &out) const;

    const std::vector<RoverStop>& getStops() const { return stops_; }

    // Duplicate grid search: checks 6-char subsquare against past stops
    bool hasVisitedGrid(const char *grid) const;

    // Capacity configuration (persisted under "rover_cfg", key "max_stops")
    size_t getMaxStops() const { return maxStops_; }
    void setMaxStops(size_t maxStops, bool persist = true);

    // Direct NVS storage operations
    void loadFromNvs();
    void saveToNvs();

    static size_t clampMaxStops(size_t val);

#if !defined(ARDUINO)
    // Host test mock helpers
    static void resetHostStorage();
    static size_t getHostStorageCount();
    static size_t getHostStorageMaxStops();
    static bool isHostStorageMaxStopsExplicit();
#endif

private:
    std::vector<RoverStop> stops_;
    size_t maxStops_;

#if !defined(ARDUINO)
    static std::vector<RoverStop> s_hostNvsStops;
    static size_t s_hostNvsMaxStops;
    static bool s_hostNvsMaxStopsStored;
#endif
};

