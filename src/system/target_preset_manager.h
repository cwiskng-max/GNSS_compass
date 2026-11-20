#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include "../config.h"
#include "../types.h"
#include "nvs_store.h"

/**
 * @brief Persistent target Maidenhead preset entry stored in NVS.
 */
#pragma pack(push, 1)
struct TargetPreset {
    char     grid[7];                      // "CM87vu" (6 chars + null)
    char     label[MAX_TARGET_LABEL_LEN];  // "Mt Diablo" (up to 16 chars + null, spaces allowed)
    uint16_t useCount;                     // Incremented on [ LOAD / SELECT ] for frequency sorting
    uint32_t lastUsedTime;                 // Unix timestamp or sequence counter (tie-breaker)

    TargetPreset() {
        std::memset(this, 0, sizeof(*this));
    }

    TargetPreset(const char *g, const char *lbl, uint16_t uses = 0, uint32_t ts = 0) {
        std::memset(this, 0, sizeof(*this));
        setGrid(g);
        setLabel(lbl);
        useCount = uses;
        lastUsedTime = ts;
    }

    void setGrid(const char *g) { copyBoundedString(grid, sizeof(grid), g); }
    void setLabel(const char *lbl) { copyBoundedString(label, sizeof(label), lbl); }

    bool operator==(const TargetPreset &other) const {
        return std::strncmp(grid, other.grid, sizeof(grid)) == 0 &&
               std::strncmp(label, other.label, sizeof(label)) == 0 &&
               useCount == other.useCount &&
               lastUsedTime == other.lastUsedTime;
    }

    bool operator!=(const TargetPreset &other) const {
        return !(*this == other);
    }
};
#pragma pack(pop)

/**
 * @brief Manages persistent target Maidenhead grid presets in ESP32 NVS.
 *
 * Uses NvsConfig and NvsListStore<TargetPreset> for wear-leveled indexed storage,
 * configurable capacity (10-500, default 100), frequency-first sorting
 * (useCount descending, lastUsedTime descending tie-breaker), regional summit
 * auto-labeling, and least-used/oldest eviction when full.
 */
class TargetPresetManager {
public:
    TargetPresetManager();

    // Initializes manager and loads stored presets and capacity from NVS
    void begin();

    // CRUD & usage tracking
    bool addOrUpdatePreset(const char *grid, const char *label = nullptr, uint32_t nowSec = 0);
    bool recordUse(size_t index, uint32_t nowSec = 0);
    bool recordUseByAddedOrder(size_t addedIndex, uint32_t nowSec = 0);
    bool removePreset(size_t index);
    bool removePresetByAddedOrder(size_t addedIndex);
    bool removePresetByGrid(const char *grid);
    void clearAll();

    // Lookup & accessors (default ordering is usage-sorted for GUI; *ByAddedOrder is static for CLI)
    int findIndexByGrid(const char *grid) const;
    int findAddedIndexByGrid(const char *grid) const;
    const TargetPreset* findByGrid(const char *grid) const;
    const TargetPreset* getPreset(size_t index) const;
    const TargetPreset* getPresetByAddedOrder(size_t addedIndex) const;
    const std::vector<TargetPreset>& getPresets() const { return presets_; }
    size_t getCount() const { return presets_.size(); }
    bool isEmpty() const { return presets_.empty(); }

    // Capacity configuration (persisted under "target_cfg", key "max_targets")
    size_t getMaxTargets() const { return maxTargets_; }
    void setMaxTargets(size_t maxTargets, bool persist = true);

    // Direct NVS storage operations
    void loadFromNvs();
    void saveToNvs();

    // Validation, normalization, and regional summit lookup
    static size_t clampMaxTargets(size_t val);
    static bool isValidGrid6(const char *grid);
    static bool normalizeGrid(const char *in, char out[7]);
    static const char* lookupRegionalSummit(const char *grid);

#if !defined(ARDUINO)
    // Host test mock helpers
    static void resetHostStorage();
    size_t getHostStorageCount() const;
    size_t getHostStorageMaxTargets() const;
#endif

private:
    void sortPresets();
    int findGuiIndexByAddedIndex(size_t addedIndex) const;

    std::vector<TargetPreset> presets_;
    std::vector<size_t> nvsSlots_;
    size_t maxTargets_;
    NvsConfig cfgStore_;
    NvsListStore<TargetPreset> listStore_;
};
