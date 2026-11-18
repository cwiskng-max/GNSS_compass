#include "target_preset_manager.h"
#include <algorithm>
#include <cctype>

namespace {

struct KnownSummit {
    const char *grid;
    const char *name;
};

const KnownSummit REGIONAL_SUMMITS[] = {
    {"CM87vu", "DIABLO"},
    {"CM97be", "UMUNHUM"},
    {"CM97bb", "LOMA P"},
    {"CM88no", "ST HELN"},
    {"CM87uv", "TAMALPAIS"},
    {"DM04ms", "FRAZIER"},
    {"DM13dr", "SADDLEBK"}
};

} // namespace

TargetPresetManager::TargetPresetManager()
    : maxTargets_(DEFAULT_MAX_TARGET_PRESETS),
      cfgStore_(TARGET_CFG_NAMESPACE),
      listStore_(TARGET_LOG_NAMESPACE, TARGET_LOG_KEY_COUNT, 't') {}

void TargetPresetManager::begin() {
    loadFromNvs();
}

size_t TargetPresetManager::clampMaxTargets(size_t val) {
    if (val < MIN_TARGET_PRESETS) return MIN_TARGET_PRESETS;
    if (val > ABS_MAX_TARGET_PRESETS) return ABS_MAX_TARGET_PRESETS;
    return val;
}

bool TargetPresetManager::isValidGrid6(const char *grid) {
    if (!grid || std::strlen(grid) != 6) {
        return false;
    }

    char f0 = static_cast<char>(std::toupper(static_cast<unsigned char>(grid[0])));
    char f1 = static_cast<char>(std::toupper(static_cast<unsigned char>(grid[1])));
    char d2 = grid[2];
    char d3 = grid[3];
    char s4 = static_cast<char>(std::toupper(static_cast<unsigned char>(grid[4])));
    char s5 = static_cast<char>(std::toupper(static_cast<unsigned char>(grid[5])));

    if (f0 < 'A' || f0 > 'R') return false;
    if (f1 < 'A' || f1 > 'R') return false;
    if (d2 < '0' || d2 > '9') return false;
    if (d3 < '0' || d3 > '9') return false;
    if (s4 < 'A' || s4 > 'X') return false;
    if (s5 < 'A' || s5 > 'X') return false;
    return true;
}

bool TargetPresetManager::normalizeGrid(const char *in, char out[7]) {
    if (!out || !isValidGrid6(in)) {
        return false;
    }
    out[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(in[0])));
    out[1] = static_cast<char>(std::toupper(static_cast<unsigned char>(in[1])));
    out[2] = in[2];
    out[3] = in[3];
    out[4] = static_cast<char>(std::tolower(static_cast<unsigned char>(in[4])));
    out[5] = static_cast<char>(std::tolower(static_cast<unsigned char>(in[5])));
    out[6] = '\0';
    return true;
}

const char* TargetPresetManager::lookupRegionalSummit(const char *grid) {
    char norm[7];
    if (!normalizeGrid(grid, norm)) {
        return nullptr;
    }
    for (size_t i = 0; i < sizeof(REGIONAL_SUMMITS) / sizeof(REGIONAL_SUMMITS[0]); ++i) {
        if (std::strcmp(norm, REGIONAL_SUMMITS[i].grid) == 0) {
            return REGIONAL_SUMMITS[i].name;
        }
    }
    return nullptr;
}

int TargetPresetManager::findIndexByGrid(const char *grid) const {
    char norm[7];
    if (!normalizeGrid(grid, norm)) {
        return -1;
    }
    for (size_t i = 0; i < presets_.size(); ++i) {
        if (std::strcmp(presets_[i].grid, norm) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

const TargetPreset* TargetPresetManager::findByGrid(const char *grid) const {
    int idx = findIndexByGrid(grid);
    if (idx < 0) return nullptr;
    return &presets_[static_cast<size_t>(idx)];
}

const TargetPreset* TargetPresetManager::getPreset(size_t index) const {
    if (index >= presets_.size()) return nullptr;
    return &presets_[index];
}

void TargetPresetManager::sortPresets() {
    std::stable_sort(presets_.begin(), presets_.end(),
        [](const TargetPreset &a, const TargetPreset &b) {
            if (a.useCount != b.useCount) {
                return a.useCount > b.useCount;
            }
            return a.lastUsedTime > b.lastUsedTime;
        });
}

bool TargetPresetManager::addOrUpdatePreset(const char *grid, const char *label, uint32_t nowSec) {
    char norm[7];
    if (!normalizeGrid(grid, norm)) {
        return false;
    }

    if (maxTargets_ == 0) {
        maxTargets_ = DEFAULT_MAX_TARGET_PRESETS;
    }

    int existingIdx = findIndexByGrid(norm);
    if (existingIdx >= 0) {
        TargetPreset &existing = presets_[static_cast<size_t>(existingIdx)];
        if (nowSec > 0) {
            existing.lastUsedTime = nowSec;
        }
        if (label && label[0] != '\0') {
            existing.setLabel(label);
        }
        sortPresets();
        saveToNvs();
        return true;
    }

    const char *effectiveLabel = "";
    if (label && label[0] != '\0') {
        effectiveLabel = label;
    } else {
        const char *summit = lookupRegionalSummit(norm);
        if (summit) {
            effectiveLabel = summit;
        }
    }

    // Evict least-used / oldest entry (at the back of the sorted vector) if at capacity
    while (presets_.size() >= maxTargets_) {
        presets_.pop_back();
    }

    TargetPreset preset(norm, effectiveLabel, 0, nowSec);
    presets_.push_back(preset);
    sortPresets();
    saveToNvs();
    return true;
}

bool TargetPresetManager::recordUse(size_t index, uint32_t nowSec) {
    if (index >= presets_.size()) {
        return false;
    }
    if (presets_[index].useCount < UINT16_MAX) {
        presets_[index].useCount++;
    }
    if (nowSec > 0) {
        presets_[index].lastUsedTime = nowSec;
    }
    sortPresets();
    saveToNvs();
    return true;
}

bool TargetPresetManager::removePreset(size_t index) {
    if (index >= presets_.size()) {
        return false;
    }
    presets_.erase(presets_.begin() + static_cast<std::ptrdiff_t>(index));
    saveToNvs();
    return true;
}

bool TargetPresetManager::removePresetByGrid(const char *grid) {
    int idx = findIndexByGrid(grid);
    if (idx < 0) {
        return false;
    }
    return removePreset(static_cast<size_t>(idx));
}

void TargetPresetManager::clearAll() {
    presets_.clear();
    listStore_.clear();
}

void TargetPresetManager::setMaxTargets(size_t maxTargets, bool persist) {
    maxTargets_ = clampMaxTargets(maxTargets);

    bool truncated = false;
    if (presets_.size() > maxTargets_) {
        presets_.resize(maxTargets_);
        truncated = true;
    }

    if (persist) {
        cfgStore_.putUShort(TARGET_CFG_KEY_MAX_TARGETS, static_cast<uint16_t>(maxTargets_));
    }

    if (truncated) {
        saveToNvs();
    }
}

void TargetPresetManager::loadFromNvs() {
    uint16_t storedMax = cfgStore_.getUShort(TARGET_CFG_KEY_MAX_TARGETS, DEFAULT_MAX_TARGET_PRESETS);
    maxTargets_ = clampMaxTargets(storedMax);
    listStore_.load(presets_, maxTargets_);
    sortPresets();
}

void TargetPresetManager::saveToNvs() {
    listStore_.save(presets_);
}

#if !defined(ARDUINO)
void TargetPresetManager::resetHostStorage() {
    NvsConfig::resetHostStorage(TARGET_CFG_NAMESPACE);
    NvsListStore<TargetPreset>::resetHostStorage(TARGET_LOG_NAMESPACE);
}

size_t TargetPresetManager::getHostStorageCount() const {
    return listStore_.getHostCount();
}

size_t TargetPresetManager::getHostStorageMaxTargets() const {
    return cfgStore_.getUShort(TARGET_CFG_KEY_MAX_TARGETS, DEFAULT_MAX_TARGET_PRESETS);
}
#endif
