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
    if (presets_.size() <= 1) return;
    if (nvsSlots_.size() != presets_.size()) {
        nvsSlots_.resize(presets_.size());
        for (size_t i = 0; i < presets_.size(); ++i) {
            nvsSlots_[i] = i;
        }
    }

    std::vector<size_t> order(presets_.size());
    for (size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    std::stable_sort(order.begin(), order.end(),
        [this](size_t ia, size_t ib) {
            const TargetPreset &a = presets_[ia];
            const TargetPreset &b = presets_[ib];
            if (a.useCount != b.useCount) {
                return a.useCount > b.useCount;
            }
            return a.lastUsedTime > b.lastUsedTime;
        });

    std::vector<TargetPreset> sortedPresets;
    std::vector<size_t> sortedSlots;
    sortedPresets.reserve(presets_.size());
    sortedSlots.reserve(nvsSlots_.size());
    for (size_t idx : order) {
        sortedPresets.push_back(presets_[idx]);
        sortedSlots.push_back(nvsSlots_[idx]);
    }
    presets_.swap(sortedPresets);
    nvsSlots_.swap(sortedSlots);
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
        size_t idx = static_cast<size_t>(existingIdx);
        TargetPreset updated = presets_[idx];
        if (nowSec > 0) {
            updated.lastUsedTime = nowSec;
        }
        if (label && label[0] != '\0') {
            updated.setLabel(label);
        }
        if (idx < nvsSlots_.size() && !listStore_.putSlot(nvsSlots_[idx], updated)) {
            return false;
        }
        presets_[idx] = updated;
        sortPresets();
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

    TargetPreset preset(norm, effectiveLabel, 0, nowSec);

    if (presets_.size() < maxTargets_) {
        size_t newSlot = presets_.size();
        if (!listStore_.putSlotAndCount(newSlot, preset, newSlot + 1)) {
            return false;
        }
        presets_.push_back(preset);
        nvsSlots_.push_back(newSlot);
        sortPresets();
        return true;
    }

    // Evict least-used / oldest entry (at the back of the sorted vector)
    bool needsTruncateSave = (presets_.size() > maxTargets_);
    while (presets_.size() > maxTargets_) {
        presets_.pop_back();
        if (!nvsSlots_.empty()) nvsSlots_.pop_back();
    }

    if (needsTruncateSave) {
        presets_.pop_back();
        presets_.push_back(preset);
        sortPresets();
        saveToNvs();
        return true;
    }

    size_t evictedSlot = nvsSlots_.empty() ? (presets_.size() - 1) : nvsSlots_.back();
    if (!listStore_.putSlot(evictedSlot, preset)) {
        return false;
    }
    presets_.pop_back();
    if (!nvsSlots_.empty()) nvsSlots_.pop_back();
    presets_.push_back(preset);
    nvsSlots_.push_back(evictedSlot);
    sortPresets();
    return true;
}

bool TargetPresetManager::recordUse(size_t index, uint32_t nowSec) {
    if (index >= presets_.size()) {
        return false;
    }
    TargetPreset updated = presets_[index];
    if (updated.useCount < UINT16_MAX) {
        updated.useCount++;
    }
    if (nowSec > 0) {
        updated.lastUsedTime = nowSec;
    }
    if (index < nvsSlots_.size()) {
        listStore_.putSlot(nvsSlots_[index], updated);
    }
    presets_[index] = updated;
    sortPresets();
    return true;
}

bool TargetPresetManager::removePreset(size_t index) {
    if (index >= presets_.size()) {
        return false;
    }
    size_t freedSlot = (index < nvsSlots_.size()) ? nvsSlots_[index] : index;
    size_t lastSlot = presets_.size() - 1;

    if (freedSlot != lastSlot) {
        for (size_t i = 0; i < nvsSlots_.size(); ++i) {
            if (i != index && nvsSlots_[i] == lastSlot) {
                listStore_.putSlot(freedSlot, presets_[i]);
                nvsSlots_[i] = freedSlot;
                break;
            }
        }
    }
    listStore_.removeSlotAndCount(lastSlot, lastSlot);

    presets_.erase(presets_.begin() + static_cast<std::ptrdiff_t>(index));
    if (index < nvsSlots_.size()) {
        nvsSlots_.erase(nvsSlots_.begin() + static_cast<std::ptrdiff_t>(index));
    }
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
    nvsSlots_.clear();
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

    bool hadCorrupt = false;
    std::vector<TargetPreset> clean;
    clean.reserve(presets_.size());
    for (size_t i = 0; i < presets_.size(); ++i) {
        TargetPreset p = presets_[i];
        p.grid[sizeof(p.grid) - 1] = '\0';
        p.label[sizeof(p.label) - 1] = '\0';
        char norm[7];
        if (!normalizeGrid(p.grid, norm)) {
            hadCorrupt = true;
            continue;
        }
        p.setGrid(norm);
        clean.push_back(p);
    }
    presets_.swap(clean);

    nvsSlots_.resize(presets_.size());
    for (size_t i = 0; i < presets_.size(); ++i) {
        nvsSlots_[i] = i;
    }
    sortPresets();
    if (hadCorrupt) {
        saveToNvs();
    }
}

void TargetPresetManager::saveToNvs() {
    nvsSlots_.resize(presets_.size());
    for (size_t i = 0; i < presets_.size(); ++i) {
        nvsSlots_[i] = i;
    }
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
