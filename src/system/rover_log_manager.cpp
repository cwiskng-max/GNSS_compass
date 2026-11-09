#include "rover_log_manager.h"
#include <cstdio>

#if defined(ARDUINO)
#include <Arduino.h>
#include <Preferences.h>
#else
std::vector<RoverStop> RoverLogManager::s_hostNvsStops;
size_t RoverLogManager::s_hostNvsMaxStops = DEFAULT_MAX_ROVER_STOPS;
bool RoverLogManager::s_hostNvsMaxStopsStored = false;
#endif

RoverLogManager::RoverLogManager()
    : maxStops_(DEFAULT_MAX_ROVER_STOPS) {}

void RoverLogManager::begin() {
    loadFromNvs();
}

size_t RoverLogManager::clampMaxStops(size_t val) {
    if (val < MIN_ROVER_STOPS) return MIN_ROVER_STOPS;
    if (val > ABS_MAX_ROVER_STOPS) return ABS_MAX_ROVER_STOPS;
    return val;
}

void RoverLogManager::setMaxStops(size_t maxStops, bool persist) {
    maxStops_ = clampMaxStops(maxStops);

    bool truncated = false;
    if (stops_.size() > maxStops_) {
        size_t excess = stops_.size() - maxStops_;
        stops_.erase(stops_.begin(), stops_.begin() + excess);
        truncated = true;
    }

    if (persist) {
#if defined(ARDUINO)
        Preferences cfgPrefs;
        if (cfgPrefs.begin(ROVER_CFG_NAMESPACE, false)) {
            cfgPrefs.putUShort(ROVER_CFG_KEY_MAX_STOPS, (uint16_t)maxStops_);
            cfgPrefs.end();
        }
#else
        s_hostNvsMaxStops = maxStops_;
        s_hostNvsMaxStopsStored = true;
#endif
    }

    if (truncated) {
        saveToNvs();
    }
}

bool RoverLogManager::appendStop(const RoverStop &stop) {
    if (maxStops_ == 0) {
        maxStops_ = DEFAULT_MAX_ROVER_STOPS;
    }

    if (stops_.size() >= maxStops_) {
        // Discard oldest stop to preserve capacity (FIFO rollover)
        stops_.erase(stops_.begin());
        stops_.push_back(stop);
        saveToNvs();
    } else {
        stops_.push_back(stop);
#if defined(ARDUINO)
        Preferences logPrefs;
        if (logPrefs.begin(ROVER_LOG_NAMESPACE, false)) {
            char key[16];
            snprintf(key, sizeof(key), "s%u", (unsigned int)(stops_.size() - 1));
            logPrefs.putBytes(key, &stops_.back(), sizeof(RoverStop));
            logPrefs.putUShort(ROVER_LOG_KEY_COUNT, (uint16_t)stops_.size());
            logPrefs.end();
        }
#else
        s_hostNvsStops.push_back(stop);
#endif
    }
    return true;
}

bool RoverLogManager::popLastStop() {
    if (stops_.empty()) {
        return false;
    }

#if defined(ARDUINO)
    size_t removedIndex = stops_.size() - 1;
#endif
    stops_.pop_back();

#if defined(ARDUINO)
    Preferences logPrefs;
    if (logPrefs.begin(ROVER_LOG_NAMESPACE, false)) {
        char key[16];
        snprintf(key, sizeof(key), "s%u", (unsigned int)removedIndex);
        logPrefs.remove(key);
        logPrefs.putUShort(ROVER_LOG_KEY_COUNT, (uint16_t)stops_.size());
        logPrefs.end();
    }
#else
    if (!s_hostNvsStops.empty()) {
        s_hostNvsStops.pop_back();
    }
#endif

    return true;
}

void RoverLogManager::clearHistory() {
    stops_.clear();

#if defined(ARDUINO)
    Preferences logPrefs;
    if (logPrefs.begin(ROVER_LOG_NAMESPACE, false)) {
        logPrefs.clear();
        logPrefs.end();
    }
#else
    s_hostNvsStops.clear();
#endif
}

const RoverStop* RoverLogManager::getStop(size_t index) const {
    if (index >= stops_.size()) return nullptr;
    return &stops_[index];
}

bool RoverLogManager::getStop(size_t index, RoverStop &out) const {
    if (index >= stops_.size()) return false;
    out = stops_[index];
    return true;
}

const RoverStop* RoverLogManager::getLastStop() const {
    if (stops_.empty()) return nullptr;
    return &stops_.back();
}

bool RoverLogManager::getLastStop(RoverStop &out) const {
    if (stops_.empty()) return false;
    out = stops_.back();
    return true;
}

const RoverStop* RoverLogManager::getPriorStop() const {
    if (stops_.size() < 2) return nullptr;
    return &stops_[stops_.size() - 2];
}

bool RoverLogManager::getPriorStop(RoverStop &out) const {
    if (stops_.size() < 2) return false;
    out = stops_[stops_.size() - 2];
    return true;
}

bool RoverLogManager::hasVisitedGrid(const char *grid) const {
    if (!grid || std::strlen(grid) < 6) {
        return false;
    }
    for (const auto &stop : stops_) {
        if (std::strlen(stop.grid) < 6) {
            continue;
        }
        bool match = true;
        for (int i = 0; i < 6; ++i) {
            if (std::toupper((unsigned char)stop.grid[i]) != std::toupper((unsigned char)grid[i])) {
                match = false;
                break;
            }
        }
        if (match) {
            return true;
        }
    }
    return false;
}

void RoverLogManager::loadFromNvs() {
#if defined(ARDUINO)
    Preferences cfgPrefs;
    if (cfgPrefs.begin(ROVER_CFG_NAMESPACE, true)) {
        uint16_t storedMax = cfgPrefs.getUShort(ROVER_CFG_KEY_MAX_STOPS, DEFAULT_MAX_ROVER_STOPS);
        maxStops_ = clampMaxStops(storedMax);
        cfgPrefs.end();
    } else {
        maxStops_ = DEFAULT_MAX_ROVER_STOPS;
    }

    stops_.clear();
    Preferences logPrefs;
    if (logPrefs.begin(ROVER_LOG_NAMESPACE, true)) {
        uint16_t count = logPrefs.getUShort(ROVER_LOG_KEY_COUNT, 0);
        if (count > maxStops_) {
            count = (uint16_t)maxStops_;
        }
        stops_.reserve(count);
        for (uint16_t i = 0; i < count; ++i) {
            char key[16];
            snprintf(key, sizeof(key), "s%u", (unsigned int)i);
            RoverStop stop;
            size_t bytesRead = logPrefs.getBytes(key, &stop, sizeof(RoverStop));
            if (bytesRead == sizeof(RoverStop)) {
                stops_.push_back(stop);
            } else {
                break;
            }
        }
        logPrefs.end();
    }
#else
    if (s_hostNvsMaxStopsStored) {
        maxStops_ = clampMaxStops(s_hostNvsMaxStops);
    } else {
        maxStops_ = DEFAULT_MAX_ROVER_STOPS;
    }

    stops_.clear();
    size_t count = s_hostNvsStops.size();
    if (count > maxStops_) {
        count = maxStops_;
    }
    stops_.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        stops_.push_back(s_hostNvsStops[i]);
    }
#endif
}

void RoverLogManager::saveToNvs() {
#if defined(ARDUINO)
    Preferences logPrefs;
    if (logPrefs.begin(ROVER_LOG_NAMESPACE, false)) {
        uint16_t oldCount = logPrefs.getUShort(ROVER_LOG_KEY_COUNT, 0);

        // Overwrite entries in place (never clear() to protect against mid-write power loss)
        for (size_t i = 0; i < stops_.size(); ++i) {
            char key[16];
            snprintf(key, sizeof(key), "s%u", (unsigned int)i);
            logPrefs.putBytes(key, &stops_[i], sizeof(RoverStop));
        }

        // Clean up any trailing orphaned keys if capacity shrunk
        for (size_t i = stops_.size(); i < oldCount; ++i) {
            char key[16];
            snprintf(key, sizeof(key), "s%u", (unsigned int)i);
            logPrefs.remove(key);
        }

        logPrefs.putUShort(ROVER_LOG_KEY_COUNT, (uint16_t)stops_.size());
        logPrefs.end();
    }
#else
    s_hostNvsStops = stops_;
#endif
}

#if !defined(ARDUINO)
void RoverLogManager::resetHostStorage() {
    s_hostNvsStops.clear();
    s_hostNvsMaxStops = DEFAULT_MAX_ROVER_STOPS;
    s_hostNvsMaxStopsStored = false;
}

size_t RoverLogManager::getHostStorageCount() {
    return s_hostNvsStops.size();
}

size_t RoverLogManager::getHostStorageMaxStops() {
    return s_hostNvsMaxStops;
}

bool RoverLogManager::isHostStorageMaxStopsExplicit() {
    return s_hostNvsMaxStopsStored;
}
#endif

