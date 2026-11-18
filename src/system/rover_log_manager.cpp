#include "rover_log_manager.h"

RoverLogManager::RoverLogManager()
    : maxStops_(DEFAULT_MAX_ROVER_STOPS),
      cfgStore_(ROVER_CFG_NAMESPACE),
      listStore_(ROVER_LOG_NAMESPACE, ROVER_LOG_KEY_COUNT, 's') {}

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
        cfgStore_.putUShort(ROVER_CFG_KEY_MAX_STOPS, static_cast<uint16_t>(maxStops_));
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
        listStore_.putSlotAndCount(stops_.size() - 1, stops_.back(), stops_.size());
    }
    return true;
}

bool RoverLogManager::popLastStop() {
    if (stops_.empty()) {
        return false;
    }

    size_t removedIndex = stops_.size() - 1;
    stops_.pop_back();
    listStore_.removeSlotAndCount(removedIndex, stops_.size());

    return true;
}

void RoverLogManager::clearHistory() {
    stops_.clear();
    listStore_.clear();
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
    uint16_t storedMax = cfgStore_.getUShort(ROVER_CFG_KEY_MAX_STOPS, DEFAULT_MAX_ROVER_STOPS);
    maxStops_ = clampMaxStops(storedMax);
    listStore_.load(stops_, maxStops_);
}

void RoverLogManager::saveToNvs() {
    listStore_.save(stops_);
}

#if !defined(ARDUINO)
void RoverLogManager::resetHostStorage() {
    NvsConfig::resetHostStorage(ROVER_CFG_NAMESPACE);
    NvsListStore<RoverStop>::resetHostStorage(ROVER_LOG_NAMESPACE);
}

size_t RoverLogManager::getHostStorageCount() {
    return NvsListStore<RoverStop>(ROVER_LOG_NAMESPACE, ROVER_LOG_KEY_COUNT, 's').getHostCount();
}

size_t RoverLogManager::getHostStorageMaxStops() {
    return NvsConfig(ROVER_CFG_NAMESPACE).getUShort(ROVER_CFG_KEY_MAX_STOPS, DEFAULT_MAX_ROVER_STOPS);
}

bool RoverLogManager::isHostStorageMaxStopsExplicit() {
    return NvsConfig(ROVER_CFG_NAMESPACE).hasHostKey(ROVER_CFG_KEY_MAX_STOPS);
}
#endif

