#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#if defined(ARDUINO)
#include <Arduino.h>
#include <Preferences.h>
#else
#include <map>
#endif

/**
 * @brief Lightweight header-only helper for scalar NVS configuration keys.
 *
 * Uses ESP32 Preferences under ARDUINO and an in-memory map on host builds
 * so managers can be unit-tested without duplicating #if defined(ARDUINO) boilerplate.
 */
class NvsConfig {
public:
    explicit NvsConfig(const char *ns) : ns_(ns ? ns : "") {}

    uint16_t getUShort(const char *key, uint16_t defaultValue) const {
        if (!key || ns_.empty()) return defaultValue;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), true)) {
            uint16_t val = prefs.getUShort(key, defaultValue);
            prefs.end();
            return val;
        }
        return defaultValue;
#else
        const auto &store = hostUShortStore();
        auto nsIt = store.find(ns_);
        if (nsIt == store.end()) return defaultValue;
        auto keyIt = nsIt->second.find(key);
        if (keyIt == nsIt->second.end()) return defaultValue;
        return keyIt->second;
#endif
    }

    bool putUShort(const char *key, uint16_t value) {
        if (!key || ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            size_t written = prefs.putUShort(key, value);
            prefs.end();
            return written == sizeof(uint16_t);
        }
        return false;
#else
        hostUShortStore()[ns_][key] = value;
        return true;
#endif
    }

    std::string getString(const char *key, const std::string &defaultValue) const {
        if (!key || ns_.empty()) return defaultValue;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), true)) {
            String val = prefs.getString(key, defaultValue.c_str());
            prefs.end();
            return std::string(val.c_str());
        }
        return defaultValue;
#else
        const auto &store = hostStringStore();
        auto nsIt = store.find(ns_);
        if (nsIt == store.end()) return defaultValue;
        auto keyIt = nsIt->second.find(key);
        if (keyIt == nsIt->second.end()) return defaultValue;
        return keyIt->second;
#endif
    }

    bool putString(const char *key, const std::string &value) {
        if (!key || ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            size_t written = prefs.putString(key, value.c_str());
            prefs.end();
            return written == value.size();
        }
        return false;
#else
        hostStringStore()[ns_][key] = value;
        return true;
#endif
    }

    float getFloat(const char *key, float defaultValue) const {
        if (!key || ns_.empty()) return defaultValue;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), true)) {
            float val = prefs.getFloat(key, defaultValue);
            prefs.end();
            return val;
        }
        return defaultValue;
#else
        const auto &store = hostFloatStore();
        auto nsIt = store.find(ns_);
        if (nsIt == store.end()) return defaultValue;
        auto keyIt = nsIt->second.find(key);
        if (keyIt == nsIt->second.end()) return defaultValue;
        return keyIt->second;
#endif
    }

    bool putFloat(const char *key, float value) {
        if (!key || ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            size_t written = prefs.putFloat(key, value);
            prefs.end();
            return written == sizeof(float);
        }
        return false;
#else
        hostFloatStore()[ns_][key] = value;
        return true;
#endif
    }

    bool remove(const char *key) {
        if (!key || ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            bool ok = prefs.remove(key);
            prefs.end();
            return ok;
        }
        return false;
#else
        bool removed = false;
        auto uIt = hostUShortStore().find(ns_);
        if (uIt != hostUShortStore().end()) removed |= (uIt->second.erase(key) > 0);
        auto sIt = hostStringStore().find(ns_);
        if (sIt != hostStringStore().end()) removed |= (sIt->second.erase(key) > 0);
        auto fIt = hostFloatStore().find(ns_);
        if (fIt != hostFloatStore().end()) removed |= (fIt->second.erase(key) > 0);
        return removed;
#endif
    }

#if !defined(ARDUINO)
    static void resetHostStorage(const char *ns = nullptr) {
        if (ns) {
            hostUShortStore().erase(ns);
            hostStringStore().erase(ns);
            hostFloatStore().erase(ns);
        } else {
            hostUShortStore().clear();
            hostStringStore().clear();
            hostFloatStore().clear();
        }
    }

    bool hasHostKey(const char *key) const {
        if (!key) return false;
        const auto &uStore = hostUShortStore();
        auto uNsIt = uStore.find(ns_);
        if (uNsIt != uStore.end() && uNsIt->second.find(key) != uNsIt->second.end()) {
            return true;
        }
        const auto &sStore = hostStringStore();
        auto sNsIt = sStore.find(ns_);
        if (sNsIt != sStore.end() && sNsIt->second.find(key) != sNsIt->second.end()) {
            return true;
        }
        const auto &fStore = hostFloatStore();
        auto fNsIt = fStore.find(ns_);
        if (fNsIt != fStore.end() && fNsIt->second.find(key) != fNsIt->second.end()) {
            return true;
        }
        return false;
    }
#endif

private:
    std::string ns_;

#if !defined(ARDUINO)
    static std::map<std::string, std::map<std::string, uint16_t>>& hostUShortStore() {
        static std::map<std::string, std::map<std::string, uint16_t>> s_store;
        return s_store;
    }

    static std::map<std::string, std::map<std::string, std::string>>& hostStringStore() {
        static std::map<std::string, std::map<std::string, std::string>> s_store;
        return s_store;
    }

    static std::map<std::string, std::map<std::string, float>>& hostFloatStore() {
        static std::map<std::string, std::map<std::string, float>> s_store;
        return s_store;
    }
#endif
};

/**
 * @brief Header-only indexed NVS list persistence helper (keys t0, t1, ... + count).
 *
 * Avoids the ~4 KB ESP-IDF single-blob limit and overwrites entries in place
 * without calling clear() mid-write, cleaning up trailing orphaned keys when
 * the list shrinks.
 */
template <typename T>
class NvsListStore {
public:
    explicit NvsListStore(const char *ns, const char *countKey = "count", char keyPrefix = 't')
        : ns_(ns ? ns : ""),
          countKey_(countKey ? countKey : "count"),
          keyPrefix_(keyPrefix) {}

    void load(std::vector<T> &out, size_t maxCount) const {
        out.clear();
        if (ns_.empty()) return;

#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), true)) {
            uint16_t count = prefs.getUShort(countKey_.c_str(), 0);
            if (count > maxCount) {
                count = static_cast<uint16_t>(maxCount);
            }
            out.reserve(count);
            for (uint16_t i = 0; i < count; ++i) {
                char key[16];
                std::snprintf(key, sizeof(key), "%c%u", keyPrefix_, static_cast<unsigned int>(i));
                T item;
                size_t bytesRead = prefs.getBytes(key, &item, sizeof(T));
                if (bytesRead == sizeof(T)) {
                    out.push_back(item);
                } else {
                    break;
                }
            }
            prefs.end();
        }
#else
        const auto &store = hostBlobStore();
        auto it = store.find(ns_);
        if (it == store.end()) return;

        size_t count = getHostCount();
        if (count > maxCount) {
            count = maxCount;
        }
        out.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            auto slotIt = it->second.find(i);
            if (slotIt != it->second.end() && slotIt->second.size() == sizeof(T)) {
                T item;
                std::memcpy(&item, slotIt->second.data(), sizeof(T));
                out.push_back(item);
            } else {
                break;
            }
        }
#endif
    }

    size_t getCount() const {
        if (ns_.empty()) return 0;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), true)) {
            uint16_t count = prefs.getUShort(countKey_.c_str(), 0);
            prefs.end();
            return count;
        }
        return 0;
#else
        return getHostCount();
#endif
    }

    bool putCount(size_t newCount) {
        if (ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            size_t written = prefs.putUShort(countKey_.c_str(), static_cast<uint16_t>(newCount));
            prefs.end();
            return written == sizeof(uint16_t);
        }
        return false;
#else
        hostCountStore()[ns_] = newCount;
        return true;
#endif
    }

    bool getRawSlot(size_t index, void *buf, size_t bufSize, size_t &bytesRead) const {
        bytesRead = 0;
        if (ns_.empty() || !buf || bufSize == 0) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), true)) {
            char key[16];
            std::snprintf(key, sizeof(key), "%c%u", keyPrefix_, static_cast<unsigned int>(index));
            bytesRead = prefs.getBytes(key, buf, bufSize);
            prefs.end();
            return bytesRead > 0;
        }
        return false;
#else
        const auto &store = hostBlobStore();
        auto nsIt = store.find(ns_);
        if (nsIt == store.end()) return false;
        auto slotIt = nsIt->second.find(index);
        if (slotIt == nsIt->second.end()) return false;
        const auto &blob = slotIt->second;
        if (blob.size() > bufSize) {
            return false;
        }
        std::memcpy(buf, blob.data(), blob.size());
        bytesRead = blob.size();
        return bytesRead > 0;
#endif
    }

    bool save(const std::vector<T> &items) {
        if (ns_.empty()) return false;

#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            uint16_t oldCount = prefs.getUShort(countKey_.c_str(), 0);
            size_t written = 0;
            bool ok = true;

            // Overwrite entries in place (never clear() to protect against mid-write power loss)
            for (size_t i = 0; i < items.size(); ++i) {
                char key[16];
                std::snprintf(key, sizeof(key), "%c%u", keyPrefix_, static_cast<unsigned int>(i));
                if (prefs.putBytes(key, &items[i], sizeof(T)) != sizeof(T)) {
                    ok = false;
                    break;
                }
                written++;
            }

            // Clean up trailing orphaned keys if the list shrunk (or failed mid-write)
            for (size_t i = written; i < oldCount; ++i) {
                char key[16];
                std::snprintf(key, sizeof(key), "%c%u", keyPrefix_, static_cast<unsigned int>(i));
                prefs.remove(key);
            }

            prefs.putUShort(countKey_.c_str(), static_cast<uint16_t>(written));
            prefs.end();
            return ok;
        }
        return false;
#else
        auto &slots = hostBlobStore()[ns_];
        slots.clear();
        for (size_t i = 0; i < items.size(); ++i) {
            std::vector<uint8_t> bytes(sizeof(T));
            std::memcpy(bytes.data(), &items[i], sizeof(T));
            slots[i] = bytes;
        }
        hostCountStore()[ns_] = items.size();
        return true;
#endif
    }

    bool putSlot(size_t index, const T &item) {
        if (ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            char key[16];
            std::snprintf(key, sizeof(key), "%c%u", keyPrefix_, static_cast<unsigned int>(index));
            bool ok = (prefs.putBytes(key, &item, sizeof(T)) == sizeof(T));
            prefs.end();
            return ok;
        }
        return false;
#else
        std::vector<uint8_t> bytes(sizeof(T));
        std::memcpy(bytes.data(), &item, sizeof(T));
        hostBlobStore()[ns_][index] = bytes;
        return true;
#endif
    }

    bool putSlotAndCount(size_t index, const T &item, size_t newCount) {
        if (ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            char key[16];
            std::snprintf(key, sizeof(key), "%c%u", keyPrefix_, static_cast<unsigned int>(index));
            bool ok = (prefs.putBytes(key, &item, sizeof(T)) == sizeof(T));
            if (ok) {
                prefs.putUShort(countKey_.c_str(), static_cast<uint16_t>(newCount));
            }
            prefs.end();
            return ok;
        }
        return false;
#else
        if (!putSlot(index, item)) return false;
        hostCountStore()[ns_] = newCount;
        return true;
#endif
    }

    bool removeSlotAndCount(size_t index, size_t newCount) {
        if (ns_.empty()) return false;
#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            char key[16];
            std::snprintf(key, sizeof(key), "%c%u", keyPrefix_, static_cast<unsigned int>(index));
            prefs.remove(key);
            prefs.putUShort(countKey_.c_str(), static_cast<uint16_t>(newCount));
            prefs.end();
            return true;
        }
        return false;
#else
        hostBlobStore()[ns_].erase(index);
        hostCountStore()[ns_] = newCount;
        return true;
#endif
    }

    bool clear() {
        if (ns_.empty()) return false;

#if defined(ARDUINO)
        Preferences prefs;
        if (prefs.begin(ns_.c_str(), false)) {
            prefs.clear();
            prefs.end();
            return true;
        }
        return false;
#else
        hostBlobStore()[ns_].clear();
        hostCountStore()[ns_] = 0;
        return true;
#endif
    }

#if !defined(ARDUINO)
    static void resetHostStorage(const char *ns = nullptr) {
        if (ns) {
            hostBlobStore().erase(ns);
            hostCountStore().erase(ns);
        } else {
            hostBlobStore().clear();
            hostCountStore().clear();
        }
    }

    size_t getHostCount() const {
        const auto &counts = hostCountStore();
        auto it = counts.find(ns_);
        if (it == counts.end()) return 0;
        return it->second;
    }

    void putHostRawSlot(size_t index, const void *buf, size_t len) {
        if (ns_.empty() || !buf || len == 0) return;
        std::vector<uint8_t> bytes(len);
        std::memcpy(bytes.data(), buf, len);
        hostBlobStore()[ns_][index] = bytes;
    }
#endif

private:
    std::string ns_;
    std::string countKey_;
    char keyPrefix_;

#if !defined(ARDUINO)
    static std::map<std::string, std::map<size_t, std::vector<uint8_t>>>& hostBlobStore() {
        static std::map<std::string, std::map<size_t, std::vector<uint8_t>>> s_store;
        return s_store;
    }

    static std::map<std::string, size_t>& hostCountStore() {
        static std::map<std::string, size_t> s_counts;
        return s_counts;
    }
#endif
};
