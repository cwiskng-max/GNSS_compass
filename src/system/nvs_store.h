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
            prefs.putUShort(key, value);
            prefs.end();
            return true;
        }
        return false;
#else
        hostUShortStore()[ns_][key] = value;
        return true;
#endif
    }

#if !defined(ARDUINO)
    static void resetHostStorage(const char *ns = nullptr) {
        if (ns) {
            hostUShortStore().erase(ns);
        } else {
            hostUShortStore().clear();
        }
    }

    bool hasHostKey(const char *key) const {
        if (!key) return false;
        const auto &store = hostUShortStore();
        auto nsIt = store.find(ns_);
        if (nsIt == store.end()) return false;
        return nsIt->second.find(key) != nsIt->second.end();
    }
#endif

private:
    std::string ns_;

#if !defined(ARDUINO)
    static std::map<std::string, std::map<std::string, uint16_t>>& hostUShortStore() {
        static std::map<std::string, std::map<std::string, uint16_t>> s_store;
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

        size_t count = it->second.size();
        if (count > maxCount) {
            count = maxCount;
        }
        out.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            if (it->second[i].size() == sizeof(T)) {
                T item;
                std::memcpy(&item, it->second[i].data(), sizeof(T));
                out.push_back(item);
            } else {
                break;
            }
        }
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
        auto &vec = hostBlobStore()[ns_];
        vec.clear();
        vec.reserve(items.size());
        for (size_t i = 0; i < items.size(); ++i) {
            std::vector<uint8_t> bytes(sizeof(T));
            std::memcpy(bytes.data(), &items[i], sizeof(T));
            vec.push_back(bytes);
        }
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
        auto &vec = hostBlobStore()[ns_];
        if (index >= vec.size()) {
            vec.resize(index + 1);
        }
        vec[index].resize(sizeof(T));
        std::memcpy(vec[index].data(), &item, sizeof(T));
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
        auto &vec = hostBlobStore()[ns_];
        if (vec.size() > newCount) {
            vec.resize(newCount);
        }
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
        (void)index;
        auto &vec = hostBlobStore()[ns_];
        if (newCount < vec.size()) {
            vec.resize(newCount);
        }
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
        return true;
#endif
    }

#if !defined(ARDUINO)
    static void resetHostStorage(const char *ns = nullptr) {
        if (ns) {
            hostBlobStore().erase(ns);
        } else {
            hostBlobStore().clear();
        }
    }

    size_t getHostCount() const {
        const auto &store = hostBlobStore();
        auto it = store.find(ns_);
        if (it == store.end()) return 0;
        return it->second.size();
    }
#endif

private:
    std::string ns_;
    std::string countKey_;
    char keyPrefix_;

#if !defined(ARDUINO)
    static std::map<std::string, std::vector<std::vector<uint8_t>>>& hostBlobStore() {
        static std::map<std::string, std::vector<std::vector<uint8_t>>> s_store;
        return s_store;
    }
#endif
};
