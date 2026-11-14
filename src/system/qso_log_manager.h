#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include <string>
#include "qso_entry.h"
#include "../config.h"

/**
 * @brief Manages the persistent contest QSO log in ESP32 NVS flash.
 *
 * Implements ring buffer / stack operations (append, undo/popLastQso,
 * 1-based deleteQsoByIndex with re-indexing, clearLog), capacity bounds
 * (default 200, clamped 10-1000), and exports to Cabrillo 3.0, ADIF, CSV,
 * and contest statistics calculations.
 */
class QsoLogManager {
public:
    QsoLogManager();

    // Initializes manager and loads stored history, capacity, and default band from NVS
    void begin();

    // Ring buffer / stack operations
    bool appendQso(const QsoEntry &qso);
    bool append(const QsoEntry &qso) { return appendQso(qso); }
    bool popLastQso();
    bool deleteQsoByIndex(size_t index); // 1-based index with re-indexing
    void clearLog();

    // Inspection & accessors
    size_t getQsoCount() const { return qsos_.size(); }
    bool isEmpty() const { return qsos_.empty(); }

    const QsoEntry* getQso(size_t index) const; // 0-based
    bool getQso(size_t index, QsoEntry &out) const;

    const QsoEntry* getLastQso() const;
    bool getLastQso(QsoEntry &out) const;

    const std::vector<QsoEntry>& getQsos() const { return qsos_; }

    // Capacity configuration (persisted under "qso_cfg", key "max_qsos")
    size_t getMaxQsos() const { return maxQsos_; }
    void setMaxQsos(size_t maxQsos, bool persist = true);
    static size_t clampMaxQsos(size_t val);

    // Default band configuration (persisted under "qso_cfg", key "def_band")
    std::string getDefaultBand() const { return defaultBand_; }
    void setDefaultBand(const std::string &band, bool persist = true);

    // Station callsign for Cabrillo export (persisted under "qso_cfg", key "my_call")
    std::string getMyCall() const { return myCall_; }
    void setMyCall(const std::string &call, bool persist = true);

    // Export & formatting functions
    std::string formatCabrillo(const std::string &myCall = "",
                               const std::string &contest = "ARRL-10-GHZ") const;
    std::string formatAdif(const std::string &programId = "GNSS Compass") const;
    std::string formatCsv() const;
    std::string formatStats() const;
    std::string formatList() const;

    // Direct NVS storage operations
    void loadFromNvs();
    void saveToNvs();

#if !defined(ARDUINO)
    // Host test mock helpers
    static void resetHostStorage();
    static size_t getHostStorageCount();
    static size_t getHostStorageMaxQsos();
    static std::string getHostStorageDefaultBand();
    static std::string getHostStorageMyCall();
#endif

private:
    std::vector<QsoEntry> qsos_;
    size_t maxQsos_;
    std::string defaultBand_;
    std::string myCall_;

#if !defined(ARDUINO)
    static std::vector<QsoEntry> s_hostNvsQsos;
    static size_t s_hostNvsMaxQsos;
    static bool s_hostNvsMaxQsosStored;
    static std::string s_hostNvsDefaultBand;
    static bool s_hostNvsDefaultBandStored;
    static std::string s_hostNvsMyCall;
    static bool s_hostNvsMyCallStored;
#endif
};

