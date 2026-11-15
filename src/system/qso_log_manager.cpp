#include "qso_log_manager.h"
#include "../protocol/voice_qso_parser.h"
#include <cstdio>
#include <ctime>
#include <set>
#include <map>
#include <algorithm>

#if defined(ARDUINO)
#include <Arduino.h>
#include <Preferences.h>
#else
std::vector<QsoEntry> QsoLogManager::s_hostNvsQsos;
size_t QsoLogManager::s_hostNvsMaxQsos = DEFAULT_MAX_QSOS;
bool QsoLogManager::s_hostNvsMaxQsosStored = false;
std::string QsoLogManager::s_hostNvsDefaultBand = DEFAULT_QSO_BAND;
bool QsoLogManager::s_hostNvsDefaultBandStored = false;
std::string QsoLogManager::s_hostNvsMyCall = DEFAULT_QSO_MY_CALL;
bool QsoLogManager::s_hostNvsMyCallStored = false;
#endif

namespace {

#if defined(ARDUINO)
constexpr uint32_t kNvsQsoMagic = 0x51534F31u; // "QSO1"
constexpr uint16_t kNvsQsoVersion = 1;
constexpr const char *kNvsKeyHead = "head";

uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

#pragma pack(push, 1)
struct NvsQsoRecord {
    uint32_t magic;
    uint16_t version;
    uint16_t crc16;
    QsoEntry entry;
};
#pragma pack(pop)

NvsQsoRecord makeNvsRecord(const QsoEntry &qso) {
    NvsQsoRecord rec;
    rec.magic = kNvsQsoMagic;
    rec.version = kNvsQsoVersion;
    rec.entry = qso;
    rec.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&rec.entry), sizeof(QsoEntry));
    return rec;
}

bool validateNvsRecord(const NvsQsoRecord &rec) {
    if (rec.magic != kNvsQsoMagic || rec.version != kNvsQsoVersion) {
        return false;
    }
    uint16_t expected = crc16Ccitt(reinterpret_cast<const uint8_t *>(&rec.entry), sizeof(QsoEntry));
    return rec.crc16 == expected;
}
#endif

void epochToDateTime(uint32_t ts, int &year, int &month, int &day, int &hour, int &minute, int &second) {
    if (ts >= 86400 * 365) {
        time_t t = static_cast<time_t>(ts);
        struct tm tmBuf;
#if defined(_WIN32)
        gmtime_s(&tmBuf, &t);
#else
        gmtime_r(&t, &tmBuf);
#endif
        year = tmBuf.tm_year + 1900;
        month = tmBuf.tm_mon + 1;
        day = tmBuf.tm_mday;
        hour = tmBuf.tm_hour;
        minute = tmBuf.tm_min;
        second = tmBuf.tm_sec;
    } else {
        year = 1970;
        month = 1;
        day = 1;
        hour = (ts / 3600) % 24;
        minute = (ts / 60) % 60;
        second = ts % 60;
    }
}

} // anonymous namespace

QsoLogManager::QsoLogManager()
    : maxQsos_(DEFAULT_MAX_QSOS), nvsHead_(0), defaultBand_(DEFAULT_QSO_BAND), myCall_(DEFAULT_QSO_MY_CALL) {}

void QsoLogManager::begin() {
    loadFromNvs();
}

size_t QsoLogManager::clampMaxQsos(size_t val) {
    if (val < MIN_QSOS) return MIN_QSOS;
    if (val > ABS_MAX_QSOS) return ABS_MAX_QSOS;
    return val;
}

void QsoLogManager::setMaxQsos(size_t maxQsos, bool persist) {
    size_t oldMax = maxQsos_;
    maxQsos_ = clampMaxQsos(maxQsos);

    bool truncated = false;
    if (qsos_.size() > maxQsos_) {
        size_t excess = qsos_.size() - maxQsos_;
        qsos_.erase(qsos_.begin(), qsos_.begin() + excess);
        truncated = true;
    }

    if (persist) {
#if defined(ARDUINO)
        Preferences cfgPrefs;
        if (cfgPrefs.begin(QSO_CFG_NAMESPACE, false)) {
            cfgPrefs.putUShort(QSO_CFG_KEY_MAX_QSOS, (uint16_t)maxQsos_);
            cfgPrefs.end();
        }
#else
        s_hostNvsMaxQsos = maxQsos_;
        s_hostNvsMaxQsosStored = true;
#endif
    }

    if (truncated || (maxQsos_ != oldMax && nvsHead_ != 0)) {
        saveToNvs();
    }
}

void QsoLogManager::setDefaultBand(const std::string &band, bool persist) {
    if (band.empty()) return;
    defaultBand_ = band;

    if (persist) {
#if defined(ARDUINO)
        Preferences cfgPrefs;
        if (cfgPrefs.begin(QSO_CFG_NAMESPACE, false)) {
            cfgPrefs.putString(QSO_CFG_KEY_DEFAULT_BAND, defaultBand_.c_str());
            cfgPrefs.end();
        }
#else
        s_hostNvsDefaultBand = defaultBand_;
        s_hostNvsDefaultBandStored = true;
#endif
    }
}

void QsoLogManager::setMyCall(const std::string &call, bool persist) {
    std::string clean = call.empty() ? DEFAULT_QSO_MY_CALL : call;
    if (clean.size() > 15) {
        clean.resize(15);
    }
    myCall_ = clean;

    if (persist) {
#if defined(ARDUINO)
        Preferences cfgPrefs;
        if (cfgPrefs.begin(QSO_CFG_NAMESPACE, false)) {
            cfgPrefs.putString(QSO_CFG_KEY_MY_CALL, myCall_.c_str());
            cfgPrefs.end();
        }
#else
        s_hostNvsMyCall = myCall_;
        s_hostNvsMyCallStored = true;
#endif
    }
}

bool QsoLogManager::appendQso(const QsoEntry &qso) {
    if (maxQsos_ == 0) {
        maxQsos_ = DEFAULT_MAX_QSOS;
    }

    if (qsos_.size() >= maxQsos_) {
        // Discard oldest QSO to preserve ring buffer capacity (O(1) FIFO rollover in NVS)
        qsos_.erase(qsos_.begin());
        qsos_.push_back(qso);
#if defined(ARDUINO)
        Preferences logPrefs;
        if (logPrefs.begin(QSO_LOG_NAMESPACE, false)) {
            char key[16];
            size_t slot = nvsHead_ % maxQsos_;
            snprintf(key, sizeof(key), "q%u", (unsigned int)slot);
            NvsQsoRecord rec = makeNvsRecord(qsos_.back());
            logPrefs.putBytes(key, &rec, sizeof(rec));
            nvsHead_ = (slot + 1) % maxQsos_;
            logPrefs.putUShort(kNvsKeyHead, (uint16_t)nvsHead_);
            logPrefs.putUShort(QSO_LOG_KEY_COUNT, (uint16_t)qsos_.size());
            logPrefs.end();
        }
#else
        if (!s_hostNvsQsos.empty()) {
            s_hostNvsQsos.erase(s_hostNvsQsos.begin());
        }
        s_hostNvsQsos.push_back(qso);
#endif
    } else {
        qsos_.push_back(qso);
#if defined(ARDUINO)
        Preferences logPrefs;
        if (logPrefs.begin(QSO_LOG_NAMESPACE, false)) {
            char key[16];
            size_t slot = (nvsHead_ + qsos_.size() - 1) % maxQsos_;
            snprintf(key, sizeof(key), "q%u", (unsigned int)slot);
            NvsQsoRecord rec = makeNvsRecord(qsos_.back());
            logPrefs.putBytes(key, &rec, sizeof(rec));
            logPrefs.putUShort(QSO_LOG_KEY_COUNT, (uint16_t)qsos_.size());
            logPrefs.end();
        }
#else
        s_hostNvsQsos.push_back(qso);
#endif
    }
    return true;
}

bool QsoLogManager::popLastQso() {
    if (qsos_.empty()) {
        return false;
    }

    qsos_.pop_back();

#if defined(ARDUINO)
    Preferences logPrefs;
    if (logPrefs.begin(QSO_LOG_NAMESPACE, false)) {
        char key[16];
        size_t removedSlot = (nvsHead_ + qsos_.size()) % maxQsos_;
        snprintf(key, sizeof(key), "q%u", (unsigned int)removedSlot);
        logPrefs.remove(key);
        logPrefs.putUShort(QSO_LOG_KEY_COUNT, (uint16_t)qsos_.size());
        logPrefs.end();
    }
#else
    if (!s_hostNvsQsos.empty()) {
        s_hostNvsQsos.pop_back();
    }
#endif
    return true;
}

bool QsoLogManager::deleteQsoByIndex(size_t index) {
    if (index < 1 || index > qsos_.size()) {
        return false;
    }

    size_t vectorIndex = index - 1;
    qsos_.erase(qsos_.begin() + vectorIndex);

#if defined(ARDUINO)
    Preferences logPrefs;
    if (logPrefs.begin(QSO_LOG_NAMESPACE, false)) {
        char key[16];
        if (vectorIndex == 0) {
            // O(1) delete of oldest entry by advancing ring head
            snprintf(key, sizeof(key), "q%u", (unsigned int)(nvsHead_ % maxQsos_));
            logPrefs.remove(key);
            nvsHead_ = (nvsHead_ + 1) % maxQsos_;
            logPrefs.putUShort(kNvsKeyHead, (uint16_t)nvsHead_);
        } else {
            // Shift only trailing elements [vectorIndex .. size-1]
            for (size_t i = vectorIndex; i < qsos_.size(); ++i) {
                size_t slot = (nvsHead_ + i) % maxQsos_;
                snprintf(key, sizeof(key), "q%u", (unsigned int)slot);
                NvsQsoRecord rec = makeNvsRecord(qsos_[i]);
                logPrefs.putBytes(key, &rec, sizeof(rec));
            }
            size_t tailSlot = (nvsHead_ + qsos_.size()) % maxQsos_;
            snprintf(key, sizeof(key), "q%u", (unsigned int)tailSlot);
            logPrefs.remove(key);
        }
        logPrefs.putUShort(QSO_LOG_KEY_COUNT, (uint16_t)qsos_.size());
        logPrefs.end();
    }
#else
    if (vectorIndex < s_hostNvsQsos.size()) {
        s_hostNvsQsos.erase(s_hostNvsQsos.begin() + vectorIndex);
    }
#endif

    return true;
}

void QsoLogManager::clearLog() {
    qsos_.clear();
    nvsHead_ = 0;
#if defined(ARDUINO)
    Preferences logPrefs;
    if (logPrefs.begin(QSO_LOG_NAMESPACE, false)) {
        logPrefs.clear();
        logPrefs.putUShort(kNvsKeyHead, 0);
        logPrefs.putUShort(QSO_LOG_KEY_COUNT, 0);
        logPrefs.end();
    }
#else
    s_hostNvsQsos.clear();
#endif
}

const QsoEntry* QsoLogManager::getQso(size_t index) const {
    if (index >= qsos_.size()) return nullptr;
    return &qsos_[index];
}

bool QsoLogManager::getQso(size_t index, QsoEntry &out) const {
    if (index >= qsos_.size()) return false;
    out = qsos_[index];
    return true;
}

const QsoEntry* QsoLogManager::getLastQso() const {
    if (qsos_.empty()) return nullptr;
    return &qsos_.back();
}

bool QsoLogManager::getLastQso(QsoEntry &out) const {
    if (qsos_.empty()) return false;
    out = qsos_.back();
    return true;
}

void QsoLogManager::loadFromNvs() {
#if defined(ARDUINO)
    Preferences cfgPrefs;
    if (cfgPrefs.begin(QSO_CFG_NAMESPACE, true)) {
        uint16_t storedMax = cfgPrefs.getUShort(QSO_CFG_KEY_MAX_QSOS, DEFAULT_MAX_QSOS);
        maxQsos_ = clampMaxQsos(storedMax);
        String defB = cfgPrefs.getString(QSO_CFG_KEY_DEFAULT_BAND, DEFAULT_QSO_BAND);
        defaultBand_ = defB.c_str();
        String storedCall = cfgPrefs.getString(QSO_CFG_KEY_MY_CALL, DEFAULT_QSO_MY_CALL);
        myCall_ = storedCall.length() > 0 ? storedCall.c_str() : DEFAULT_QSO_MY_CALL;
        cfgPrefs.end();
    } else {
        maxQsos_ = DEFAULT_MAX_QSOS;
        defaultBand_ = DEFAULT_QSO_BAND;
        myCall_ = DEFAULT_QSO_MY_CALL;
    }

    qsos_.clear();
    nvsHead_ = 0;
    Preferences logPrefs;
    if (logPrefs.begin(QSO_LOG_NAMESPACE, true)) {
        uint16_t count = logPrefs.getUShort(QSO_LOG_KEY_COUNT, 0);
        uint16_t head = logPrefs.getUShort(kNvsKeyHead, 0);
        if (maxQsos_ > 0) {
            nvsHead_ = head % maxQsos_;
        }
        if (count > maxQsos_) {
            count = (uint16_t)maxQsos_;
        }
        qsos_.reserve(count);
        for (uint16_t i = 0; i < count; ++i) {
            char key[16];
            size_t slot = (nvsHead_ + i) % maxQsos_;
            snprintf(key, sizeof(key), "q%u", (unsigned int)slot);
            uint8_t rawBuf[sizeof(NvsQsoRecord)];
            size_t bytesRead = logPrefs.getBytes(key, rawBuf, sizeof(rawBuf));
            if (bytesRead == sizeof(NvsQsoRecord)) {
                NvsQsoRecord rec;
                std::memcpy(&rec, rawBuf, sizeof(rec));
                if (validateNvsRecord(rec)) {
                    qsos_.push_back(rec.entry);
                }
            } else if (bytesRead == sizeof(QsoEntry)) {
                // Legacy unversioned record fallback
                QsoEntry q;
                std::memcpy(&q, rawBuf, sizeof(QsoEntry));
                qsos_.push_back(q);
            }
        }
        logPrefs.end();
    }
#else
    if (s_hostNvsMaxQsosStored) {
        maxQsos_ = clampMaxQsos(s_hostNvsMaxQsos);
    } else {
        maxQsos_ = DEFAULT_MAX_QSOS;
    }

    if (s_hostNvsDefaultBandStored) {
        defaultBand_ = s_hostNvsDefaultBand;
    } else {
        defaultBand_ = DEFAULT_QSO_BAND;
    }

    if (s_hostNvsMyCallStored) {
        myCall_ = s_hostNvsMyCall;
    } else {
        myCall_ = DEFAULT_QSO_MY_CALL;
    }

    qsos_.clear();
    size_t count = s_hostNvsQsos.size();
    if (count > maxQsos_) {
        count = maxQsos_;
    }
    qsos_.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        qsos_.push_back(s_hostNvsQsos[i]);
    }
#endif
}

void QsoLogManager::saveToNvs() {
    nvsHead_ = 0;
#if defined(ARDUINO)
    Preferences logPrefs;
    if (logPrefs.begin(QSO_LOG_NAMESPACE, false)) {
        uint16_t oldCount = logPrefs.getUShort(QSO_LOG_KEY_COUNT, 0);
        size_t clearLimit = (oldCount > maxQsos_) ? oldCount : maxQsos_;

        for (size_t i = 0; i < qsos_.size(); ++i) {
            char key[16];
            snprintf(key, sizeof(key), "q%u", (unsigned int)i);
            NvsQsoRecord rec = makeNvsRecord(qsos_[i]);
            logPrefs.putBytes(key, &rec, sizeof(rec));
        }

        for (size_t i = qsos_.size(); i < clearLimit; ++i) {
            char key[16];
            snprintf(key, sizeof(key), "q%u", (unsigned int)i);
            if (logPrefs.isKey(key)) {
                logPrefs.remove(key);
            }
        }

        logPrefs.putUShort(kNvsKeyHead, 0);
        logPrefs.putUShort(QSO_LOG_KEY_COUNT, (uint16_t)qsos_.size());
        logPrefs.end();
    }
#else
    s_hostNvsQsos.clear();
    for (size_t i = 0; i < qsos_.size(); ++i) {
        s_hostNvsQsos.push_back(qsos_[i]);
    }
#endif
}

#if !defined(ARDUINO)
void QsoLogManager::resetHostStorage() {
    s_hostNvsQsos.clear();
    s_hostNvsMaxQsos = DEFAULT_MAX_QSOS;
    s_hostNvsMaxQsosStored = false;
    s_hostNvsDefaultBand = DEFAULT_QSO_BAND;
    s_hostNvsDefaultBandStored = false;
    s_hostNvsMyCall = DEFAULT_QSO_MY_CALL;
    s_hostNvsMyCallStored = false;
}

size_t QsoLogManager::getHostStorageCount() {
    return s_hostNvsQsos.size();
}

size_t QsoLogManager::getHostStorageMaxQsos() {
    return s_hostNvsMaxQsos;
}

std::string QsoLogManager::getHostStorageDefaultBand() {
    return s_hostNvsDefaultBand;
}

std::string QsoLogManager::getHostStorageMyCall() {
    return s_hostNvsMyCall;
}
#endif

std::string QsoLogManager::getEffectiveMyCall(const std::string &myCall) const {
    return myCall.empty()
        ? (myCall_.empty() ? std::string(DEFAULT_QSO_MY_CALL) : myCall_)
        : myCall;
}

std::string QsoLogManager::formatCabrilloHeader(const std::string &effectiveCall,
                                                const std::string &contest) {
    std::string out;
    out += "START-OF-LOG: 3.0\r\n";
    out += "CREATED-BY: GNSS Compass\r\n";
    out += "CONTEST: " + (contest.empty() ? "ARRL-10-GHZ" : contest) + "\r\n";
    out += "CALLSIGN: " + effectiveCall + "\r\n";
    return out;
}

std::string QsoLogManager::formatCabrilloRow(const QsoEntry &qso, const std::string &effectiveCall) {
    int y, m, d, hh, mm, ss;
    epochToDateTime(qso.timestamp, y, m, d, hh, mm, ss);
    std::string cabBand = VoiceQsoParser::bandToCabrillo(qso.band);

    // Cabrillo 3.0 specification for ARRL 10 GHz and VHF/Microwave contests
    // requires 6-character Maidenhead locator exchanges (e.g. CM87wa).
    // Truncate 8-digit locators to 6 digits for contest log robot compliance.
    char myGrid6[7] = {0};
    snprintf(myGrid6, sizeof(myGrid6), "%.6s", qso.my_grid);
    char hisGrid6[7] = {0};
    snprintf(hisGrid6, sizeof(hisGrid6), "%.6s", qso.his_grid);

    char line[256];
    snprintf(line, sizeof(line), "QSO: %-5s PH %04d-%02d-%02d %02d%02d %-10s %-6s %-10s %-6s\r\n",
             cabBand.c_str(), y, m, d, hh, mm,
             effectiveCall.c_str(),
             myGrid6,
             qso.callsign,
             hisGrid6);
    return std::string(line);
}

std::string QsoLogManager::formatCabrilloFooter() {
    return "END-OF-LOG:\r\n";
}

std::string QsoLogManager::formatCabrillo(const std::string &myCall,
                                         const std::string &contest) const {
    const std::string effectiveCall = getEffectiveMyCall(myCall);
    std::string out = formatCabrilloHeader(effectiveCall, contest);
    for (const auto &qso : qsos_) {
        out += formatCabrilloRow(qso, effectiveCall);
    }
    out += formatCabrilloFooter();
    return out;
}

std::string QsoLogManager::formatAdifHeader(const std::string &programId) {
    std::string out;
    out += "ADIF Export from GNSS Compass\r\n";
    out += "<ADIF_VER:5>3.1.4\r\n";
    char progBuf[64];
    snprintf(progBuf, sizeof(progBuf), "<PROGRAMID:%zu>%s\r\n", programId.size(), programId.c_str());
    out += progBuf;
    out += "<EOH>\r\n\r\n";
    return out;
}

std::string QsoLogManager::formatAdifRow(const QsoEntry &qso) {
    int y, m, d, hh, mm, ss;
    epochToDateTime(qso.timestamp, y, m, d, hh, mm, ss);
    std::string adifBand = VoiceQsoParser::bandToAdif(qso.band);

    char rec[512];
    char dateBuf[32];
    snprintf(dateBuf, sizeof(dateBuf), "%04d%02d%02d", y, m, d);
    char timeBuf[32];
    snprintf(timeBuf, sizeof(timeBuf), "%02d%02d%02d", hh, mm, ss);
    char distBuf[32];
    snprintf(distBuf, sizeof(distBuf), "%.1f", qso.distance_km);

    std::string r;
    snprintf(rec, sizeof(rec), "<CALL:%zu>%s ", std::strlen(qso.callsign), qso.callsign);
    r += rec;
    snprintf(rec, sizeof(rec), "<QSO_DATE:%zu>%s ", std::strlen(dateBuf), dateBuf);
    r += rec;
    snprintf(rec, sizeof(rec), "<TIME_ON:%zu>%s ", std::strlen(timeBuf), timeBuf);
    r += rec;
    snprintf(rec, sizeof(rec), "<BAND:%zu>%s ", adifBand.size(), adifBand.c_str());
    r += rec;
    if (qso.freq_khz > 0) {
        char freqBuf[32];
        snprintf(freqBuf, sizeof(freqBuf), "%.3f", qso.freq_khz / 1000.0);
        snprintf(rec, sizeof(rec), "<FREQ:%zu>%s ", std::strlen(freqBuf), freqBuf);
        r += rec;
    }
    r += "<MODE:3>SSB ";
    if (qso.my_grid[0] != '\0') {
        snprintf(rec, sizeof(rec), "<MY_GRIDSQUARE:%zu>%s ", std::strlen(qso.my_grid), qso.my_grid);
        r += rec;
    }
    if (qso.his_grid[0] != '\0') {
        snprintf(rec, sizeof(rec), "<GRIDSQUARE:%zu>%s ", std::strlen(qso.his_grid), qso.his_grid);
        r += rec;
    }
    snprintf(rec, sizeof(rec), "<DISTANCE:%zu>%s ", std::strlen(distBuf), distBuf);
    r += rec;
    r += "<EOR>\r\n";
    return r;
}

std::string QsoLogManager::formatAdif(const std::string &programId) const {
    std::string out = formatAdifHeader(programId);
    for (const auto &qso : qsos_) {
        out += formatAdifRow(qso);
    }
    return out;
}

std::string QsoLogManager::formatCsvHeader() {
    return "Timestamp,Date,Time,Callsign,Band,Frequency_kHz,MyGrid,HisGrid,Distance_km,Bearing_deg\r\n";
}

std::string QsoLogManager::formatCsvRow(const QsoEntry &qso) {
    int y, m, d, hh, mm, ss;
    epochToDateTime(qso.timestamp, y, m, d, hh, mm, ss);
    char row[256];
    snprintf(row, sizeof(row), "%u,%04d-%02d-%02d,%02d:%02d:%02d,%s,%s,%u,%s,%s,%.1f,%.1f\r\n",
             qso.timestamp, y, m, d, hh, mm, ss,
             qso.callsign, qso.band, qso.freq_khz,
             qso.my_grid, qso.his_grid,
             qso.distance_km, qso.bearing_deg);
    return std::string(row);
}

std::string QsoLogManager::formatCsv() const {
    std::string out = formatCsvHeader();
    for (const auto &qso : qsos_) {
        out += formatCsvRow(qso);
    }
    return out;
}

std::string QsoLogManager::formatStats() const {
    std::string out;
    out += "========================================\r\n";
    out += "         Contest QSO Statistics         \r\n";
    out += "========================================\r\n";

    if (qsos_.empty()) {
        out += "Log is empty (0 QSOs).\r\n";
        out += "========================================\r\n";
        return out;
    }

    std::set<std::string> uniqueCalls;
    std::set<std::string> uniqueGrids;
    std::map<std::string, std::pair<size_t, double>> bandMap;
    double totalDistance = 0.0;
    float maxDist = -1.0f;
    const QsoEntry *bestDx = nullptr;

    for (const auto &qso : qsos_) {
        uniqueCalls.insert(qso.callsign);
        if (qso.his_grid[0] != '\0') {
            uniqueGrids.insert(qso.his_grid);
        }
        totalDistance += qso.distance_km;
        if (qso.distance_km > maxDist) {
            maxDist = qso.distance_km;
            bestDx = &qso;
        }
        auto &b = bandMap[qso.band];
        b.first++;
        b.second += qso.distance_km;
    }

    double avgDist = totalDistance / qsos_.size();

    char buf[256];
    snprintf(buf, sizeof(buf), "Total QSOs:        %zu\r\n", qsos_.size());
    out += buf;
    snprintf(buf, sizeof(buf), "Unique Callsigns:  %zu\r\n", uniqueCalls.size());
    out += buf;
    snprintf(buf, sizeof(buf), "Unique Grids:      %zu\r\n", uniqueGrids.size());
    out += buf;
    snprintf(buf, sizeof(buf), "Total Distance:    %.1f km\r\n", totalDistance);
    out += buf;
    snprintf(buf, sizeof(buf), "Average Distance:  %.1f km\r\n", avgDist);
    out += buf;
    if (bestDx) {
        snprintf(buf, sizeof(buf), "Best DX:           %.1f km (%s in %s on %s)\r\n",
                 bestDx->distance_km, bestDx->callsign, bestDx->his_grid, bestDx->band);
        out += buf;
    }
    out += "Band Breakdown:\r\n";
    for (const auto &pair : bandMap) {
        snprintf(buf, sizeof(buf), "  %-8s: %3zu QSOs (%.1f km)\r\n",
                 pair.first.c_str(), pair.second.first, pair.second.second);
        out += buf;
    }
    out += "========================================\r\n";
    return out;
}

std::string QsoLogManager::formatListHeader() {
    std::string out;
    out += "====================================================================================================\r\n";
    out += " #  | Date & Time UTC     | Callsign     | Band    | Frequency      | My Grid  | His Grid | Dist(km) | Bearing\r\n";
    out += "====================================================================================================\r\n";
    return out;
}

std::string QsoLogManager::formatListRow(size_t index1Based, const QsoEntry &qso) {
    int y, m, d, hh, mm, ss;
    epochToDateTime(qso.timestamp, y, m, d, hh, mm, ss);

    char freqStr[32];
    if (qso.freq_khz > 0) {
        snprintf(freqStr, sizeof(freqStr), "%10.3f MHz", qso.freq_khz / 1000.0);
    } else {
        snprintf(freqStr, sizeof(freqStr), "          ---  ");
    }

    char line[256];
    snprintf(line, sizeof(line),
             "%3zu | %04d-%02d-%02d %02d:%02d:%02d | %-12s | %-7s | %s | %-8s | %-8s | %8.1f | %7.1f\r\n",
             index1Based, y, m, d, hh, mm, ss,
             qso.callsign, qso.band, freqStr,
             qso.my_grid, qso.his_grid,
             qso.distance_km, qso.bearing_deg);
    return std::string(line);
}

std::string QsoLogManager::formatListFooter() {
    return "====================================================================================================\r\n";
}

std::string QsoLogManager::formatList() const {
    std::string out = formatListHeader();
    if (qsos_.empty()) {
        out += "  (No QSOs logged)\r\n";
    } else {
        for (size_t i = 0; i < qsos_.size(); ++i) {
            out += formatListRow(i + 1, qsos_[i]);
        }
    }
    out += formatListFooter();
    return out;
}
