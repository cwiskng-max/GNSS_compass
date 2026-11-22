#include "qso_log_manager.h"
#include "../protocol/voice_qso_parser.h"
#include <algorithm>
#include <cstdio>
#include <ctime>
#include <map>
#include <set>

namespace {

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

void epochToDateTime(uint32_t ts, int &year, int &month, int &day, int &hour,
                     int &minute, int &second) {
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

QsoLogManager::NvsQsoRecord QsoLogManager::makeNvsRecord(const QsoEntry &qso) {
  NvsQsoRecord rec;
  rec.magic = kNvsQsoMagic;
  rec.version = kNvsQsoVersion;
  rec.entry = qso;
  rec.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&rec.entry),
                         sizeof(QsoEntry));
  return rec;
}

bool QsoLogManager::validateNvsRecord(const NvsQsoRecord &rec) {
  if (rec.magic != kNvsQsoMagic || rec.version != kNvsQsoVersion) {
    return false;
  }
  uint16_t expected = crc16Ccitt(reinterpret_cast<const uint8_t *>(&rec.entry),
                                 sizeof(QsoEntry));
  return rec.crc16 == expected;
}

QsoLogManager::QsoLogManager()
    : maxQsos_(DEFAULT_MAX_QSOS), nvsHead_(0), defaultBand_(DEFAULT_QSO_BAND),
      myCall_(DEFAULT_QSO_MY_CALL), cardTimeoutSec_(DEFAULT_QSO_CARD_TIMEOUT_S),
      cfgStore_(QSO_CFG_NAMESPACE), logCfgStore_(QSO_LOG_NAMESPACE),
      listStore_(QSO_LOG_NAMESPACE, QSO_LOG_KEY_COUNT, 'q') {}

void QsoLogManager::begin() { loadFromNvs(); }

size_t QsoLogManager::clampMaxQsos(size_t val) {
  if (val < MIN_QSOS)
    return MIN_QSOS;
  if (val > ABS_MAX_QSOS)
    return ABS_MAX_QSOS;
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
    cfgStore_.putUShort(QSO_CFG_KEY_MAX_QSOS, static_cast<uint16_t>(maxQsos_));
  }

  if (truncated || (maxQsos_ != oldMax && nvsHead_ != 0)) {
    saveToNvs();
  }
}

void QsoLogManager::setDefaultBand(const std::string &band, bool persist) {
  if (band.empty())
    return;
  defaultBand_ = band;

  if (persist) {
    cfgStore_.putString(QSO_CFG_KEY_DEFAULT_BAND, defaultBand_);
  }
}

void QsoLogManager::setMyCall(const std::string &call, bool persist) {
  std::string clean = call.empty() ? DEFAULT_QSO_MY_CALL : call;
  if (clean.size() > 15) {
    clean.resize(15);
  }
  myCall_ = clean;

  if (persist) {
    cfgStore_.putString(QSO_CFG_KEY_MY_CALL, myCall_);
  }
}

uint16_t QsoLogManager::clampCardTimeoutSec(uint16_t sec) {
  if (sec < MIN_QSO_CARD_TIMEOUT_S)
    return MIN_QSO_CARD_TIMEOUT_S;
  if (sec > MAX_QSO_CARD_TIMEOUT_S)
    return MAX_QSO_CARD_TIMEOUT_S;
  return sec;
}

void QsoLogManager::setCardTimeoutSec(uint16_t sec, bool persist) {
  cardTimeoutSec_ = clampCardTimeoutSec(sec);

  if (persist) {
    cfgStore_.putUShort(QSO_CFG_KEY_CARD_TIMEOUT_S, cardTimeoutSec_);
  }
}

bool QsoLogManager::appendQso(const QsoEntry &qso) {
  if (maxQsos_ == 0) {
    maxQsos_ = DEFAULT_MAX_QSOS;
  }

  if (qsos_.size() >= maxQsos_) {
    // Discard oldest QSO to preserve ring buffer capacity (O(1) FIFO rollover
    // in NVS)
    qsos_.erase(qsos_.begin());
    qsos_.push_back(qso);
    size_t slot = nvsHead_ % maxQsos_;
    NvsQsoRecord rec = makeNvsRecord(qsos_.back());
    if (listStore_.putSlot(slot, rec)) {
      nvsHead_ = (slot + 1) % maxQsos_;
      logCfgStore_.putUShort(kNvsKeyHead, static_cast<uint16_t>(nvsHead_));
      listStore_.putCount(qsos_.size());
    }
  } else {
    qsos_.push_back(qso);
    size_t slot = (nvsHead_ + qsos_.size() - 1) % maxQsos_;
    NvsQsoRecord rec = makeNvsRecord(qsos_.back());
    listStore_.putSlotAndCount(slot, rec, qsos_.size());
  }
  return true;
}

bool QsoLogManager::popLastQso() {
  if (qsos_.empty()) {
    return false;
  }

  qsos_.pop_back();
  size_t removedSlot = (nvsHead_ + qsos_.size()) % maxQsos_;
  listStore_.removeSlotAndCount(removedSlot, qsos_.size());
  return true;
}

bool QsoLogManager::deleteQsoByIndex(size_t index) {
  if (index < 1 || index > qsos_.size()) {
    return false;
  }

  size_t vectorIndex = index - 1;
  qsos_.erase(qsos_.begin() + static_cast<std::ptrdiff_t>(vectorIndex));

  if (vectorIndex == 0) {
    // O(1) delete of oldest entry by advancing ring head
    size_t oldHeadSlot = nvsHead_ % maxQsos_;
    nvsHead_ = (nvsHead_ + 1) % maxQsos_;
    listStore_.removeSlotAndCount(oldHeadSlot, qsos_.size());
    logCfgStore_.putUShort(kNvsKeyHead, static_cast<uint16_t>(nvsHead_));
  } else {
    // Shift only trailing elements [vectorIndex .. size-1]
    for (size_t i = vectorIndex; i < qsos_.size(); ++i) {
      size_t slot = (nvsHead_ + i) % maxQsos_;
      NvsQsoRecord rec = makeNvsRecord(qsos_[i]);
      listStore_.putSlot(slot, rec);
    }
    size_t tailSlot = (nvsHead_ + qsos_.size()) % maxQsos_;
    listStore_.removeSlotAndCount(tailSlot, qsos_.size());
  }

  return true;
}

void QsoLogManager::clearLog() {
  qsos_.clear();
  nvsHead_ = 0;
  listStore_.clear();
  logCfgStore_.putUShort(kNvsKeyHead, 0);
  listStore_.putCount(0);
}

void QsoLogManager::resetDefaults() {
  clearLog();
  maxQsos_ = DEFAULT_MAX_QSOS;
  defaultBand_ = DEFAULT_QSO_BAND;
  myCall_ = DEFAULT_QSO_MY_CALL;
  cardTimeoutSec_ = DEFAULT_QSO_CARD_TIMEOUT_S;
  cfgStore_.remove(QSO_CFG_KEY_MAX_QSOS);
  cfgStore_.remove(QSO_CFG_KEY_DEFAULT_BAND);
  cfgStore_.remove(QSO_CFG_KEY_MY_CALL);
  cfgStore_.remove(QSO_CFG_KEY_CARD_TIMEOUT_S);
}

const QsoEntry *QsoLogManager::getQso(size_t index) const {
  if (index >= qsos_.size())
    return nullptr;
  return &qsos_[index];
}

bool QsoLogManager::getQso(size_t index, QsoEntry &out) const {
  if (index >= qsos_.size())
    return false;
  out = qsos_[index];
  return true;
}

const QsoEntry *QsoLogManager::getLastQso() const {
  if (qsos_.empty())
    return nullptr;
  return &qsos_.back();
}

bool QsoLogManager::getLastQso(QsoEntry &out) const {
  if (qsos_.empty())
    return false;
  out = qsos_.back();
  return true;
}

void QsoLogManager::loadFromNvs() {
  uint16_t storedMax =
      cfgStore_.getUShort(QSO_CFG_KEY_MAX_QSOS, DEFAULT_MAX_QSOS);
  maxQsos_ = clampMaxQsos(storedMax);

  std::string defB =
      cfgStore_.getString(QSO_CFG_KEY_DEFAULT_BAND, DEFAULT_QSO_BAND);
  defaultBand_ = defB.empty() ? DEFAULT_QSO_BAND : defB;

  std::string storedCall =
      cfgStore_.getString(QSO_CFG_KEY_MY_CALL, DEFAULT_QSO_MY_CALL);
  myCall_ = storedCall.empty() ? DEFAULT_QSO_MY_CALL : storedCall;

  uint16_t storedTimeout = cfgStore_.getUShort(QSO_CFG_KEY_CARD_TIMEOUT_S,
                                               DEFAULT_QSO_CARD_TIMEOUT_S);
  cardTimeoutSec_ = clampCardTimeoutSec(storedTimeout);

  qsos_.clear();
  nvsHead_ = 0;
  uint16_t count = static_cast<uint16_t>(listStore_.getCount());
  uint16_t head = logCfgStore_.getUShort(kNvsKeyHead, 0);
  if (maxQsos_ > 0) {
    nvsHead_ = head % maxQsos_;
  }
  if (count > maxQsos_) {
    count = static_cast<uint16_t>(maxQsos_);
  }
  qsos_.reserve(count);
  for (uint16_t i = 0; i < count; ++i) {
    size_t slot = (nvsHead_ + i) % maxQsos_;
    uint8_t rawBuf[sizeof(NvsQsoRecord)];
    size_t bytesRead = 0;
    if (!listStore_.getRawSlot(slot, rawBuf, sizeof(rawBuf), bytesRead)) {
      continue;
    }
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
}

void QsoLogManager::saveToNvs() {
  nvsHead_ = 0;
  std::vector<NvsQsoRecord> records;
  records.reserve(qsos_.size());
  for (size_t i = 0; i < qsos_.size(); ++i) {
    records.push_back(makeNvsRecord(qsos_[i]));
  }
  listStore_.save(records);
  logCfgStore_.putUShort(kNvsKeyHead, 0);
}

#if !defined(ARDUINO)
void QsoLogManager::resetHostStorage() {
  NvsConfig::resetHostStorage(QSO_CFG_NAMESPACE);
  NvsConfig::resetHostStorage(QSO_LOG_NAMESPACE);
  NvsListStore<NvsQsoRecord>::resetHostStorage(QSO_LOG_NAMESPACE);
}

size_t QsoLogManager::getHostStorageCount() {
  return NvsListStore<NvsQsoRecord>(QSO_LOG_NAMESPACE, QSO_LOG_KEY_COUNT, 'q')
      .getHostCount();
}

size_t QsoLogManager::getHostStorageMaxQsos() {
  return NvsConfig(QSO_CFG_NAMESPACE)
      .getUShort(QSO_CFG_KEY_MAX_QSOS, DEFAULT_MAX_QSOS);
}

std::string QsoLogManager::getHostStorageDefaultBand() {
  return NvsConfig(QSO_CFG_NAMESPACE)
      .getString(QSO_CFG_KEY_DEFAULT_BAND, DEFAULT_QSO_BAND);
}

std::string QsoLogManager::getHostStorageMyCall() {
  return NvsConfig(QSO_CFG_NAMESPACE)
      .getString(QSO_CFG_KEY_MY_CALL, DEFAULT_QSO_MY_CALL);
}

uint16_t QsoLogManager::getHostStorageCardTimeoutSec() {
  return NvsConfig(QSO_CFG_NAMESPACE)
      .getUShort(QSO_CFG_KEY_CARD_TIMEOUT_S, DEFAULT_QSO_CARD_TIMEOUT_S);
}

void QsoLogManager::seedHostRawSlot(size_t slot, const void *data, size_t len,
                                    size_t count, size_t head) {
  NvsListStore<NvsQsoRecord> store(QSO_LOG_NAMESPACE, QSO_LOG_KEY_COUNT, 'q');
  store.putHostRawSlot(slot, data, len);
  store.putCount(count);
  NvsConfig(QSO_LOG_NAMESPACE)
      .putUShort(kNvsKeyHead, static_cast<uint16_t>(head));
}
#endif

std::string QsoLogManager::getEffectiveMyCall(const std::string &myCall) const {
  return myCall.empty()
             ? (myCall_.empty() ? std::string(DEFAULT_QSO_MY_CALL) : myCall_)
             : myCall;
}

std::string
QsoLogManager::formatCabrilloHeader(const std::string &effectiveCall,
                                    const std::string &contest) {
  std::string out;
  out += "START-OF-LOG: 3.0\r\n";
  out += "CREATED-BY: GNSS Compass\r\n";
  out += "CONTEST: " + (contest.empty() ? "ARRL-10-GHZ" : contest) + "\r\n";
  out += "CALLSIGN: " + effectiveCall + "\r\n";
  return out;
}

std::string QsoLogManager::formatCabrilloRow(const QsoEntry &qso,
                                             const std::string &effectiveCall) {
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
  snprintf(line, sizeof(line),
           "QSO: %-5s PH %04d-%02d-%02d %02d%02d %-10s %-6s %-10s %-6s\r\n",
           cabBand.c_str(), y, m, d, hh, mm, effectiveCall.c_str(), myGrid6,
           qso.callsign, hisGrid6);
  return std::string(line);
}

std::string QsoLogManager::formatCabrilloFooter() { return "END-OF-LOG:\r\n"; }

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
  snprintf(progBuf, sizeof(progBuf), "<PROGRAMID:%zu>%s\r\n", programId.size(),
           programId.c_str());
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
  snprintf(rec, sizeof(rec), "<CALL:%zu>%s ", std::strlen(qso.callsign),
           qso.callsign);
  r += rec;
  snprintf(rec, sizeof(rec), "<QSO_DATE:%zu>%s ", std::strlen(dateBuf),
           dateBuf);
  r += rec;
  snprintf(rec, sizeof(rec), "<TIME_ON:%zu>%s ", std::strlen(timeBuf), timeBuf);
  r += rec;
  snprintf(rec, sizeof(rec), "<BAND:%zu>%s ", adifBand.size(),
           adifBand.c_str());
  r += rec;
  if (qso.freq_khz > 0) {
    char freqBuf[32];
    snprintf(freqBuf, sizeof(freqBuf), "%.3f", qso.freq_khz / 1000.0);
    snprintf(rec, sizeof(rec), "<FREQ:%zu>%s ", std::strlen(freqBuf), freqBuf);
    r += rec;
  }
  r += "<MODE:3>SSB ";
  if (qso.my_grid[0] != '\0') {
    snprintf(rec, sizeof(rec), "<MY_GRIDSQUARE:%zu>%s ",
             std::strlen(qso.my_grid), qso.my_grid);
    r += rec;
  }
  if (qso.his_grid[0] != '\0') {
    snprintf(rec, sizeof(rec), "<GRIDSQUARE:%zu>%s ", std::strlen(qso.his_grid),
             qso.his_grid);
    r += rec;
  }
  snprintf(rec, sizeof(rec), "<DISTANCE:%zu>%s ", std::strlen(distBuf),
           distBuf);
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
  return "Timestamp,Date,Time,Callsign,Band,Frequency_kHz,MyGrid,HisGrid,"
         "Distance_km,Bearing_deg\r\n";
}

std::string QsoLogManager::formatCsvRow(const QsoEntry &qso) {
  int y, m, d, hh, mm, ss;
  epochToDateTime(qso.timestamp, y, m, d, hh, mm, ss);
  char row[256];
  snprintf(row, sizeof(row),
           "%u,%04d-%02d-%02d,%02d:%02d:%02d,%s,%s,%u,%s,%s,%.1f,%.1f\r\n",
           qso.timestamp, y, m, d, hh, mm, ss, qso.callsign, qso.band,
           qso.freq_khz, qso.my_grid, qso.his_grid, qso.distance_km,
           qso.bearing_deg);
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
    snprintf(
        buf, sizeof(buf), "Best DX:           %.1f km (%s in %s on %s)\r\n",
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
  out += "====================================================================="
         "===============================\r\n";
  out += " #  | Date & Time UTC     | Callsign     | Band    | Frequency      "
         "| My Grid  | His Grid | Dist(km) | Bearing\r\n";
  out += "====================================================================="
         "===============================\r\n";
  return out;
}

std::string QsoLogManager::formatListRow(size_t index1Based,
                                         const QsoEntry &qso) {
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
           "%3zu | %04d-%02d-%02d %02d:%02d:%02d | %-12s | %-7s | %s | %-8s | "
           "%-8s | %8.1f | %7.1f\r\n",
           index1Based, y, m, d, hh, mm, ss, qso.callsign, qso.band, freqStr,
           qso.my_grid, qso.his_grid, qso.distance_km, qso.bearing_deg);
  return std::string(line);
}

std::string QsoLogManager::formatListFooter() {
  return "====================================================================="
         "===============================\r\n";
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
