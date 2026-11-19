#include "serial_cli.h"
#include "serial_cli_helpers.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include <cstdio>
#include <cstring>

using namespace cli_helpers;

void SerialCli::handleLogCommand(const char *arg, unsigned long nowMs) {
    while (arg && (*arg == ' ' || *arg == '\t')) arg++;
    if (!arg || *arg == '\0') {
        pendingQsoInput_ = true;
        pendingQsoSource_ = activeSource_;
        println("[QSO] Callsign? (e.g. 'K6MG' or 'Kilo Six Mike Golf', or 'cancel')");
        return;
    }

    if (equalsIgnoreCase(arg, "undo")) {
        handleLogUndo();
        return;
    }
    if (startsWithIgnoreCase(arg, "delete") || startsWithIgnoreCase(arg, "del")) {
        handleLogDelete(arg);
        return;
    }
    if (startsWithIgnoreCase(arg, "clear")) {
        handleLogClear(arg);
        return;
    }
    if (equalsIgnoreCase(arg, "list") || equalsIgnoreCase(arg, "show")) {
        handleLogList();
        return;
    }
    if (equalsIgnoreCase(arg, "stats")) {
        handleLogStats();
        return;
    }
    if (startsWithIgnoreCase(arg, "export")) {
        handleLogExport(arg);
        return;
    }
    if (startsWithIgnoreCase(arg, "default")) {
        handleLogDefault(arg);
        return;
    }
    if (equalsIgnoreCase(arg, "mycall") || startsWithIgnoreCase(arg, "mycall ") ||
        equalsIgnoreCase(arg, "my call") || startsWithIgnoreCase(arg, "my call ") ||
        equalsIgnoreCase(arg, "my callsign") || startsWithIgnoreCase(arg, "my callsign ") ||
        equalsIgnoreCase(arg, "callsign") || startsWithIgnoreCase(arg, "callsign ")) {
        handleLogCall(arg);
        return;
    }
    if (startsWithIgnoreCase(arg, "max") || startsWithIgnoreCase(arg, "capacity")) {
        const char *p = arg + (startsWithIgnoreCase(arg, "max") ? 3 : 8);
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) {
            size_t curMax = qsoLogManager_ ? qsoLogManager_->getMaxQsos() : DEFAULT_MAX_QSOS;
            printFormat("[QSO LOG] Current capacity: %zu QSOs (range %d-%d)\r\n", curMax, MIN_QSOS, ABS_MAX_QSOS);
        } else {
            int val = 0;
            if (!parseStrictPositiveInt(p, val)) {
                printFormat("[QSO ERROR] Invalid capacity '%s'. Expected a number (%d-%d).\r\n", p, MIN_QSOS, ABS_MAX_QSOS);
                return;
            }
            if (qsoLogManager_) {
                qsoLogManager_->setMaxQsos(static_cast<size_t>(val));
                printFormat("[QSO LOG] Maximum capacity set to %zu QSOs (saved to NVS).\r\n", qsoLogManager_->getMaxQsos());
            }
        }
        return;
    }
    if (equalsIgnoreCase(arg, "timeout") || startsWithIgnoreCase(arg, "timeout ") ||
        equalsIgnoreCase(arg, "time out") || startsWithIgnoreCase(arg, "time out ")) {
        const char *p = arg + (startsWithIgnoreCase(arg, "timeout") ? 7 : 8);
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) {
            uint16_t curSec = qsoLogManager_ ? qsoLogManager_->getCardTimeoutSec() : DEFAULT_QSO_CARD_TIMEOUT_S;
            printFormat("[QSO LOG] Current pop-up card timeout: %u s (range %d-%d s, default %d s)\r\n",
                        curSec, MIN_QSO_CARD_TIMEOUT_S, MAX_QSO_CARD_TIMEOUT_S, DEFAULT_QSO_CARD_TIMEOUT_S);
        } else if (equalsIgnoreCase(p, "reset") || equalsIgnoreCase(p, "default")) {
            if (qsoLogManager_) {
                qsoLogManager_->setCardTimeoutSec(DEFAULT_QSO_CARD_TIMEOUT_S);
            }
            printFormat("[QSO LOG] Pop-up card timeout reset to %d s (saved to NVS).\r\n",
                        DEFAULT_QSO_CARD_TIMEOUT_S);
        } else {
            int val = 0;
            if (!parseStrictPositiveInt(p, val)) {
                printFormat("[QSO ERROR] Invalid timeout '%s'. Expected seconds (%d-%d).\r\n",
                            p, MIN_QSO_CARD_TIMEOUT_S, MAX_QSO_CARD_TIMEOUT_S);
                return;
            }
            if (qsoLogManager_) {
                qsoLogManager_->setCardTimeoutSec(static_cast<uint16_t>(val));
                printFormat("[QSO LOG] Pop-up card timeout set to %u s (saved to NVS).\r\n",
                            qsoLogManager_->getCardTimeoutSec());
            }
        }
        return;
    }
    if (equalsIgnoreCase(arg, "help") || std::strcmp(arg, "?") == 0) {
        printLogHelp();
        return;
    }

    std::string defBand = qsoLogManager_ ? qsoLogManager_->getDefaultBand() : "10 GHz";
    ParsedQso parsed;
    if (!VoiceQsoParser::parseQso(arg, parsed, defBand)) {
        printFormat("[QSO ERROR] %s. Usage: log <call> [band/freq]\r\n", parsed.error_message.c_str());
        return;
    }

    CliNavContext nctx = getNavContext();
    uint32_t ts = 0;
    if (gpsManager_.hasTime() && gpsManager_.getEpochTime() > 0) {
        ts = gpsManager_.getEpochTime();
    }

    const char *myGrid = nctx.currentGrid;
    const char *hisGrid = nctx.targetGrid;

    float distKm = 0.0f;
    float bearingDeg = 0.0f;

    GeoPoint curPt, tgtPt;
    if (gridToGeoPoint(myGrid, curPt) && gridToGeoPoint(hisGrid, tgtPt)) {
        double distMeters = 0.0, brg = 0.0;
        calculateWgs84(curPt, tgtPt, distMeters, brg);
        distKm = static_cast<float>(distMeters / 1000.0);
        bearingDeg = static_cast<float>(brg);
    } else if (nctx.roverDistanceMeters > 0.0) {
        distKm = static_cast<float>(nctx.roverDistanceMeters / 1000.0);
        bearingDeg = static_cast<float>(nctx.targetBearing);
    }

    QsoEntry entry(ts, parsed.callsign.c_str(), parsed.band.c_str(), parsed.freq_khz,
                   myGrid, hisGrid, distKm, bearingDeg);
    if (qsoLogManager_) {
        qsoLogManager_->appendQso(entry);
    }
    size_t qsoIndex = qsoLogManager_ ? qsoLogManager_->getQsoCount() : 1;

    if (qsoLoggedCb_) {
        qsoLoggedCb_(entry, qsoIndex, nowMs, qsoLoggedContext_);
    }

    char timeStr[8];
    if (ts > 0) {
        int h = (ts / 3600) % 24;
        int m = (ts / 60) % 60;
        std::snprintf(timeStr, sizeof(timeStr), "%02d:%02dz", h, m);
    } else {
        std::snprintf(timeStr, sizeof(timeStr), "--:--z");
    }
    char bandStr[48];
    if (parsed.freq_khz > 0) {
        std::snprintf(bandStr, sizeof(bandStr), "%s (%.3f MHz)", parsed.band.c_str(), static_cast<double>(parsed.freq_khz) / 1000.0);
    } else {
        std::snprintf(bandStr, sizeof(bandStr), "%s", parsed.band.c_str());
    }
    printFormat("[QSO LOGGED #%zu] %s | %s | %s | %s -> %s (%.1f km @ %.0f deg)\r\n",
                qsoIndex, parsed.callsign.c_str(), bandStr, timeStr,
                (myGrid && myGrid[0]) ? myGrid : "------",
                (hisGrid && hisGrid[0]) ? hisGrid : "------",
                distKm, bearingDeg);
}

void SerialCli::handleLogUndo() {
    if (!qsoLogManager_ || qsoLogManager_->isEmpty()) {
        println("[QSO LOG] Log is empty, nothing to undo.");
        return;
    }
    QsoEntry last;
    qsoLogManager_->getLastQso(last);
    size_t lastIndex = qsoLogManager_->getQsoCount();
    qsoLogManager_->popLastQso();
    size_t remaining = qsoLogManager_->getQsoCount();
    printFormat("[QSO LOG] Removed last entry #%zu (%s on %s). Log contains %zu contacts.\r\n",
                lastIndex, last.callsign, last.band, remaining);
}

void SerialCli::handleLogDelete(const char *arg) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "delete")) {
        arg += 6;
    } else if (startsWithIgnoreCase(arg, "del")) {
        arg += 3;
    }
    while (*arg == ' ' || *arg == '\t') arg++;
    if (!*arg) {
        println("[QSO ERROR] Missing QSO number. Usage: log delete <#> (e.g. 'log delete 3')");
        return;
    }
    int idx = 0;
    if (!parsePositiveIntOrVoice(arg, idx) || !qsoLogManager_ || static_cast<size_t>(idx) > qsoLogManager_->getQsoCount()) {
        size_t count = qsoLogManager_ ? qsoLogManager_->getQsoCount() : 0;
        printFormat("[QSO ERROR] Invalid QSO index '%s'. Log contains %zu contacts.\r\n", arg, count);
        return;
    }
    QsoEntry entry;
    qsoLogManager_->getQso(static_cast<size_t>(idx - 1), entry);
    qsoLogManager_->deleteQsoByIndex(static_cast<size_t>(idx));
    size_t remaining = qsoLogManager_->getQsoCount();
    printFormat("[QSO LOG] Deleted QSO #%d (%s on %s). Entries re-indexed (%zu contacts remain).\r\n",
                idx, entry.callsign, entry.band, remaining);
}

void SerialCli::handleLogClear(const char *arg) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "clear")) {
        arg += 5;
    }
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "confirm") || startsWithIgnoreCase(arg, "yes") ||
        startsWithIgnoreCase(arg, "-y") || startsWithIgnoreCase(arg, "force")) {
        pendingQsoClear_ = false;
        doLogClear();
        return;
    }
    if (startsWithIgnoreCase(arg, "cancel") || startsWithIgnoreCase(arg, "no") ||
        startsWithIgnoreCase(arg, "-n")) {
        pendingQsoClear_ = false;
        println("[QSO LOG] QSO clear aborted.");
        return;
    }

    pendingQsoClear_ = true;
    pendingQsoClearSource_ = activeSource_;
    println("[QSO LOG] WARNING: This will permanently erase all contest QSO history!");
    println("[QSO LOG] Type 'log clear confirm' (or 'yes') to proceed, or 'no' to abort.");
}

void SerialCli::doLogClear() {
    if (qsoLogManager_) {
        qsoLogManager_->clearLog();
    }
    println("[QSO LOG] Contest QSO log cleared (0 contacts).");
}

void SerialCli::startStreamingExport(ExportFormat fmt) {
    if (!qsoLogManager_) return;

    pendingExportFormat_ = fmt;
    pendingExportIndex_ = 0;
    pendingExportSource_ = activeSource_;
    pendingExportCall_ = qsoLogManager_->getEffectiveMyCall();

    switch (fmt) {
        case ExportFormat::List:
            print(QsoLogManager::formatListHeader().c_str());
            break;
        case ExportFormat::Cabrillo:
            print(QsoLogManager::formatCabrilloHeader(pendingExportCall_).c_str());
            break;
        case ExportFormat::Adif:
            print(QsoLogManager::formatAdifHeader().c_str());
            break;
        case ExportFormat::Csv:
            print(QsoLogManager::formatCsvHeader().c_str());
            break;
        default:
            pendingExportFormat_ = ExportFormat::None;
            return;
    }

    if (qsoLogManager_->isEmpty()) {
        if (fmt == ExportFormat::List) {
            print("  (No QSOs logged)\r\n");
            print(QsoLogManager::formatListFooter().c_str());
        } else if (fmt == ExportFormat::Cabrillo) {
            print(QsoLogManager::formatCabrilloFooter().c_str());
        }
        pendingExportFormat_ = ExportFormat::None;
        return;
    }

    servicePendingExport();
}

void SerialCli::servicePendingExport(bool fromUpdate) {
    if (pendingExportFormat_ == ExportFormat::None) {
        return;
    }
    if (pendingExportFormat_ == ExportFormat::TargetList) {
        if (!targetPresetManager_) {
            pendingExportFormat_ = ExportFormat::None;
            return;
        }
    } else if (!qsoLogManager_) {
        pendingExportFormat_ = ExportFormat::None;
        return;
    }

    CliSource prevSource = activeSource_;
    activeSource_ = pendingExportSource_;

    const bool throttledBle = (pendingExportSource_ == CLI_SOURCE_BLE && bleManager_ != nullptr);
    if (throttledBle && (!bleManager_->isConnected() || !bleManager_->isSubscribed())) {
        pendingExportFormat_ = ExportFormat::None;
        activeSource_ = prevSource;
        return;
    }

    const size_t total = (pendingExportFormat_ == ExportFormat::TargetList)
                             ? targetPresetManager_->getCount()
                             : qsoLogManager_->getQsoCount();
    while (pendingExportIndex_ < total) {
        if (throttledBle) {
            size_t avail = bleManager_->getTxCapacity() - bleManager_->getTxQueued();
            if (avail < 512) {
                activeSource_ = prevSource;
                return;
            }
        }

        if (pendingExportFormat_ == ExportFormat::TargetList) {
            const TargetPreset *p = targetPresetManager_->getPreset(pendingExportIndex_);
            if (p) {
                const char *lbl = (p->label[0] != '\0') ? p->label : "(none)";
                printFormat("  #%zu: %s - %-16s (uses: %u, slot: t%zu)\r\n",
                            pendingExportIndex_ + 1, p->grid, lbl,
                            static_cast<unsigned>(p->useCount), pendingExportIndex_);
            }
        } else {
            const QsoEntry *qso = qsoLogManager_->getQso(pendingExportIndex_);
            if (qso) {
                std::string row;
                switch (pendingExportFormat_) {
                    case ExportFormat::List:
                        row = QsoLogManager::formatListRow(pendingExportIndex_ + 1, *qso);
                        break;
                    case ExportFormat::Cabrillo:
                        row = QsoLogManager::formatCabrilloRow(*qso, pendingExportCall_);
                        break;
                    case ExportFormat::Adif:
                        row = QsoLogManager::formatAdifRow(*qso);
                        break;
                    case ExportFormat::Csv:
                        row = QsoLogManager::formatCsvRow(*qso);
                        break;
                    default:
                        break;
                }
                if (!row.empty()) {
                    print(row.c_str());
                }
            }
        }
        pendingExportIndex_++;
    }

    if (throttledBle) {
        size_t avail = bleManager_->getTxCapacity() - bleManager_->getTxQueued();
        if (avail < 128) {
            activeSource_ = prevSource;
            return;
        }
    }

    if (pendingExportFormat_ == ExportFormat::List) {
        print(QsoLogManager::formatListFooter().c_str());
    } else if (pendingExportFormat_ == ExportFormat::Cabrillo) {
        print(QsoLogManager::formatCabrilloFooter().c_str());
    } else if (pendingExportFormat_ == ExportFormat::TargetList) {
        println("===================================");
    }

    pendingExportFormat_ = ExportFormat::None;
    if (fromUpdate) {
        printPrompt();
    }
    activeSource_ = prevSource;
}

void SerialCli::handleLogList() {
    if (!qsoLogManager_ || qsoLogManager_->isEmpty()) {
        println("[QSO LOG] Log is empty (0 contacts).");
        return;
    }
    startStreamingExport(ExportFormat::List);
}

void SerialCli::handleLogStats() {
    if (!qsoLogManager_) {
        println("[QSO LOG] Log manager unavailable.");
        return;
    }
    std::string statsStr = qsoLogManager_->formatStats();
    print(statsStr.c_str());
}

void SerialCli::handleLogExport(const char *arg) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "export")) {
        arg += 6;
    }
    while (*arg == ' ' || *arg == '\t') arg++;
    if (!*arg) {
        println("[QSO ERROR] Missing export format. Usage: log export <cabrillo|adif|csv>");
        return;
    }
    if (!qsoLogManager_) {
        println("[QSO ERROR] Log manager unavailable.");
        return;
    }
    if (startsWithIgnoreCase(arg, "cabrillo") || startsWithIgnoreCase(arg, "cab")) {
        startStreamingExport(ExportFormat::Cabrillo);
    } else if (startsWithIgnoreCase(arg, "adif") || startsWithIgnoreCase(arg, "adi")) {
        startStreamingExport(ExportFormat::Adif);
    } else if (startsWithIgnoreCase(arg, "csv")) {
        startStreamingExport(ExportFormat::Csv);
    } else {
        printFormat("[QSO ERROR] Unknown export format '%s'. Usage: log export <cabrillo|adif|csv>\r\n", arg);
    }
}

void SerialCli::handleLogDefault(const char *arg) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "default")) {
        arg += 7;
    }
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "band")) {
        arg += 4;
    }
    while (*arg == ' ' || *arg == '\t') arg++;
    if (!*arg) {
        std::string cur = qsoLogManager_ ? qsoLogManager_->getDefaultBand() : "10 GHz";
        printFormat("[QSO LOG] Current default band: %s\r\n", cur.c_str());
        println("[QSO LOG] Usage: log default band <band> (e.g. 'log default band 24' or 'log default band 10 GHz')");
        return;
    }
    std::string bnd, cab, adif;
    uint32_t freqKhz = 0;
    if (!VoiceQsoParser::parseBandOrFreq(arg, bnd, freqKhz, cab, adif)) {
        printFormat("[QSO ERROR] Unrecognized band '%s'.\r\n", arg);
        return;
    }
    if (qsoLogManager_) {
        qsoLogManager_->setDefaultBand(bnd);
    }
    printFormat("[QSO LOG] Default band set to %s.\r\n", bnd.c_str());
}

void SerialCli::handleLogCall(const char *arg) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "my callsign")) {
        arg += 11;
    } else if (startsWithIgnoreCase(arg, "callsign")) {
        arg += 8;
    } else if (startsWithIgnoreCase(arg, "my call")) {
        arg += 7;
    } else if (startsWithIgnoreCase(arg, "mycall")) {
        arg += 6;
    }
    while (*arg == ' ' || *arg == '\t') arg++;

    if (!*arg) {
        std::string cur = qsoLogManager_ ? qsoLogManager_->getMyCall() : DEFAULT_QSO_MY_CALL;
        printFormat("[QSO LOG] Current station callsign: %s\r\n", cur.c_str());
        println("[QSO LOG] Usage: log mycall <callsign|reset> (e.g. 'log mycall K6MG/R' or 'log my call kilo six mike golf')");
        return;
    }

    if (equalsIgnoreCase(arg, "reset") || equalsIgnoreCase(arg, "default") || equalsIgnoreCase(arg, "clear")) {
        if (qsoLogManager_) {
            qsoLogManager_->setMyCall(DEFAULT_QSO_MY_CALL);
        }
        printFormat("[QSO LOG] Station callsign reset to %s.\r\n", DEFAULT_QSO_MY_CALL);
        return;
    }

    std::string normalized = VoiceQsoParser::normalizeVoiceInput(arg);
    if (normalized.empty() || normalized.size() > 15) {
        printFormat("[QSO ERROR] Invalid station callsign '%s'.\r\n", arg);
        return;
    }

    if (qsoLogManager_) {
        qsoLogManager_->setMyCall(normalized);
    }
    printFormat("[QSO LOG] Station callsign set to %s (saved to NVS).\r\n", normalized.c_str());
}

void SerialCli::printLogHelp() {
    println();
    println("=== Contest QSO Logging Commands ===");
    println("  log <call> [band/freq]   - Immediately log QSO and display pop-up card");
    println("                             (e.g. 'log k6mg 10', 'log 24 k6mg', 'log k6mg 10368.1',");
    println("                              or phonetic: 'log kilo six mike golf')");
    println("  log undo                 - Remove the most recent QSO entry");
    println("  log delete <#> (or del)  - Delete specific QSO entry by number");
    println("  log clear [confirm]      - Erase entire contest QSO history");
    println("  log list (or show)       - Display formatted ASCII table of logged QSOs");
    println("  log stats                - Show contest statistics (QSOs, DX, bands, grids)");
    println("  log export <cab|adif|csv>- Stream contest log export");
    println("  log mycall [<call>|reset]- Configure station callsign for Cabrillo export");
    println("  log default band <band>  - Configure default band in NVS (default: 10 GHz)");
    println("  log max <#>              - Configure maximum QSO storage capacity (10-1000)");
    println("  log timeout [<sec>|reset]- Configure QSO pop-up card timeout (1-60 s, default: 5 s)");
}
