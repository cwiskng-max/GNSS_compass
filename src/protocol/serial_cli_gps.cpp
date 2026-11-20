#include "serial_cli.h"
#include "serial_cli_helpers.h"
#include "../geo/maidenhead.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

using namespace cli_helpers;

static void extractField(const char *src, int targetIdx, char *dst, size_t dstSize) {
    if (!src || !dst || dstSize == 0) return;
    dst[0] = '\0';
    int idx = 0;
    const char *p = src;
    while (*p && idx < targetIdx) {
        if (*p == ',') idx++;
        p++;
    }
    if (idx != targetIdx) return;
    size_t i = 0;
    while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n' && (i + 1) < dstSize) {
        dst[i++] = *p++;
    }
    dst[i] = '\0';
}

void SerialCli::printGpsHelp() {
    println();
    println("=== GPS Receiver Commands ===");
    println("  gps status            - Live GNSS receiver stream & link diagnostic");
    println("  gps probe             - Test bidirectional TX connection to UM982");
    println("  gps config [baseline <cm> [tol]]");
    println("                        - Run automated 5 Hz $KSXT receiver setup");
    println("  gps send <command>    - Send raw command to UM982 receiver");
    println("  gps check (or audit)  - Query receiver running parameters");
}

void SerialCli::handleGpsCommand(const char *arg, unsigned long nowMs) {
    if (equalsIgnoreCase(arg, "help") || std::strcmp(arg, "?") == 0) {
        printGpsHelp();
        return;
    }

    if (equalsIgnoreCase(arg, "probe")) {
        println("[GPS PROBE] Transmitting 'version' discovery probe on GPIO 43...");
        gpsManager_.probeTx(nowMs);
        return;
    }

    if (startsWithIgnoreCase(arg, "config")) {
        if (gpsManager_.getConfigPipeline().isRunning()) {
            println("[GPS CONFIG] Configuration pipeline is already running.");
            return;
        }

        const char *sub = arg + 6;
        while (*sub == ' ' || *sub == '\t') sub++;

        float baselineCm = 0.0f;
        float tolCm = 2.5f;

        if (startsWithIgnoreCase(sub, "baseline")) {
            sub += 8;
            while (*sub == ' ' || *sub == '\t') sub++;
            const char *next = nullptr;
            if (!parseFloatToken(sub, baselineCm, next) || baselineCm <= 0.0f) {
                println("[GPS ERROR] Usage: gps config [baseline <cm> [tol]]");
                return;
            }
            while (*next == ' ' || *next == '\t') next++;
            if (*next != '\0') {
                float t = 0.0f;
                if (!parseStrictFloat(next, t) || t <= 0.0f) {
                    println("[GPS ERROR] Invalid baseline tolerance. Usage: gps config [baseline <cm> [tol]]");
                    return;
                }
                tolCm = t;
            }
        } else if (*sub != '\0') {
            const char *next = nullptr;
            if (!parseFloatToken(sub, baselineCm, next) || baselineCm <= 0.0f) {
                println("[GPS ERROR] Usage: gps config [baseline <cm> [tol]]");
                return;
            }
            while (*next == ' ' || *next == '\t') next++;
            if (*next != '\0') {
                float t = 0.0f;
                if (!parseStrictFloat(next, t) || t <= 0.0f) {
                    println("[GPS ERROR] Invalid baseline tolerance. Usage: gps config [baseline <cm> [tol]]");
                    return;
                }
                tolCm = t;
            }
        }

        if (!gpsManager_.isSendAvailable()) {
            if (gpsManager_.getTxState() == GpsTxState::Unknown) {
                println("[GPS CONFIG] Testing TX connection on GPIO 43 before configuration...");
                pendingConfig_ = true;
                pendingConfigSource_ = activeSource_;
                pendingConfigBaselineCm_ = baselineCm;
                pendingConfigTolCm_ = tolCm;
                gpsManager_.probeTx(nowMs);
                return;
            }
            println("[GPS] TX wire not detected. Connect GPIO 43 to UM982 RX to configure.");
            return;
        }

        if (baselineCm > 0.0f) {
            printFormat("[GPS CONFIG] Initiating configuration pipeline (Baseline: %.1f cm, Tol: %.1f cm)...\r\n", baselineCm, tolCm);
        } else {
            println("[GPS CONFIG] Initiating configuration pipeline (Standard 5 Hz $KSXT)...");
        }
        gpsManager_.startConfiguration(baselineCm, tolCm, true, nowMs);
        return;
    }

    if (startsWithIgnoreCase(arg, "send")) {
        if (!gpsManager_.isSendAvailable()) {
            println("[GPS] Cannot send: TX wire not connected.");
            return;
        }
        const char *cmd = arg + 4;
        while (*cmd == ' ' || *cmd == '\t') cmd++;
        if (*cmd == '\0') {
            println("[GPS ERROR] Usage: gps send <command> (e.g. 'gps send version', 'gps send uniloglist com1')");
            return;
        }
        gpsManager_.sendCommand(cmd);
        printFormat("[GPS TX] Sent: '%s'\r\n", cmd);
        return;
    }

    if (equalsIgnoreCase(arg, "check") || equalsIgnoreCase(arg, "audit")) {
        if (auditState_.inProgress) {
            println("[GPS AUDIT] Audit already in progress. Please wait...");
            return;
        }
        if (!gpsManager_.isSendAvailable()) {
            if (gpsManager_.getTxState() == GpsTxState::Unknown) {
                println("[GPS AUDIT] Testing TX connection on GPIO 43 before query...");
                pendingAudit_ = true;
                pendingAuditSource_ = activeSource_;
                gpsManager_.probeTx(nowMs);
                return;
            }
            println("[GPS] Cannot audit: TX wire not connected (Receive-only mode).");
            return;
        }
        startGpsAudit(nowMs);
        return;
    }

    println("[GPS ERROR] Unknown gps subcommand.");
    printGpsHelp();
}

void SerialCli::startGpsAudit(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    auditState_.reset(nowMs, activeSource_);
    println("[GPS AUDIT] Querying receiver parameters (VERSION, MODE, UNILOGLIST COM1, HEADINGSTATUSA)...");
    gpsManager_.sendCommand("VERSION");
    gpsManager_.sendCommand("MODE");
    gpsManager_.sendCommand("UNILOGLIST COM1");
    gpsManager_.sendCommand("LOG COM1 HEADINGSTATUSA ONCE");
}

bool SerialCli::handleAuditResponse(const char *resp) {
    if (!auditState_.inProgress || !resp) return false;

    unsigned long nowMs = 0;
#if defined(ARDUINO)
    nowMs = millis();
#endif
    if (nowMs > 0) {
        auditState_.lastRespTimeMs = nowMs;
    }

    // Suppress command execution echoes (e.g. "$command,version,response: OK*76")
    if (startsWithIgnoreCase(resp, "$command,") || startsWithIgnoreCase(resp, "$command ")) {
        return true;
    }

    // Report error responses
    if (std::strstr(resp, "ERROR") != nullptr || std::strstr(resp, "error") != nullptr) {
        printFormat("[GPS AUDIT ERROR] %s\r\n", resp);
        return true;
    }

    // Parse #VERSION
    if (startsWithIgnoreCase(resp, "#VERSION")) {
        const char *semi = std::strchr(resp, ';');
        if (semi) {
            const char *payload = semi + 1;
            extractField(payload, 0, auditState_.model, sizeof(auditState_.model));
            extractField(payload, 1, auditState_.fwVersion, sizeof(auditState_.fwVersion));
            extractField(payload, 2, auditState_.hwPn, sizeof(auditState_.hwPn));
            extractField(payload, 3, auditState_.serialNum, sizeof(auditState_.serialNum));
            extractField(payload, 5, auditState_.buildDate, sizeof(auditState_.buildDate));
        }
        auditState_.receivedVersion = true;
        if (auditState_.isComplete()) {
            renderAuditReport();
        }
        return true;
    }

    // Parse #MODE
    if (startsWithIgnoreCase(resp, "#MODE")) {
        const char *semi = std::strchr(resp, ';');
        if (semi) {
            const char *payload = semi + 1;
            while (*payload == ' ' || *payload == '\t') payload++;
            if (startsWithIgnoreCase(payload, "MODE ")) {
                payload += 5;
            }
            extractField(payload, 0, auditState_.mode, sizeof(auditState_.mode));
        }
        auditState_.receivedMode = true;
        if (auditState_.isComplete()) {
            renderAuditReport();
        }
        return true;
    }

    // Parse #UNILOGLIST
    if (startsWithIgnoreCase(resp, "#UNILOGLIST")) {
        return true;
    }

    // Parse UNILOGLIST log entries starting with '<'
    if (resp[0] == '<') {
        const char *p = resp + 1;
        while (*p == ' ' || *p == '\t') p++;
        bool isDigitsOnly = true;
        bool hasDigits = false;
        const char *temp = p;
        while (*temp && *temp != '\r' && *temp != '\n') {
            if (*temp >= '0' && *temp <= '9') {
                hasDigits = true;
            } else if (*temp != ' ' && *temp != '\t') {
                isDigitsOnly = false;
                break;
            }
            temp++;
        }
        if (isDigitsOnly && hasDigits) {
            auditState_.expectedLogs = std::atoi(p);
            if (auditState_.expectedLogs == 0) {
                auditState_.receivedUniloglist = true;
                if (auditState_.isComplete()) {
                    renderAuditReport();
                }
            }
            return true;
        }

        if (!isDigitsOnly && *p != '\0') {
            char logEntry[64] = {0};
            if (std::strstr(p, "KSXT") != nullptr && std::strstr(p, "0.2") != nullptr) {
                std::snprintf(logEntry, sizeof(logEntry), "KSXT COM1 0.2s (5 Hz)");
            } else if (std::strstr(p, "KSXT") != nullptr && std::strstr(p, "0.1") != nullptr) {
                std::snprintf(logEntry, sizeof(logEntry), "KSXT COM1 0.1s (10 Hz)");
            } else if (std::strstr(p, "KSXT") != nullptr && std::strstr(p, "1") != nullptr) {
                std::snprintf(logEntry, sizeof(logEntry), "KSXT COM1 1s (1 Hz)");
            } else {
                size_t len = 0;
                while (p[len] && p[len] != '\r' && p[len] != '\n' && (len + 1) < sizeof(logEntry)) {
                    logEntry[len] = p[len];
                    len++;
                }
                while (len > 0 && (logEntry[len - 1] == ' ' || logEntry[len - 1] == '\t')) {
                    logEntry[--len] = '\0';
                }
            }

            if (logEntry[0] != '\0') {
                if (auditState_.com1Logs[0] != '\0') {
                    std::strncat(auditState_.com1Logs, ", ", sizeof(auditState_.com1Logs) - std::strlen(auditState_.com1Logs) - 1);
                }
                std::strncat(auditState_.com1Logs, logEntry, sizeof(auditState_.com1Logs) - std::strlen(auditState_.com1Logs) - 1);
            }
            auditState_.collectedLogs++;
            if (auditState_.expectedLogs <= 0 || auditState_.collectedLogs >= auditState_.expectedLogs) {
                auditState_.receivedUniloglist = true;
            }
            if (auditState_.isComplete()) {
                renderAuditReport();
            }
        }
        return true;
    }

    // Parse #HEADINGSTATUSA
    if (startsWithIgnoreCase(resp, "#HEADINGSTATUSA") || startsWithIgnoreCase(resp, "#HEADINGSTATUS")) {
        const char *semi = std::strchr(resp, ';');
        if (semi) {
            const char *payload = semi + 1;
            char *endPtr = nullptr;
            float lengthCm = std::strtof(payload, &endPtr);
            float tolCm = 0.0f;
            if (endPtr && *endPtr == ',') {
                tolCm = std::strtof(endPtr + 1, nullptr);
            }
            auditState_.baselineLengthCm = lengthCm;
            auditState_.baselineTolCm = tolCm;
            auditState_.hasBaseline = (lengthCm > 0.001f);
        }
        auditState_.receivedHeadingStatus = true;
        if (auditState_.isComplete()) {
            renderAuditReport();
        }
        return true;
    }

    return false;
}

void SerialCli::renderAuditReport() {
    if (!auditState_.inProgress) return;
    auditState_.inProgress = false;

    CliSource prevSource = activeSource_;
    activeSource_ = auditState_.source;

    println();
    println("========================================");
    println("     GNSS Receiver Audit Report         ");
    println("========================================");
    if (auditState_.model[0] != '\0') {
        if (auditState_.hwPn[0] != '\0') {
            printFormat("Hardware Model: %s (PN: %s)\r\n", auditState_.model, auditState_.hwPn);
        } else {
            printFormat("Hardware Model: %s\r\n", auditState_.model);
        }
    } else {
        println("Hardware Model: Unknown");
    }

    if (auditState_.serialNum[0] != '\0') {
        printFormat("Serial Number:  %s\r\n", auditState_.serialNum);
    }

    if (auditState_.fwVersion[0] != '\0') {
        if (auditState_.buildDate[0] != '\0') {
            printFormat("Firmware:       %s (%s)\r\n", auditState_.fwVersion, auditState_.buildDate);
        } else {
            printFormat("Firmware:       %s\r\n", auditState_.fwVersion);
        }
    } else {
        println("Firmware:       Unknown");
    }

    if (auditState_.mode[0] != '\0') {
        printFormat("Operating Mode: %s\r\n", auditState_.mode);
    } else {
        println("Operating Mode: Unknown");
    }

    if (auditState_.com1Logs[0] != '\0') {
        printFormat("COM1 Output:    %s\r\n", auditState_.com1Logs);
    } else {
        println("COM1 Output:    None / Unknown");
    }

    if (auditState_.hasBaseline) {
        printFormat("Baseline Range: %.1f cm (± %.1f cm)  [CONSTRAINED]\r\n",
                    auditState_.baselineLengthCm, auditState_.baselineTolCm);
    } else {
        println("Baseline Range: Unconstrained (Auto)");
    }

    bool modeOk = (std::strstr(auditState_.mode, "ROVER") != nullptr);
    bool logOk = (std::strstr(auditState_.com1Logs, "KSXT") != nullptr);
    if (modeOk && logOk) {
        println("Overall Config: [OK] Optimal for Dual-Antenna Compass");
    } else if (auditState_.receivedVersion) {
        println("Overall Config: [WARN] Receiver active; KSXT or ROVER not detected");
    } else {
        println("Overall Config: [FAIL] Incomplete response from receiver");
    }

    println("========================================");
    println();
    printPrompt();

    activeSource_ = prevSource;
}

void SerialCli::printGpsStatus(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    GpsLinkVerdict v = gpsManager_.evaluateStreamHealth(nowMs);

    println();
    println("========================================");
    println("     GNSS Receiver Stream Diagnosis     ");
    println("========================================");
    if (simulator_.isActive()) {
        println("Operation Mode: SIMULATION ACTIVE (Hardware Serial2 suppressed)");
        printFormat("Sim Injections: %u pkts (last %lu ms ago)\r\n",
                    simulator_.getPacketCount(),
                    (nowMs - simulator_.getLastInjectionMillis()));
    }

    // Bidirectional Link & TX Capability Status
    if (gpsManager_.isSendAvailable()) {
        println("Link Status:    CONNECTED (Bidirectional TX/RX Active)");
        println("TX Capability:  AVAILABLE (UM982 command responses confirmed)");
    } else if (gpsManager_.getTxState() == GpsTxState::Probing) {
        println("Link Status:    PROBING (Testing TX on GPIO 43)");
        println("TX Capability:  PROBING (Discovery probe in flight)");
    } else if (gpsManager_.getTxState() == GpsTxState::Unavailable) {
        println("Link Status:    CONNECTED (Receive-Only Mode)");
        println("TX Capability:  UNAVAILABLE (TX wire not detected on GPIO 43)");
    } else {
        println("Link Status:    CONNECTED (Receive-Only Mode)");
        println("TX Capability:  UNKNOWN (Type 'gps probe' to test TX line)");
    }

    // Receiver Configuration Assessment
    if (gpsManager_.getKsxtCount() > 0 && (nowMs - gpsManager_.getLastKsxtMillis()) < 3000) {
        if (gpsManager_.getHdtPacketCount() == 0 && gpsManager_.getGllPacketCount() == 0) {
            println("Config State:   OPTIMAL ($KSXT streaming at 5.0 Hz, legacy unlogged)");
        } else {
            println("Config State:   OPTIMAL ($KSXT active, legacy messages also detected)");
        }
    } else if (gpsManager_.getHdtPacketCount() > 0 || gpsManager_.getGllPacketCount() > 0) {
        println("Config State:   LEGACY COMPATIBLE ($GPHDT/$GNGLL active, upgrade to $KSXT available)");
    } else if (gpsManager_.getTotalBytesReceived() > 0) {
        println("Config State:   UNCONFIGURED / FACTORY DEFAULT (No heading sentence detected)");
    } else {
        println("Config State:   NO DATA");
    }

    if (gpsManager_.getConfigPipeline().isRunning()) {
        printFormat("Config Engine:  RUNNING (Step: %s)\r\n",
                    GpsConfigPipeline::stepToString(gpsManager_.getConfigPipeline().getCurrentStep()));
    } else if (gpsManager_.getConfigPipeline().isSuccess()) {
        println("Config Engine:  SUCCESS (Receiver configured)");
    } else if (gpsManager_.getConfigPipeline().isFailed()) {
        printFormat("Config Engine:  FAILED (%s)\r\n",
                    gpsManager_.getConfigPipeline().getFailureReasonString());
    }

    if (v == GPS_LINK_DISCONNECTED) {
        println("Physical Link:  DISCONNECTED / NO DATA");
        if (gpsManager_.getTotalBytesReceived() == 0) {
            println("Details:        0 bytes received on Serial2 since boot.");
        } else {
            printFormat("Details:        Last byte received %.1f seconds ago.\r\n",
                        (nowMs - gpsManager_.getLastByteMillis()) / 1000.0);
        }
        println("Verdict:        [FAIL] NO GPS DATA STREAM DETECTED");
        println("Troubleshooting:");
        println("  1. Verify GX12 3-pin cable is securely connected.");
        println("  2. Check 5V / 12V power to the UM982 receiver enclosure.");
        println("  3. Verify RX pin (GPIO 44) wiring to UM982 TX.");
    } else if (v == GPS_LINK_NOISY_CORRUPT) {
        println("Physical Link:  BYTES RECEIVED, BUT NO VALID NMEA/KSXT");
        printFormat("Total Bytes:    %lu\r\n", gpsManager_.getTotalBytesReceived());
        println("Verdict:        [FAIL] FRAMING / BAUD RATE MISMATCH");
        println("Troubleshooting:");
        println("  1. Verify receiver COM1 baud rate is set to 115200 (8N1).");
        println("  2. Check for loose connection or electrical noise on signal line.");
    } else {
        println("Physical Link:  CONNECTED (Hardware Serial Link Active)");
        printFormat("Total Bytes:    %lu (last byte %lu ms ago)\r\n",
                    gpsManager_.getTotalBytesReceived(),
                    nowMs - gpsManager_.getLastByteMillis());
        println();
        println("Message Streams:");
        printFormat("  - $GNGLL (Position):   %u pkts (last ", gpsManager_.getGllPacketCount());
        if (gpsManager_.getGllPacketCount() > 0) {
            printFormat("%.1fs ago)\r\n", (nowMs - gpsManager_.getLastGllMillis()) / 1000.0);
        } else {
            println("never)");
        }

        printFormat("  - $KSXT  (Navigation): %u pkts (last ", gpsManager_.getKsxtCount());
        if (gpsManager_.getKsxtCount() > 0) {
            printFormat("%.1fs ago)\r\n", (nowMs - gpsManager_.getLastKsxtMillis()) / 1000.0);
        } else {
            println("never)");
        }

        printFormat("  - $HDT   (Heading):    %u pkts (last ", gpsManager_.getHdtPacketCount());
        if (gpsManager_.getHdtPacketCount() > 0) {
            printFormat("%.1fs ago)\r\n", (nowMs - gpsManager_.getLastHdtMillis()) / 1000.0);
        } else {
            println("never)");
        }

        println();
        println("Data Fix Status:");
        if (gpsManager_.hasLocation()) {
            char gbuf[10];
            geoPointToGrid(gpsManager_.getLocation(), gbuf, 8);
            printFormat("  - Coordinates:         %.5f, %.5f (Grid: %s)\r\n",
                        gpsManager_.getLocation().latitude,
                        gpsManager_.getLocation().longitude,
                        gbuf);
        } else {
            println("  - Coordinates:         SEARCHING (No fix yet)");
        }

        print("  - Dual-Antenna Heading:");
        if (gpsManager_.hasHeading()) {
            if (gpsManager_.getBearingQuality() == BEARING_FIXED) {
                println(" RTK FIXED (Pure White - High Precision)");
            } else if (gpsManager_.getBearingQuality() == BEARING_FLOAT) {
                println(" RTK FLOAT (Bright Yellow - Degraded Sky View)");
            } else {
                println(" LOST / STALE (Bright Red - Tracking Dropped)");
            }
        } else {
            println(" ACQUIRING (Waiting for initial heading lock)");
        }

        print("\nOverall Verdict:        ");
        if (v == GPS_LINK_OK_FIXED) {
            println("[PASS] FULLY OPERATIONAL (RTK Fixed)");
        } else if (v == GPS_LINK_HEADING_FLOAT) {
            println("[PASS - WARN] OPERATIONAL (RTK Float - Degraded reception)");
        } else if (v == GPS_LINK_POSITION_ONLY) {
            println("[WAIT] POSITION LOCKED, WAITING FOR HEADING BASELINE");
        } else {
            println("[WAIT] RECEIVER TALKING, ACQUIRING SATELLITES");
        }
    }
    println("========================================\n");
}
