#include "serial_cli.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include "text_output.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cctype>

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <iostream>
#endif

// Portable case-insensitive comparisons
static bool equalsIgnoreCase(const char *a, const char *b) {
    if (!a || !b) return false;
#if defined(_WIN32)
    return _stricmp(a, b) == 0;
#else
    return strcasecmp(a, b) == 0;
#endif
}

static bool startsWithIgnoreCase(const char *str, const char *prefix) {
    if (!str || !prefix) return false;
    size_t prefixLen = strlen(prefix);
#if defined(_WIN32)
    return _strnicmp(str, prefix, prefixLen) == 0;
#else
    return strncasecmp(str, prefix, prefixLen) == 0;
#endif
}

SerialCli::SerialCli(GpsManager &gpsManager, Calibration &calibration, GpsSimulator &simulator, RoverLogManager *roverLogManager, BleSerialManager *bleManager)
    : gpsManager_(gpsManager),
      calibration_(calibration),
      simulator_(simulator),
      roverLogManager_(roverLogManager),
      bleManager_(bleManager),
      activeSource_(CLI_SOURCE_USB),
      navContextFn_(nullptr),
      navContext_(nullptr),
      runTestCb_(nullptr),
      runTestContext_(nullptr),
      stateChangedCb_(nullptr),
      stateChangedContext_(nullptr),
      roverSimCb_(nullptr),
      roverSimContext_(nullptr),
      roverStartCb_(nullptr),
      roverStopCb_(nullptr),
      roverUndoCb_(nullptr),
      roverActionContext_(nullptr),
      outputFn_(nullptr),
      outputContext_(nullptr),
      restartCb_(nullptr),
      restartContext_(nullptr),
      pendingRoverClear_(false),
      pendingAudit_(false),
      pendingAuditSource_(CLI_SOURCE_USB)
{
    registerGpsCallbacks();
    attachBle();
}

void SerialCli::registerGpsCallbacks() {
    gpsManager_.setOnResponse([](const char *resp, void *ctx) {
        auto *self = static_cast<SerialCli*>(ctx);
        if (self && !self->gpsManager_.getConfigPipeline().isRunning()) {
            if (self->auditState_.inProgress && self->handleAuditResponse(resp)) {
                return;
            }
            self->broadcastFormat("[GPS RX] %s\r\n", resp);
        }
    }, this);

    gpsManager_.getConfigPipeline().setOnProgress([](ConfigStep, ConfigPipelineState, const char *msg, void *ctx) {
        auto *self = static_cast<SerialCli*>(ctx);
        if (self && msg && msg[0] != '\0') {
            self->broadcastln(msg);
        }
    }, this);

    gpsManager_.setOnProbeResult([](GpsTxState state, const char *msg, void *ctx) {
        auto *self = static_cast<SerialCli*>(ctx);
        if (!self) return;
        if (msg && msg[0] != '\0') {
            self->broadcastln(msg);
        }
        if (self->pendingAudit_) {
            self->pendingAudit_ = false;
            CliSource prevSource = self->getActiveSource();
            self->setActiveSource(self->pendingAuditSource_);
            if (state == GpsTxState::Available) {
                self->startGpsAudit(0);
            } else if (state == GpsTxState::Unavailable) {
                self->println("[GPS] Cannot audit: TX wire not connected (Receive-only mode).");
                self->printPrompt();
            }
            self->setActiveSource(prevSource);
        }
    }, this);
}

void SerialCli::begin() {
#if defined(ARDUINO)
    while (Serial.available()) {
        (void)Serial.read();
    }
#endif
    usbLine_.reset();
    bleLine_.reset();
    pendingRoverClear_ = false;
    pendingAudit_ = false;
    pendingAuditSource_ = CLI_SOURCE_USB;
    auditState_.inProgress = false;
    activeSource_ = CLI_SOURCE_USB;
}

void SerialCli::attachBle() {
    if (!bleManager_) return;
    bleManager_->setDataReceivedCallback([](const char *data, size_t len, void *ctx) {
        static_cast<SerialCli*>(ctx)->handleBleInput(data, len);
    }, this);
}

void SerialCli::setBleManager(BleSerialManager *mgr) {
    bleManager_ = mgr;
    attachBle();
}

void SerialCli::handleBleConnectionChanged(bool connected) {
    (void)connected;
    // A partial line from the previous session must not prefix the next client's first command,
    // and replies to later asynchronous output should no longer target the old BLE session.
    bleLine_.reset();
    activeSource_ = CLI_SOURCE_USB;
}

void SerialCli::setOriginGrid(const char *grid) {
    if (grid) {
        strncpy(fallbackNavContext_.originGrid, grid, sizeof(fallbackNavContext_.originGrid) - 1);
        fallbackNavContext_.originGrid[sizeof(fallbackNavContext_.originGrid) - 1] = '\0';
    } else {
        fallbackNavContext_.originGrid[0] = '\0';
    }
}

void SerialCli::setCurrentGrid(const char *grid) {
    if (grid) {
        strncpy(fallbackNavContext_.currentGrid, grid, sizeof(fallbackNavContext_.currentGrid) - 1);
        fallbackNavContext_.currentGrid[sizeof(fallbackNavContext_.currentGrid) - 1] = '\0';
    } else {
        fallbackNavContext_.currentGrid[0] = '\0';
    }
}

CliNavContext SerialCli::getNavContext() const {
    return navContextFn_ ? navContextFn_(navContext_) : fallbackNavContext_;
}

double SerialCli::getTargetBearing() const {
    return getNavContext().targetBearing;
}

bool SerialCli::isRoverMode() const {
    return getNavContext().isRoverMode;
}

double SerialCli::getRoverDistance() const {
    return getNavContext().roverDistanceMeters;
}

void SerialCli::consoleWrite(const char *str) {
    if (outputFn_) {
        outputFn_(str, outputContext_);
        return;
    }
#if defined(ARDUINO)
    Serial.print(str);
#else
    std::cout << str;
#endif
}

void SerialCli::print(const char *str) {
    if (!str) return;
    if (activeSource_ == CLI_SOURCE_BLE && bleManager_) {
        bleManager_->print(str);
    }
    consoleWrite(str); // USB reply, or mirror of a BLE reply for live monitoring
}

void SerialCli::printSink(const char *str, void *ctx) {
    static_cast<SerialCli*>(ctx)->print(str);
}

void SerialCli::println(const char *str) {
    textout::println(printSink, this, str);
}

void SerialCli::printFormat(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    textout::vprintFormat(printSink, this, fmt, args);
    va_end(args);
}

void SerialCli::printPrompt() {
    print("> ");
}

void SerialCli::broadcast(const char *str) {
    if (!str) return;
    consoleWrite(str);
    if (bleManager_ && bleManager_->isConnected()) {
        bleManager_->print(str);
    }
}

void SerialCli::broadcastSink(const char *str, void *ctx) {
    static_cast<SerialCli*>(ctx)->broadcast(str);
}

void SerialCli::broadcastln(const char *str) {
    textout::println(broadcastSink, this, str);
}

void SerialCli::broadcastFormat(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    textout::vprintFormat(broadcastSink, this, fmt, args);
    va_end(args);
}

void SerialCli::handleBleInput(const char *data, size_t len, unsigned long nowMs) {
    if (!data || len == 0) return;
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    for (size_t i = 0; i < len; ++i) {
        if (bleLine_.feed(data[i]) == LineBuffer::LineReady) {
            executeCommand(bleLine_.c_str(), nowMs, CLI_SOURCE_BLE);
            bleLine_.clear();
        }
    }
}

void SerialCli::update(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif

    if (bleManager_) {
        bleManager_->update(nowMs);
    }

    if (auditState_.inProgress) {
        bool quietAfterResponses = (auditState_.receivedVersion &&
                                     auditState_.receivedMode &&
                                     auditState_.lastRespTimeMs > 0 &&
                                     (nowMs - auditState_.lastRespTimeMs >= 500));
        bool hardTimeout = (nowMs - auditState_.startTimeMs >= 3500);

        if (quietAfterResponses || hardTimeout) {
            renderAuditReport();
        }
    }

#if defined(ARDUINO)
    while (Serial.available()) {
        char c = (char)Serial.read();
        // Raw sentence injection lines ("$...") are not echoed
        bool echo = usbLine_.empty() ? (c != '$') : (usbLine_.c_str()[0] != '$');
        switch (usbLine_.feed(c)) {
            case LineBuffer::Appended:
                if (echo) Serial.write(c); // Terminal echo
                break;
            case LineBuffer::Erased:
                Serial.print("\b \b");
                break;
            case LineBuffer::Cleared:
                Serial.println("^C");
                printPrompt();
                break;
            case LineBuffer::LineReady:
                if (usbLine_.c_str()[0] != '$') {
                    Serial.println(); // Terminal newline echo for interactive commands
                }
                executeCommand(usbLine_.c_str(), nowMs, CLI_SOURCE_USB);
                usbLine_.clear();
                break;
            case LineBuffer::Ignored:
                break;
        }
    }
#else
    (void)nowMs;
#endif
}

void SerialCli::executeCommand(const char *cmdLine, unsigned long nowMs, CliSource source) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    activeSource_ = source;
    if (!cmdLine) return;

    // Skip leading whitespace
    while (*cmdLine == ' ' || *cmdLine == '\t' || *cmdLine == '\r' || *cmdLine == '\n') {
        cmdLine++;
    }
    if (*cmdLine == '\0') {
        printPrompt();
        return;
    }

    // Direct sentence injection: e.g. "$KSXT..." or "$GNGLL..."
    if (cmdLine[0] == '$') {
        if (!simulator_.isActive()) {
            println("[SIM] Simulation mode activated by sentence injection. Hardware Serial2 suppressed.");
        }
        simulator_.injectSentence(cmdLine, nullptr, nowMs);
        gpsManager_.feedSentence(cmdLine, nowMs);
        if (stateChangedCb_) {
            stateChangedCb_(stateChangedContext_);
        }
        return;
    }

    // Copy to trimmed buffer and strip trailing whitespace
    char cleanCmd[LineBuffer::kCapacity];
    size_t len = 0;
    while (cmdLine[len] && len < sizeof(cleanCmd) - 1) {
        cleanCmd[len] = cmdLine[len];
        len++;
    }
    cleanCmd[len] = '\0';

    while (len > 0 && (cleanCmd[len - 1] == ' ' || cleanCmd[len - 1] == '\t' ||
                       cleanCmd[len - 1] == '\r' || cleanCmd[len - 1] == '\n')) {
        cleanCmd[--len] = '\0';
    }
    if (len == 0) {
        printPrompt();
        return;
    }

    if (pendingRoverClear_) {
        pendingRoverClear_ = false;
        if (equalsIgnoreCase(cleanCmd, "yes") || equalsIgnoreCase(cleanCmd, "y") ||
            equalsIgnoreCase(cleanCmd, "confirm")) {
            doRoverClear();
        } else {
            println("[ROVER] Rover clear aborted.");
        }
        printPrompt();
        return;
    }

    if (startsWithIgnoreCase(cleanCmd, "rover")) {
        const char *arg = cleanCmd + 5;
        while (*arg == ' ' || *arg == '\t') arg++;
        handleRoverCommand(arg, nowMs);
    } else if (startsWithIgnoreCase(cleanCmd, "sim")) {
        const char *arg = cleanCmd + 3;
        while (*arg == ' ' || *arg == '\t') arg++;
        handleSimCommand(arg, nowMs);
    } else if (equalsIgnoreCase(cleanCmd, "help") || strcmp(cleanCmd, "?") == 0) {
        printHelp();
    } else if (startsWithIgnoreCase(cleanCmd, "gps")) {
        const char *arg = cleanCmd + 3;
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0' || equalsIgnoreCase(arg, "status")) {
            printGpsStatus(nowMs);
        } else {
            handleGpsCommand(arg, nowMs);
        }
    } else if (equalsIgnoreCase(cleanCmd, "status") ||
               equalsIgnoreCase(cleanCmd, "diag")   || equalsIgnoreCase(cleanCmd, "s")   ||
               equalsIgnoreCase(cleanCmd, "g")) {
        printGpsStatus(nowMs);
    } else if (equalsIgnoreCase(cleanCmd, "test") || equalsIgnoreCase(cleanCmd, "demo") ||
               equalsIgnoreCase(cleanCmd, "t")) {
        if (runTestCb_) {
            runTestCb_(runTestContext_);
        } else {
            println("[CLI] Test runner callback not registered.");
        }
    } else if (equalsIgnoreCase(cleanCmd, "ble") || startsWithIgnoreCase(cleanCmd, "ble ")) {
        const char *arg = cleanCmd + 3;
        while (*arg == ' ' || *arg == '\t') arg++;
        if (!bleManager_) {
            println("[BLE] Bluetooth not available");
        } else if (equalsIgnoreCase(arg, "diag on")) {
            bleManager_->setDiagEnabled(true);
            println("[BLE] Diagnostics ON (RSSI every 3 s while connected, main-loop stall reports; USB serial)");
        } else if (equalsIgnoreCase(arg, "diag off")) {
            bleManager_->setDiagEnabled(false);
            println("[BLE] Diagnostics OFF");
        } else if (arg[0] == '\0' || equalsIgnoreCase(arg, "status")) {
            printFormat("[BLE] Device: %s\r\n", bleManager_->getDeviceName());
            printFormat("[BLE] Connected: %s | Subscribed: %s | MTU: %u\r\n",
                        bleManager_->isConnected() ? "yes" : "no",
                        bleManager_->isSubscribed() ? "yes" : "no",
                        (unsigned)bleManager_->getMtu());
            printFormat("[BLE] Advertising: %s\r\n", bleManager_->isAdvertising() ? "yes" : "no");
            printFormat("[BLE] TX queue: %u / %u bytes | Dropped: %lu bytes | Send retries: %lu\r\n",
                        (unsigned)bleManager_->getTxQueued(), (unsigned)bleManager_->getTxCapacity(),
                        (unsigned long)bleManager_->getTxDroppedBytes(),
                        (unsigned long)bleManager_->getTxRetries());
            printFormat("[BLE] Diagnostics: %s\r\n", bleManager_->isDiagEnabled() ? "on" : "off");
        } else {
            println("Usage: ble [status] | ble diag on|off");
        }
    } else if (startsWithIgnoreCase(cleanCmd, "cal")) {
        const char *arg = cleanCmd + 3;
        while (*arg == ' ' || *arg == '\t') arg++;
        handleCalCommand(arg);
    } else if (equalsIgnoreCase(cleanCmd, "disconnect") || equalsIgnoreCase(cleanCmd, "exit") ||
               equalsIgnoreCase(cleanCmd, "quit")) {
        if (activeSource_ == CLI_SOURCE_BLE && bleManager_) {
            println("Disconnecting BLE connection...");
            bleManager_->disconnect();
        } else {
            println("[CLI] 'disconnect' applies to active wireless Bluetooth sessions.");
        }
    } else if (equalsIgnoreCase(cleanCmd, "restart") || equalsIgnoreCase(cleanCmd, "reboot")) {
        println("[SYSTEM] Restarting ESP32 controller...");
        if (bleManager_) {
            bleManager_->update(nowMs);
        }
        if (restartCb_) {
            restartCb_(restartContext_);
        } else {
#if defined(ARDUINO)
            delay(100);
            esp_restart();
#endif
        }
        return;
    } else {
        printFormat("Unknown command: '%s'. Type 'help' for available commands.\r\n", cleanCmd);
    }

    if (!pendingAudit_ && !auditState_.inProgress) {
        printPrompt();
    }
}

void SerialCli::handleGpsCommand(const char *arg, unsigned long nowMs) {
    if (equalsIgnoreCase(arg, "probe")) {
        println("[GPS PROBE] Transmitting 'version' discovery probe on GPIO 43...");
        gpsManager_.probeTx(nowMs);
        return;
    }

    if (startsWithIgnoreCase(arg, "config")) {
        if (!gpsManager_.isSendAvailable()) {
            println("[GPS] TX wire not detected. Connect GPIO 43 to UM982 RX to configure.");
            return;
        }
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
            char *endPtr = nullptr;
            baselineCm = std::strtof(sub, &endPtr);
            if (endPtr && *endPtr) {
                float t = std::strtof(endPtr, nullptr);
                if (t > 0.0f) tolCm = t;
            }
        } else if ((*sub >= '0' && *sub <= '9') || *sub == '.') {
            char *endPtr = nullptr;
            baselineCm = std::strtof(sub, &endPtr);
            if (endPtr && *endPtr) {
                float t = std::strtof(endPtr, nullptr);
                if (t > 0.0f) tolCm = t;
            }
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

    println("[GPS ERROR] Unknown gps subcommand. Available:");
    println("  gps status            - Live GNSS receiver stream & link diagnostic");
    println("  gps probe             - Test bidirectional TX connection to UM982");
    println("  gps config [baseline] - Run automated 5 Hz $KSXT receiver setup");
    println("  gps send <command>    - Send raw command to UM982 receiver");
    println("  gps check (or audit)  - Query receiver running parameters");
}

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
    if (strstr(resp, "ERROR") != nullptr || strstr(resp, "error") != nullptr) {
        printFormat("[GPS AUDIT ERROR] %s\r\n", resp);
        return true;
    }

    // Parse #VERSION
    if (startsWithIgnoreCase(resp, "#VERSION")) {
        const char *semi = strchr(resp, ';');
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
        const char *semi = strchr(resp, ';');
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
        // Acknowledge header; wait for entries
        return true;
    }

    // Parse UNILOGLIST log entries starting with '<'
    if (resp[0] == '<') {
        const char *p = resp + 1;
        while (*p == ' ' || *p == '\t') p++;
        // Check if digits-only line (e.g. "< 1" or "< \t 1 \r\n")
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
            auditState_.expectedLogs = atoi(p);
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
            if (strstr(p, "KSXT") != nullptr && strstr(p, "0.2") != nullptr) {
                snprintf(logEntry, sizeof(logEntry), "KSXT COM1 0.2s (5 Hz)");
            } else if (strstr(p, "KSXT") != nullptr && strstr(p, "0.1") != nullptr) {
                snprintf(logEntry, sizeof(logEntry), "KSXT COM1 0.1s (10 Hz)");
            } else if (strstr(p, "KSXT") != nullptr && strstr(p, "1") != nullptr) {
                snprintf(logEntry, sizeof(logEntry), "KSXT COM1 1s (1 Hz)");
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
                    strncat(auditState_.com1Logs, ", ", sizeof(auditState_.com1Logs) - strlen(auditState_.com1Logs) - 1);
                }
                strncat(auditState_.com1Logs, logEntry, sizeof(auditState_.com1Logs) - strlen(auditState_.com1Logs) - 1);
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
        const char *semi = strchr(resp, ';');
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

    bool modeOk = (strstr(auditState_.mode, "ROVER") != nullptr);
    bool logOk = (strstr(auditState_.com1Logs, "KSXT") != nullptr);
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

void SerialCli::printHelp() {
    println();
    println("=== GNSS Compass Serial Commands ===");
    println("  status (or gps)       - Live GNSS receiver stream & link diagnostic");
    println("  gps probe             - Test bidirectional TX connection to UM982");
    println("  gps config [baseline] - Run automated 5 Hz $KSXT receiver setup");
    println("  gps send <command>    - Send raw command to UM982 receiver");
    println("  gps check (or audit)  - Query receiver running parameters");
    println("  rover [status|start|stop|history|undo|max|clear] - Rover transit tracking");
    println("  test (or demo)        - Run visual display self-test demo");
    println("  cal [snap|zero|<val>] - View or configure antenna calibration");
    println("  sim [on|off|hdg|pos|grid|rover|status] - GPS simulation commands");
    println("  exit (or disconnect)  - Terminate wireless BLE connection cleanly");
    println("  restart (or reboot)   - Restart the ESP32 controller");
    println("  ble [diag on|off]     - BLE link status / toggle BLE RF diagnostics");
    println("  $<NMEA/KSXT sentence> - Inject raw GNSS sentence directly");
    println("  help (or ?)           - Show this help menu");
    println("====================================");
    println();
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

void SerialCli::printCalibrationStatus() {
    CliNavContext nav = getNavContext();
    println();
    println("=== Antenna Calibration Status ===");
    printFormat("  Stored Offset : %+5.1f deg\r\n", calibration_.getOffset());
    printFormat("  Target Bearing: %5.1f deg\r\n", nav.targetBearing);
    if (gpsManager_.hasHeading()) {
        double rawHdg = gpsManager_.getRawHeading();
        printFormat("  Raw GNSS Hdg  : %5.1f deg\r\n", rawHdg);
        printFormat("  Calibrated Hdg: %5.1f deg\r\n", gpsManager_.getCalibratedHeading(calibration_.getOffset()));
    } else {
        println("  Raw GNSS Hdg  : ---.- (No heading fix)");
    }
    println("==================================");
    println();
}

void SerialCli::printSimulationStatus(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    println();
    println("=== GPS Simulation Mode Status ===");
    printFormat("  Simulation Active : %s\r\n",
                simulator_.isActive() ? "YES (Hardware Serial2 suppressed)" : "NO (Live Serial2 active)");
    printFormat("  Sim Packets Recv  : %u\r\n", simulator_.getPacketCount());
    if (simulator_.getPacketCount() > 0) {
        printFormat("  Last Sim Packet   : %lu ms ago\r\n",
                    (nowMs - simulator_.getLastInjectionMillis()));
    }

    const char *qStr = "LOST";
    if (gpsManager_.getBearingQuality() == BEARING_FIXED) qStr = "FIXED";
    else if (gpsManager_.getBearingQuality() == BEARING_FLOAT) qStr = "FLOAT";

    printFormat("  Current Heading   : %5.1f deg (Quality: %s)\r\n",
                gpsManager_.hasHeading() ? gpsManager_.getCalibratedHeading(calibration_.getOffset()) : 0.0f,
                qStr);

    char localGrid[10] = "JJ00AA00";
    if (gpsManager_.hasLocation()) {
        geoPointToGrid(gpsManager_.getLocation(), localGrid, 8);
        printFormat("  Current Position  : %9.5f, %9.5f (Grid: %s)\r\n",
                    gpsManager_.getLocation().latitude,
                    gpsManager_.getLocation().longitude,
                    localGrid);
    } else {
        println("  Current Position  : ---.-----, ---.----- (No fix)");
    }

    CliNavContext nav = getNavContext();
    printFormat("  Rover Mode Active : %s\r\n",
                nav.isRoverMode ? "YES (Rover Mode)" : "NO (Stationary target tracking)");
    if (nav.isRoverMode) {
        int distKm = (int)std::floor(nav.roverDistanceMeters / 1000.0);
        printFormat("  Rover Distance    : %5.1f km (Alert: %s)\r\n",
                    nav.roverDistanceMeters / 1000.0,
                    (distKm < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR");
    }
    println("==================================");
    println();
}

void SerialCli::handleCalCommand(const char *arg) {
    if (!arg || *arg == '\0' || equalsIgnoreCase(arg, "status") || equalsIgnoreCase(arg, "show")) {
        printCalibrationStatus();
        return;
    }

    if (equalsIgnoreCase(arg, "zero") || equalsIgnoreCase(arg, "reset") || strcmp(arg, "0") == 0) {
        calibration_.reset();
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
        println("[CAL] Offset reset to 0.0 deg (Cleared).");
        return;
    }

    if (equalsIgnoreCase(arg, "snap")) {
        if (!gpsManager_.hasHeading()) {
            println("[CAL ERROR] Cannot snap: No valid GNSS heading fix.");
        } else {
            CliNavContext nav = getNavContext();
            calibration_.snap(nav.targetBearing, gpsManager_.getRawHeading());
            if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
            printFormat("[CAL] Snapped! New offset: %+5.1f deg\r\n", calibration_.getOffset());
        }
        return;
    }

    const char *numStr = arg;
    if (startsWithIgnoreCase(arg, "set ")) {
        numStr = arg + 4;
        while (*numStr == ' ' || *numStr == '\t') numStr++;
    }

    float val = (float)atof(numStr);
    calibration_.setOffset(val);
    if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
    printFormat("[CAL] Offset set to %+5.1f deg\r\n", calibration_.getOffset());
}

void SerialCli::handleSimCommand(const char *arg, unsigned long nowMs) {
    if (!arg || *arg == '\0' || equalsIgnoreCase(arg, "status")) {
        printSimulationStatus(nowMs);
        return;
    }

    if (equalsIgnoreCase(arg, "on")) {
        simulator_.setActive(true);
        println("[SIM] Simulation mode enabled. Hardware Serial2 input is now SUPPRESSED.");
        return;
    }

    if (equalsIgnoreCase(arg, "off")) {
        simulator_.setActive(false);
        println("[SIM] Simulation mode disabled. Hardware Serial2 receiver link RESTORED.");
        return;
    }

    // All active simulation commands automatically enable simulation mode
    simulator_.setActive(true);

    if (startsWithIgnoreCase(arg, "hdg")) {
        const char *p = arg + 3;
        while (*p == ' ' || *p == '\t') p++;
        float h = (float)atof(p);
        BearingQuality q = BEARING_FIXED;

        const char *sp = strchr(p, ' ');
        if (sp) {
            while (*sp == ' ') sp++;
            if (equalsIgnoreCase(sp, "float")) q = BEARING_FLOAT;
            else if (equalsIgnoreCase(sp, "lost") || equalsIgnoreCase(sp, "invalid")) q = BEARING_LOST;
            else q = BEARING_FIXED;
        }

        gpsManager_.setHeading(h, q, nowMs);
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);

        const char *qStr = (q == BEARING_FIXED) ? "FIXED" : ((q == BEARING_FLOAT) ? "FLOAT" : "LOST");
        printFormat("[SIM] Heading set to %5.1f deg (Quality: %s)\r\n", h, qStr);
        return;
    }

    if (startsWithIgnoreCase(arg, "pos")) {
        const char *p = arg + 3;
        while (*p == ' ' || *p == '\t') p++;
        char *endPtr = nullptr;
        double lat = strtod(p, &endPtr);
        if (endPtr && *endPtr != '\0') {
            double lon = strtod(endPtr, nullptr);
            gpsManager_.setLocation(lat, lon, nowMs);
            if (stateChangedCb_) stateChangedCb_(stateChangedContext_);

            char gbuf[10];
            geoPointToGrid(gpsManager_.getLocation(), gbuf, 8);
            printFormat("[SIM] Position set to %9.5f, %9.5f (Grid: %s)\r\n", lat, lon, gbuf);
            return;
        }
        println("[SIM ERROR] Usage: sim pos <lat> <lon>");
        return;
    }

    if (startsWithIgnoreCase(arg, "grid")) {
        const char *g = arg + 4;
        while (*g == ' ' || *g == '\t') g++;
        size_t gLen = strlen(g);
        if (gLen >= 4 && gLen <= 8) {
            GeoPoint pt;
            if (gridToGeoPoint(g, pt)) {
                gpsManager_.setLocation(pt, nowMs);
                if (stateChangedCb_) stateChangedCb_(stateChangedContext_);

                char gbuf[10];
                geoPointToGrid(gpsManager_.getLocation(), gbuf, 8);
                printFormat("[SIM] Position set to Grid %s (%9.5f, %9.5f)\r\n",
                            gbuf, pt.latitude, pt.longitude);
                return;
            }
        }
        println("[SIM ERROR] Usage: sim grid <grid4_or_6_or_8>");
        return;
    }

    if (startsWithIgnoreCase(arg, "rover")) {
        const char *p = arg + 5;
        while (*p == ' ' || *p == '\t') p++;
        if (roverSimCb_) {
            char ogBuf[10] = "";
            double distKm = 0.0;
            const char *sp = strchr(p, ' ');
            if (sp) {
                size_t ogLen = sp - p;
                if (ogLen < sizeof(ogBuf)) {
                    strncpy(ogBuf, p, ogLen);
                    ogBuf[ogLen] = '\0';
                }
                distKm = atof(sp + 1);
            } else if (*p != '\0') {
                distKm = atof(p);
            }
            roverSimCb_(ogBuf, distKm, roverSimContext_);
            return;
        }

        // Default rover handler if no UI callback
        GeoPoint origin = gpsManager_.hasLocation() ? gpsManager_.getLocation() : GeoPoint(38.35412, -119.45821, true);
        double distKm = atof(p);
        GeoPoint simulated = GpsSimulator::calculateRoverDisplacement(origin, distKm, 45.0);
        gpsManager_.setLocation(simulated, nowMs);
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
        printFormat("[SIM ROVER] Distance: %5.1f km\r\n", distKm);
        return;
    }

    if (startsWithIgnoreCase(arg, "time")) {
        const char *t = arg + 4;
        while (*t == ' ' || *t == '\t') t++;
        const char *colon = strchr(t, ':');
        if (colon) {
            int h = atoi(t);
            int m = atoi(colon + 1);
            if (h >= 0 && h < 24 && m >= 0 && m < 60) {
                gpsManager_.setTime(h, m);
                if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
                printFormat("[SIM] UTC time set to %02d:%02dZ\r\n", h, m);
                return;
            }
        }
        println("[SIM ERROR] Usage: sim time <hh:mm>");
        return;
    }

    println("[SIM ERROR] Unknown sim command. Available:");
    println("  sim on / sim off");
    println("  sim status");
    println("  sim hdg <deg> [fixed|float|lost]");
    println("  sim pos <lat> <lon>");
    println("  sim grid <grid>");
    println("  sim rover [origin_grid] <dist_km>");
    println("  sim time <hh:mm>");
}

void SerialCli::handleRoverCommand(const char *arg, unsigned long nowMs) {
    while (arg && (*arg == ' ' || *arg == '\t')) arg++;
    if (!arg || *arg == '\0' || startsWithIgnoreCase(arg, "status")) {
        printRoverStatus(nowMs);
        return;
    }

    if (equalsIgnoreCase(arg, "start")) {
        handleRoverStart(nowMs);
        return;
    }

    if (equalsIgnoreCase(arg, "stop")) {
        handleRoverStop(nowMs);
        return;
    }

    if (equalsIgnoreCase(arg, "history") || equalsIgnoreCase(arg, "log")) {
        printRoverHistory();
        return;
    }

    if (equalsIgnoreCase(arg, "undo")) {
        handleRoverUndo(nowMs);
        return;
    }

    if (startsWithIgnoreCase(arg, "max")) {
        const char *p = arg + 3;
        while (*p == ' ' || *p == '\t') p++;
        handleRoverMax(p);
        return;
    }

    if (startsWithIgnoreCase(arg, "clear")) {
        const char *p = arg + 5;
        while (*p == ' ' || *p == '\t') p++;
        handleRoverClear(p);
        return;
    }

    if (equalsIgnoreCase(arg, "help") || strcmp(arg, "?") == 0) {
        printRoverHelp();
        return;
    }

    printFormat("[ROVER ERROR] Unknown rover command: '%s'. Type 'rover help' for available commands.\r\n", arg);
}

void SerialCli::handleRoverStart(unsigned long nowMs) {
    CliNavContext preNav = getNavContext();
    if (preNav.isRoverMode) {
        println("[ROVER] Rover mode is already active. Ignoring restart.");
        return;
    }

    if (roverStartCb_) {
        roverStartCb_(roverActionContext_, nowMs);
    } else {
        if (!gpsManager_.hasLocation()) {
            println("[ROVER ERROR] Cannot start Rover Mode: No valid GPS position fix.");
            return;
        }
        fallbackNavContext_.isRoverMode = true;
        GeoPoint loc = gpsManager_.getLocation();
        fallbackNavContext_.originLat = loc.latitude;
        fallbackNavContext_.originLon = loc.longitude;
        geoPointToGrid(loc, fallbackNavContext_.originGrid);
        if (fallbackNavContext_.currentGrid[0] == '\0') {
            snprintf(fallbackNavContext_.currentGrid, sizeof(fallbackNavContext_.currentGrid), "%s", fallbackNavContext_.originGrid);
        }
        if (roverLogManager_ && roverLogManager_->isEmpty()) {
            RoverStop stop1(0, loc.latitude, loc.longitude, fallbackNavContext_.originGrid, 0.0f, false);
            roverLogManager_->appendStop(stop1);
        }
    }
    if (stateChangedCb_) {
        stateChangedCb_(stateChangedContext_);
    }
    CliNavContext nav = getNavContext();
    if (!nav.isRoverMode) {
        println("[ROVER ERROR] Cannot start Rover Mode: No valid GPS position fix.");
        return;
    }
    printFormat("[ROVER] Rover mode started at %s (%9.5f, %9.5f)\r\n",
                nav.originGrid[0] ? nav.originGrid : "Unknown",
                nav.originLat, nav.originLon);
}

void SerialCli::handleRoverStop(unsigned long nowMs) {
    CliNavContext nav = getNavContext();
    if (!nav.isRoverMode) {
        println("[ROVER] Rover mode is not currently active.");
        return;
    }
    if (roverStopCb_) {
        roverStopCb_(roverActionContext_, nowMs);
    } else {
        fallbackNavContext_.isRoverMode = false;
        if (roverLogManager_ && gpsManager_.hasLocation()) {
            GeoPoint loc = gpsManager_.getLocation();
            char gbuf[9];
            geoPointToGrid(loc, gbuf);
            float dispKm = static_cast<float>(fallbackNavContext_.roverDistanceMeters / 1000.0);
            RoverStop st(0, loc.latitude, loc.longitude, gbuf, dispKm, dispKm >= ROVER_ALERT_THRESHOLD_KM);
            roverLogManager_->appendStop(st);
        }
    }
    if (stateChangedCb_) {
        stateChangedCb_(stateChangedContext_);
    }
    println("[ROVER] Rover mode stopped. Position committed to history.");
}

void SerialCli::handleRoverUndo(unsigned long nowMs) {
    if (roverLogManager_ && roverLogManager_->isEmpty()) {
        println("[ROVER] No stops to undo (history is empty).");
        return;
    }
    if (roverUndoCb_) {
        roverUndoCb_(roverActionContext_, nowMs);
    } else if (roverLogManager_) {
        roverLogManager_->popLastStop();
        fallbackNavContext_.isRoverMode = true;
        if (!roverLogManager_->isEmpty()) {
            const RoverStop *restored = roverLogManager_->getLastStop();
            fallbackNavContext_.originLat = restored->latitude;
            fallbackNavContext_.originLon = restored->longitude;
            snprintf(fallbackNavContext_.originGrid, sizeof(fallbackNavContext_.originGrid), "%s", restored->grid);
        }
    }
    if (stateChangedCb_) {
        stateChangedCb_(stateChangedContext_);
    }

    if (roverLogManager_) {
        if (roverLogManager_->isEmpty()) {
            println("[ROVER] Rover history is now empty. Auto-rearmed at current location.");
        } else {
            const RoverStop *last = roverLogManager_->getLastStop();
            printFormat("[ROVER] Discarded last stop. Restored origin to %s. Rover mode auto-rearmed.\r\n",
                        last ? last->grid : "Unknown");
        }
    } else {
        println("[ROVER] Discarded last stop. Rover mode auto-rearmed.");
    }
}

void SerialCli::handleRoverMax(const char *arg) {
    if (!roverLogManager_) {
        println("[ROVER] Rover log manager not available.");
        return;
    }
    while (*arg == ' ' || *arg == '\t') arg++;
    if (*arg == '\0') {
        printFormat("[ROVER] Max stops capacity: %zu (range: %u-%u)\r\n",
                    roverLogManager_->getMaxStops(), MIN_ROVER_STOPS, ABS_MAX_ROVER_STOPS);
        return;
    }
    int val = atoi(arg);
    if (val <= 0) {
        println("[ROVER ERROR] Invalid capacity. Range: 10-500.");
        return;
    }
    roverLogManager_->setMaxStops(static_cast<size_t>(val), true);
    printFormat("[ROVER] Max stops capacity set to %zu\r\n", roverLogManager_->getMaxStops());
}

void SerialCli::handleRoverClear(const char *arg) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (startsWithIgnoreCase(arg, "confirm") || startsWithIgnoreCase(arg, "yes") ||
        startsWithIgnoreCase(arg, "-y") || startsWithIgnoreCase(arg, "force")) {
        pendingRoverClear_ = false;
        doRoverClear();
        return;
    }
    if (startsWithIgnoreCase(arg, "cancel") || startsWithIgnoreCase(arg, "no") ||
        startsWithIgnoreCase(arg, "-n")) {
        pendingRoverClear_ = false;
        println("[ROVER] Rover clear aborted.");
        return;
    }

    pendingRoverClear_ = true;
    println("[ROVER] WARNING: This will permanently erase all contest location history!");
    println("[ROVER] Type 'rover clear confirm' (or 'yes') to proceed, or 'no' to abort.");
}

void SerialCli::doRoverClear() {
    if (roverLogManager_) {
        roverLogManager_->clearHistory();
        println("[ROVER] Rover location history cleared from NVS.");
    } else {
        println("[ROVER] Rover log manager not available.");
    }
    if (stateChangedCb_) {
        stateChangedCb_(stateChangedContext_);
    }
}

void SerialCli::printRoverStatus(unsigned long nowMs) {
    CliNavContext nav = getNavContext();
    println();
    println("========================================");
    println("          Rover Transit Status          ");
    println("========================================");
    printFormat("Mode:             %s\r\n", nav.isRoverMode ? "ACTIVE (Rover Mode)" : "INACTIVE (Bearing Mode)");
    if (nav.originLat != 0.0 || nav.originLon != 0.0 || nav.originGrid[0] != '\0') {
        printFormat("Origin Coords:    %9.6f, %10.6f\r\n", nav.originLat, nav.originLon);
        printFormat("Origin Grid:      %s\r\n", nav.originGrid[0] ? nav.originGrid : "Unknown");
    } else {
        println("Origin Coords:    Not set (Tap Row 1 or 'rover start')");
    }

    double dispKm = nav.isRoverMode ? (nav.roverDistanceMeters / 1000.0) : nav.lastValidDisplacementKm;
    if (nav.isRoverMode && nav.roverState == RoverState::GpsLostStale) {
        dispKm = nav.lastValidDisplacementKm;
    }
    printFormat("Displacement:     %5.1f km (Threshold: %.1f km)\r\n", dispKm, ROVER_ALERT_THRESHOLD_KM);

    // Current grid and boundary status
    const char *curGrid = nav.currentGrid[0] ? nav.currentGrid : (gpsManager_.hasLocation() ? "Unknown" : "No GPS");
    bool sameGrid = (strncasecmp(nav.originGrid, curGrid, 6) == 0);
    const char *boundaryStr = sameGrid ? "SAME GRID" : "TRAVERSED GRID";
    if (!sameGrid && roverLogManager_ && roverLogManager_->hasVisitedGrid(curGrid)) {
        boundaryStr = "TRAVERSED GRID (Previously Visited)";
    }
    printFormat("Current Grid:     %s (%s)\r\n", curGrid, boundaryStr);

    // Qualification state A-D
    const char *stateStr = "Unknown";
    switch (nav.roverState) {
        case RoverState::InTransitUnder16k:
            stateStr = "STATE A: In-Transit (< 16.0 km alert)";
            break;
        case RoverState::QualifiedNewGrid:
            stateStr = "STATE B: QUALIFIED (>= 16.0 km, New Grid)";
            break;
        case RoverState::QualifiedRevisitedGrid:
            stateStr = "STATE C: REVISITED (>= 16.0 km, Previously Visited Grid)";
            break;
        case RoverState::GpsLostStale:
            stateStr = "STATE D: NO GPS (Signal lost / stale, holding last distance)";
            break;
    }
    printFormat("Qualification:    %s\r\n", stateStr);

    // GPS fix health
    GpsLinkVerdict v = gpsManager_.evaluateStreamHealth(nowMs);
    const char *gpsHealthStr = "DISCONNECTED";
    if (v == GPS_LINK_OK_FIXED) gpsHealthStr = "FIXED (High-Precision RTK)";
    else if (v == GPS_LINK_HEADING_FLOAT) gpsHealthStr = "FLOAT (Degraded RTK)";
    else if (v == GPS_LINK_POSITION_ONLY) gpsHealthStr = "POSITION ONLY (Waiting for Dual Antenna)";
    else if (v == GPS_LINK_ACQUIRING) gpsHealthStr = "ACQUIRING (No Carrier Lock)";
    else if (v == GPS_LINK_NOISY_CORRUPT) gpsHealthStr = "CORRUPT STREAM";
    printFormat("GPS Fix Health:   %s (%s)\r\n", gpsHealthStr, nav.isGpsFresh ? "Fresh" : "Stale / Timeout");

    if (roverLogManager_) {
        printFormat("History Log:      %zu stops logged (max %zu)\r\n",
                    roverLogManager_->getStopCount(), roverLogManager_->getMaxStops());
    }
    println("========================================");
    println();
}

void SerialCli::printRoverHistory() {
    if (!roverLogManager_) {
        println("[ROVER] Rover log manager not available.");
        return;
    }
    size_t count = roverLogManager_->getStopCount();
    size_t maxStops = roverLogManager_->getMaxStops();

    println();
    printFormat("=== ROVER LOCATION HISTORY (%zu stop%s / max %zu) ===\r\n",
                count, (count == 1 ? "" : "s"), maxStops);

    if (count == 0) {
        println("(No stops recorded)");
        println("==================================================");
        println();
        return;
    }

    println("# | UTC Timestamp       | Latitude   | Longitude    | Grid     | Disp (km) | Status");
    println("--+---------------------+------------+--------------+----------+-----------+-----------");

    for (size_t i = 0; i < count; i++) {
        const RoverStop *st = roverLogManager_->getStop(i);
        if (!st) continue;

        char tsBuf[64];
        if (st->timestamp >= 86400 * 365) {
            time_t t = static_cast<time_t>(st->timestamp);
            struct tm tmBuf;
#if defined(_WIN32)
            gmtime_s(&tmBuf, &t);
#else
            gmtime_r(&t, &tmBuf);
#endif
            snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02d %02d:%02d:%02d",
                     tmBuf.tm_year + 1900, tmBuf.tm_mon + 1, tmBuf.tm_mday,
                     tmBuf.tm_hour, tmBuf.tm_min, tmBuf.tm_sec);
        } else {
            int h = (st->timestamp / 3600) % 24;
            int m = (st->timestamp / 60) % 60;
            int s = st->timestamp % 60;
            snprintf(tsBuf, sizeof(tsBuf), "--- %02d:%02d:%02d UTC   ", h, m, s);
        }

        const char *status = "UNDER 16k";
        if (st->displacement_km == 0.0f && !st->qualified_16km) {
            status = "ORIGIN";
        } else if (st->qualified_16km) {
            status = "QUALIFIED";
        }

        printFormat("%-2zu| %-19s | %10.6f | %-12.6f | %-8s | %7.1f k | %s\r\n",
                    i + 1, tsBuf, st->latitude, st->longitude,
                    st->grid[0] ? st->grid : "--------",
                    st->displacement_km, status);
    }
    println("==================================================");
    println();
}

void SerialCli::printRoverHelp() {
    println();
    println("=== Rover Mode Serial Commands ===");
    println("  rover [status]        - Displays active rover transit status");
    println("  rover start           - Emulates Row 1 tap: arms Rover Mode at current location");
    println("  rover stop            - Emulates target exit: commits stop and returns to Bearing");
    println("  rover history (or log)- Formatted ASCII table of contest location stops");
    println("  rover undo            - Discards last stop and auto-rearms Rover Mode");
    println("  rover max [n]         - Gets or sets max stops capacity (10-500) in NVS");
    println("  rover clear [confirm] - Clears rover location history in NVS");
    println("===================================");
    println();
}

