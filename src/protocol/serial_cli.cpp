#include "serial_cli.h"
#include "serial_cli_helpers.h"
#include "text_output.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(ARDUINO)
#include <Arduino.h>
#include <nvs.h>
#else
#include <iostream>
#endif

using namespace cli_helpers;

namespace {

bool isKnownCommand(const char *cleanCmd) {
    if (!cleanCmd || cleanCmd[0] == '\0') return false;
    return startsWithIgnoreCase(cleanCmd, "rover") ||
           (startsWithIgnoreCase(cleanCmd, "log") &&
            (cleanCmd[3] == '\0' || cleanCmd[3] == ' ' || cleanCmd[3] == '\t')) ||
           startsWithIgnoreCase(cleanCmd, "target") ||
           startsWithIgnoreCase(cleanCmd, "sim") ||
           equalsIgnoreCase(cleanCmd, "help") || startsWithIgnoreCase(cleanCmd, "help ") ||
           std::strcmp(cleanCmd, "?") == 0 || startsWithIgnoreCase(cleanCmd, "? ") ||
           startsWithIgnoreCase(cleanCmd, "gps") ||
           equalsIgnoreCase(cleanCmd, "status") || equalsIgnoreCase(cleanCmd, "diag") ||
           equalsIgnoreCase(cleanCmd, "s") || equalsIgnoreCase(cleanCmd, "g") ||
           equalsIgnoreCase(cleanCmd, "test") || equalsIgnoreCase(cleanCmd, "demo") ||
           equalsIgnoreCase(cleanCmd, "t") ||
           equalsIgnoreCase(cleanCmd, "ble") || startsWithIgnoreCase(cleanCmd, "ble ") ||
           startsWithIgnoreCase(cleanCmd, "cal") ||
           equalsIgnoreCase(cleanCmd, "disconnect") || equalsIgnoreCase(cleanCmd, "exit") ||
           equalsIgnoreCase(cleanCmd, "quit") ||
           equalsIgnoreCase(cleanCmd, "ver") || equalsIgnoreCase(cleanCmd, "version") ||
           equalsIgnoreCase(cleanCmd, "restart") || equalsIgnoreCase(cleanCmd, "reboot") ||
           equalsIgnoreCase(cleanCmd, "factory reset");
}

const char* skipCmdArg(const char *cmd, size_t prefixLen) {
    const char *arg = cmd + prefixLen;
    while (*arg == ' ' || *arg == '\t') arg++;
    return arg;
}

} // namespace

SerialCli::SerialCli(GpsManager &gpsManager, Calibration &calibration, GpsSimulator &simulator,
                     RoverLogManager *roverLogManager, BleSerialManager *bleManager,
                     QsoLogManager *qsoLogManager, TargetPresetManager *targetPresetManager)
    : gpsManager_(gpsManager),
      calibration_(calibration),
      simulator_(simulator),
      roverLogManager_(roverLogManager),
      qsoLogManager_(qsoLogManager),
      targetPresetManager_(targetPresetManager),
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
      targetGridCb_(nullptr),
      targetGridContext_(nullptr),
      qsoLoggedCb_(nullptr),
      qsoLoggedContext_(nullptr),
      outputFn_(nullptr),
      outputContext_(nullptr),
      prepareRestartCb_(nullptr),
      prepareRestartContext_(nullptr),
      restartCb_(nullptr),
      restartContext_(nullptr),
      pendingRoverClear_(false),
      pendingRoverClearSource_(CLI_SOURCE_USB),
      pendingQsoClear_(false),
      pendingQsoClearSource_(CLI_SOURCE_USB),
      pendingQsoInput_(false),
      pendingQsoSource_(CLI_SOURCE_USB),
      pendingFactoryReset_(false),
      pendingFactoryResetSource_(CLI_SOURCE_USB),
      pendingFactoryResetExec_(false),
      factoryResetInProgress_(false),
      pendingExportFormat_(ExportFormat::None),
      pendingExportIndex_(0),
      pendingExportSource_(CLI_SOURCE_USB),
      pendingAudit_(false),
      pendingAuditSource_(CLI_SOURCE_USB),
      pendingConfig_(false),
      pendingConfigSource_(CLI_SOURCE_USB),
      pendingConfigBaselineCm_(0.0f),
      pendingConfigTolCm_(2.5f)
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
        if (self->pendingConfig_) {
            self->pendingConfig_ = false;
            CliSource prevSource = self->getActiveSource();
            self->setActiveSource(self->pendingConfigSource_);
            if (state == GpsTxState::Available) {
                if (self->pendingConfigBaselineCm_ > 0.0f) {
                    self->printFormat("[GPS CONFIG] Initiating configuration pipeline (Baseline: %.1f cm, Tol: %.1f cm)...\r\n",
                                      self->pendingConfigBaselineCm_, self->pendingConfigTolCm_);
                } else {
                    self->println("[GPS CONFIG] Initiating configuration pipeline (Standard 5 Hz $KSXT)...");
                }
                self->gpsManager_.startConfiguration(self->pendingConfigBaselineCm_, self->pendingConfigTolCm_, true, 0);
            } else if (state == GpsTxState::Unavailable) {
                self->println("[GPS] TX wire not detected. Connect GPIO 43 to UM982 RX to configure.");
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
    pendingRoverClearSource_ = CLI_SOURCE_USB;
    pendingQsoClear_ = false;
    pendingQsoClearSource_ = CLI_SOURCE_USB;
    pendingQsoInput_ = false;
    pendingQsoSource_ = CLI_SOURCE_USB;
    pendingFactoryReset_ = false;
    pendingFactoryResetSource_ = CLI_SOURCE_USB;
    pendingFactoryResetExec_ = false;
    factoryResetInProgress_ = false;
    pendingExportFormat_ = ExportFormat::None;
    pendingExportIndex_ = 0;
    pendingExportSource_ = CLI_SOURCE_USB;
    pendingExportCall_.clear();
    pendingAudit_ = false;
    pendingAuditSource_ = CLI_SOURCE_USB;
    pendingConfig_ = false;
    pendingConfigSource_ = CLI_SOURCE_USB;
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
    bleLine_.reset();
    if (pendingQsoInput_ && pendingQsoSource_ == CLI_SOURCE_BLE) {
        pendingQsoInput_ = false;
    }
    if (pendingRoverClear_ && pendingRoverClearSource_ == CLI_SOURCE_BLE) {
        pendingRoverClear_ = false;
    }
    if (pendingQsoClear_ && pendingQsoClearSource_ == CLI_SOURCE_BLE) {
        pendingQsoClear_ = false;
    }
    if (pendingFactoryReset_ && pendingFactoryResetSource_ == CLI_SOURCE_BLE) {
        pendingFactoryReset_ = false;
    }
    if (pendingExportFormat_ != ExportFormat::None && pendingExportSource_ == CLI_SOURCE_BLE) {
        pendingExportFormat_ = ExportFormat::None;
    }
    if (pendingAudit_ && pendingAuditSource_ == CLI_SOURCE_BLE) {
        pendingAudit_ = false;
    }
    if (pendingConfig_ && pendingConfigSource_ == CLI_SOURCE_BLE) {
        pendingConfig_ = false;
    }
    if (auditState_.inProgress && auditState_.source == CLI_SOURCE_BLE) {
        auditState_.source = CLI_SOURCE_USB;
    }
    activeSource_ = CLI_SOURCE_USB;
}

void SerialCli::setOriginGrid(const char *grid) {
    if (grid) {
        std::strncpy(fallbackNavContext_.originGrid, grid, sizeof(fallbackNavContext_.originGrid) - 1);
        fallbackNavContext_.originGrid[sizeof(fallbackNavContext_.originGrid) - 1] = '\0';
    } else {
        fallbackNavContext_.originGrid[0] = '\0';
    }
}

void SerialCli::setCurrentGrid(const char *grid) {
    if (grid) {
        std::strncpy(fallbackNavContext_.currentGrid, grid, sizeof(fallbackNavContext_.currentGrid) - 1);
        fallbackNavContext_.currentGrid[sizeof(fallbackNavContext_.currentGrid) - 1] = '\0';
    } else {
        fallbackNavContext_.currentGrid[0] = '\0';
    }
}

void SerialCli::setTargetGrid(const char *grid) {
    if (grid) {
        std::strncpy(fallbackNavContext_.targetGrid, grid, sizeof(fallbackNavContext_.targetGrid) - 1);
        fallbackNavContext_.targetGrid[sizeof(fallbackNavContext_.targetGrid) - 1] = '\0';
    } else {
        fallbackNavContext_.targetGrid[0] = '\0';
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
    consoleWrite(str);
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
    if (pendingFactoryResetExec_ || factoryResetInProgress_) return;
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    for (size_t i = 0; i < len; ++i) {
        if (bleLine_.feed(data[i]) == LineBuffer::LineReady) {
            char cmdCopy[LineBuffer::kCapacity];
            std::snprintf(cmdCopy, sizeof(cmdCopy), "%s", bleLine_.c_str());
            bleLine_.clear();
            executeCommand(cmdCopy, nowMs, CLI_SOURCE_BLE);
            if (pendingFactoryResetExec_ || factoryResetInProgress_) {
                bleLine_.reset();
                return;
            }
        }
    }
}

void SerialCli::update(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif

    if (factoryResetInProgress_) {
        return;
    }

    if (pendingFactoryResetExec_) {
        pendingFactoryResetExec_ = false;
        doFactoryReset(nowMs);
        return;
    }

    if (bleManager_) {
        bleManager_->update(nowMs);
        if (pendingFactoryResetExec_) {
            pendingFactoryResetExec_ = false;
            doFactoryReset(nowMs);
            return;
        }
        if (factoryResetInProgress_) {
            return;
        }
    }

    if (pendingExportFormat_ != ExportFormat::None) {
        servicePendingExport(true);
    }

    if (auditState_.inProgress) {
        if (nowMs - auditState_.startTimeMs >= 4000) {
            renderAuditReport();
        }
    }

#if defined(ARDUINO)
    while (Serial.available()) {
        char c = (char)Serial.read();
        bool echo = usbLine_.empty() ? (c != '$') : (usbLine_.c_str()[0] != '$');
        switch (usbLine_.feed(c)) {
            case LineBuffer::Appended:
                if (echo) Serial.write(c);
                break;
            case LineBuffer::Erased:
                Serial.print("\b \b");
                break;
            case LineBuffer::Cleared:
                Serial.println("^C");
                printPrompt();
                break;
            case LineBuffer::LineReady: {
                if (usbLine_.c_str()[0] != '$') {
                    Serial.println();
                }
                char cmdCopy[LineBuffer::kCapacity];
                std::snprintf(cmdCopy, sizeof(cmdCopy), "%s", usbLine_.c_str());
                usbLine_.clear();
                executeCommand(cmdCopy, nowMs, CLI_SOURCE_USB);
                if (factoryResetInProgress_ || pendingFactoryResetExec_) {
                    return;
                }
                break;
            }
            case LineBuffer::Ignored:
                break;
        }
    }
#else
    (void)nowMs;
#endif
}

void SerialCli::executeCommand(const char *cmdLine, unsigned long nowMs, CliSource source) {
    if (factoryResetInProgress_) return;
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
#endif
    activeSource_ = source;
    if (!cmdLine) return;

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

    // Strip speech-to-text (e.g. Gboard) sentence punctuation while preserving
    // decimal numbers (e.g. 10368.1), standalone '?', hyphens, and raw 'gps send' / 'ble suffix' payloads.
    if (len > 0 && !startsWithIgnoreCase(cleanCmd, "gps send") && !startsWithIgnoreCase(cleanCmd, "ble suffix")) {
        size_t writeIdx = 0;
        for (size_t i = 0; i < len; ++i) {
            char c = cleanCmd[i];
            bool isPunctSpace = false;
            if (c == ' ' || c == '\t' || c == ',' || c == '!' || c == ';') {
                isPunctSpace = true;
            } else if (c == '.' || c == ':') {
                bool betweenDigits = (i > 0 && i + 1 < len &&
                                      std::isdigit(static_cast<unsigned char>(cleanCmd[i - 1])) &&
                                      std::isdigit(static_cast<unsigned char>(cleanCmd[i + 1])));
                isPunctSpace = !betweenDigits;
            } else if (c == '?') {
                bool standalone = ((i == 0 || cleanCmd[i - 1] == ' ' || cleanCmd[i - 1] == '\t') &&
                                   (i + 1 == len || cleanCmd[i + 1] == ' ' || cleanCmd[i + 1] == '\t'));
                isPunctSpace = !standalone;
            }

            if (isPunctSpace) {
                if (writeIdx > 0 && cleanCmd[writeIdx - 1] != ' ') {
                    cleanCmd[writeIdx++] = ' ';
                }
            } else {
                cleanCmd[writeIdx++] = c;
            }
        }
        if (writeIdx > 0 && cleanCmd[writeIdx - 1] == ' ') {
            writeIdx--;
        }
        cleanCmd[writeIdx] = '\0';
        len = writeIdx;

        // Strip stray trailing "send" or "sent" from voice keyboard submission (e.g. Gboard),
        // while preserving "gps send ...".
        if (len > 5 && !startsWithIgnoreCase(cleanCmd, "gps send") &&
            (equalsIgnoreCase(&cleanCmd[len - 5], " send") ||
             equalsIgnoreCase(&cleanCmd[len - 5], " sent"))) {
            len -= 5;
            cleanCmd[len] = '\0';
        }
    }

    if (len == 0) {
        printPrompt();
        return;
    }

    if (pendingRoverClear_ && activeSource_ == pendingRoverClearSource_) {
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

    if (pendingQsoClear_ && activeSource_ == pendingQsoClearSource_) {
        pendingQsoClear_ = false;
        if (equalsIgnoreCase(cleanCmd, "yes") || equalsIgnoreCase(cleanCmd, "y") ||
            equalsIgnoreCase(cleanCmd, "confirm")) {
            doLogClear();
        } else {
            println("[QSO LOG] QSO clear aborted.");
        }
        printPrompt();
        return;
    }

    if (pendingFactoryReset_ && activeSource_ == pendingFactoryResetSource_) {
        pendingFactoryReset_ = false;
        if (equalsIgnoreCase(cleanCmd, "yes")) {
#if defined(ARDUINO)
            if (activeSource_ == CLI_SOURCE_BLE) {
                // Defer NVS erase + restart to loopTask (Core 1) instead of running inside BLE callback (Core 0)
                pendingFactoryResetExec_ = true;
                return;
            }
#endif
            doFactoryReset(nowMs);
            return;
        }
        println("[SYSTEM] Factory reset aborted.");
        printPrompt();
        return;
    }

    if (pendingQsoInput_ && activeSource_ == pendingQsoSource_) {
        pendingQsoInput_ = false;
        if (equalsIgnoreCase(cleanCmd, "cancel")) {
            println("[QSO] Cancelled.");
            printPrompt();
            return;
        }
        if (!isKnownCommand(cleanCmd)) {
            handleLogCommand(cleanCmd, nowMs);
            printPrompt();
            return;
        }
    }

    if (startsWithIgnoreCase(cleanCmd, "rover")) {
        handleRoverCommand(skipCmdArg(cleanCmd, 5), nowMs);
    } else if (startsWithIgnoreCase(cleanCmd, "log") &&
               (cleanCmd[3] == '\0' || cleanCmd[3] == ' ' || cleanCmd[3] == '\t')) {
        handleLogCommand(skipCmdArg(cleanCmd, 3), nowMs);
    } else if (startsWithIgnoreCase(cleanCmd, "target")) {
        handleTargetCommand(skipCmdArg(cleanCmd, 6), nowMs);
    } else if (startsWithIgnoreCase(cleanCmd, "sim")) {
        handleSimCommand(skipCmdArg(cleanCmd, 3), nowMs);
    } else if (equalsIgnoreCase(cleanCmd, "help") || startsWithIgnoreCase(cleanCmd, "help ")) {
        handleHelpCommand(skipCmdArg(cleanCmd, 4));
    } else if (std::strcmp(cleanCmd, "?") == 0 || startsWithIgnoreCase(cleanCmd, "? ")) {
        handleHelpCommand(skipCmdArg(cleanCmd, 1));
    } else if (startsWithIgnoreCase(cleanCmd, "gps")) {
        const char *arg = skipCmdArg(cleanCmd, 3);
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
        handleBleCommand(skipCmdArg(cleanCmd, 3));
    } else if (startsWithIgnoreCase(cleanCmd, "cal")) {
        handleCalCommand(skipCmdArg(cleanCmd, 3));
    } else if (equalsIgnoreCase(cleanCmd, "disconnect") || equalsIgnoreCase(cleanCmd, "exit") ||
               equalsIgnoreCase(cleanCmd, "quit")) {
        if (activeSource_ == CLI_SOURCE_BLE && bleManager_) {
            println("Disconnecting BLE connection...");
            bleManager_->disconnect();
        } else {
            println("[CLI] 'disconnect' applies to active wireless Bluetooth sessions.");
        }
    } else if (equalsIgnoreCase(cleanCmd, "ver") || equalsIgnoreCase(cleanCmd, "version")) {
        printFormat("GNSS Compass %s (Built %s %s)\r\n", FW_VERSION, __DATE__, __TIME__);
    } else if (equalsIgnoreCase(cleanCmd, "factory reset")) {
        pendingFactoryReset_ = true;
        pendingFactoryResetSource_ = activeSource_;
        println("[SYSTEM] WARNING: This will erase ALL saved NVS data:");
        println("  - Antenna calibration offset");
        println("  - Custom BLE device name suffix");
        println("  - Rover stop history & capacity");
        println("  - Contest QSO log & settings");
        println("  - Target grid presets & capacity");
        println("Type 'yes' to erase all saved data and reboot, or anything else to abort.");
    } else if (equalsIgnoreCase(cleanCmd, "restart") || equalsIgnoreCase(cleanCmd, "reboot")) {
        println("[SYSTEM] Restarting ESP32 controller...");
        if (bleManager_) {
            bleManager_->flushAndDisconnect(nowMs);
        }
        if (prepareRestartCb_) {
            prepareRestartCb_(prepareRestartContext_);
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

    if (!pendingAudit_ && !auditState_.inProgress && pendingExportFormat_ == ExportFormat::None) {
        printPrompt();
    }
}

void SerialCli::handleHelpCommand(const char *arg) {
    if (!arg || *arg == '\0') {
        printHelp();
        return;
    }
    if (equalsIgnoreCase(arg, "gps")) {
        printGpsHelp();
    } else if (equalsIgnoreCase(arg, "rover")) {
        printRoverHelp();
    } else if (equalsIgnoreCase(arg, "target")) {
        printTargetHelp();
    } else if (equalsIgnoreCase(arg, "log")) {
        printLogHelp();
    } else if (equalsIgnoreCase(arg, "cal")) {
        printCalHelp();
    } else if (equalsIgnoreCase(arg, "sim")) {
        printSimHelp();
    } else if (equalsIgnoreCase(arg, "ble")) {
        println("Usage: ble [status] | ble suffix [<val>|reset] | ble diag on|off");
    } else {
        printFormat("Unknown help topic: '%s'.\r\n", arg);
        printHelp();
    }
}

void SerialCli::printHelp() {
    println();
    println("=== GNSS Compass Serial Commands ===");
    println("  status (or gps)       - Live GNSS receiver stream & link diagnostic");
    println("  gps [status|probe|config|send|check]");
    println("                        - UM982 GNSS receiver setup and diagnostics");
    println("  rover [status|start|stop|history|undo|max|clear]");
    println("                        - Rover transit tracking");
    println("  target <grid|index>   - Set active target grid or select saved preset #");
    println("  target [add|save|select|del|list|max|clear]");
    println("                        - Manage saved target grid presets");
    println("  log <call> [band/freq]- Voice-friendly contest QSO logging (10 GHz default)");
    println("  log [undo|del|list|stats|export|mycall|default|clear]");
    println("                        - QSO log management and contest export");
    println("  test (or demo)        - Run visual display self-test demo");
    println("  cal [snap|zero|<val>] - View or configure antenna calibration");
    println("  sim [on|off|hdg|pos|grid|rover|status]");
    println("                        - GPS simulation commands");
    println("  exit (or disconnect)  - Terminate wireless BLE connection cleanly");
    println("  restart (or reboot)   - Restart the ESP32 controller");
    println("  factory reset         - Erase all NVS settings/logs and reboot (requires 'yes')");
    println("  version (or ver)      - Display firmware build version and compile time");
    println("  ble [diag on|off]     - BLE link status / toggle BLE RF diagnostics");
    println("  ble suffix [<val>|reset]");
    println("                        - Configure BLE device name suffix (max 16 chars)");
    println("  $<NMEA/KSXT sentence> - Inject raw GNSS sentence directly");
    println("  help (or ?)           - Show this help menu");
    println("====================================");
    println();
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

void SerialCli::handleBleCommand(const char *arg) {
    if (!bleManager_) {
        println("[BLE] Bluetooth not available");
        return;
    }
    if (equalsIgnoreCase(arg, "diag on")) {
        bleManager_->setDiagEnabled(true);
        println("[BLE] Diagnostics ON (RSSI every 3 s while connected, main-loop stall reports; USB serial)");
    } else if (equalsIgnoreCase(arg, "diag off")) {
        bleManager_->setDiagEnabled(false);
        println("[BLE] Diagnostics OFF");
    } else if (startsWithIgnoreCase(arg, "suffix")) {
        const char *sub = arg + 6;
        while (*sub == ' ' || *sub == '\t') sub++;
        if (*sub == '\0') {
            if (bleManager_->isCustomSuffix()) {
                printFormat("[BLE] Device name: %s (Custom suffix: %s)\r\n",
                            bleManager_->getDeviceName(), bleManager_->getDeviceSuffix());
            } else {
                printFormat("[BLE] Device name: %s (Default MAC suffix: %s)\r\n",
                            bleManager_->getDeviceName(), bleManager_->getDeviceSuffix());
            }
        } else if (equalsIgnoreCase(sub, "reset") || equalsIgnoreCase(sub, "default")) {
            bleManager_->resetDeviceSuffix(true);
            printFormat("[BLE] Device name reset to default: %s\r\n", bleManager_->getDeviceName());
            println("[BLE] Saved to non-volatile storage.");
            println("[BLE] Note: If your phone still displays the old name, toggle your phone's Bluetooth off and on to clear its scan cache.");
        } else {
            size_t len = std::strlen(sub);
            while (len > 0 && (sub[len - 1] == ' ' || sub[len - 1] == '\t' ||
                               sub[len - 1] == '\r' || sub[len - 1] == '\n')) {
                len--;
            }
            if (len > BleSerialManager::kMaxSuffixLength) {
                println("[BLE ERROR] Suffix too long (maximum 16 characters).");
                return;
            }
            char cleanSuffix[BleSerialManager::kMaxSuffixLength + 1];
            std::memcpy(cleanSuffix, sub, len);
            cleanSuffix[len] = '\0';

            bool valid = true;
            for (size_t i = 0; i < len; ++i) {
                char c = cleanSuffix[i];
                if (c <= 32 || c >= 127) {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                println("[BLE ERROR] Invalid suffix. Use letters, numbers, hyphens or underscores (no spaces).");
                return;
            }

            if (bleManager_->setDeviceSuffix(cleanSuffix, true)) {
                printFormat("[BLE] Device name updated to: %s\r\n", bleManager_->getDeviceName());
                println("[BLE] Saved to non-volatile storage.");
                println("[BLE] Note: If your phone still displays the old name, toggle your phone's Bluetooth off and on to clear its scan cache.");
            } else {
                println("[BLE ERROR] Failed to set device suffix.");
            }
        }
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
    } else if (equalsIgnoreCase(arg, "help") || std::strcmp(arg, "?") == 0) {
        println("Usage: ble [status] | ble suffix [<val>|reset] | ble diag on|off");
    } else {
        println("Usage: ble [status] | ble suffix [<val>|reset] | ble diag on|off");
    }
}

void SerialCli::printCalHelp() {
    println();
    println("=== Antenna Calibration Commands ===");
    println("  cal [status|show]     - Display current calibration offset & heading");
    println("  cal snap              - Align antenna heading to current target bearing");
    println("  cal zero (or reset)   - Reset calibration offset to 0.0 deg");
    println("  cal [set] <deg>       - Set manual antenna offset in degrees");
}

void SerialCli::handleCalCommand(const char *arg) {
    if (arg && (equalsIgnoreCase(arg, "help") || std::strcmp(arg, "?") == 0)) {
        printCalHelp();
        return;
    }

    if (!arg || *arg == '\0' || equalsIgnoreCase(arg, "status") || equalsIgnoreCase(arg, "show")) {
        printCalibrationStatus();
        return;
    }

    if (equalsIgnoreCase(arg, "zero") || equalsIgnoreCase(arg, "reset") || std::strcmp(arg, "0") == 0) {
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

    float val = 0.0f;
    if (!parseStrictFloat(numStr, val)) {
        printFormat("[CAL ERROR] Invalid offset '%s'. Usage: cal [snap|zero|<deg>]\r\n", numStr);
        return;
    }
    calibration_.setOffset(val);
    if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
    printFormat("[CAL] Offset set to %+5.1f deg\r\n", calibration_.getOffset());
}

void SerialCli::printSimHelp() {
    println();
    println("=== GPS Simulation Commands ===");
    println("  sim on / sim off      - Enable or disable simulation mode");
    println("  sim status            - Show simulation state & injected values");
    println("  sim hdg <deg> [fixed|float|lost]");
    println("                        - Inject simulated heading & quality");
    println("  sim pos <lat> <lon>   - Inject simulated latitude & longitude");
    println("  sim grid <grid>       - Inject simulated Maidenhead grid (4/6/8-char)");
    println("  sim rover [origin_grid] <dist_km>");
    println("                        - Simulate rover transit displacement");
    println("  sim time <hh:mm>      - Inject simulated UTC time");
}

void SerialCli::handleSimCommand(const char *arg, unsigned long nowMs) {
    if (arg && (equalsIgnoreCase(arg, "help") || std::strcmp(arg, "?") == 0)) {
        printSimHelp();
        return;
    }

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
        const char *p = skipSubcommandPrefix(arg, 3);
        float h = 0.0f;
        const char *next = nullptr;
        if (!parseFloatToken(p, h, next)) {
            println("[SIM ERROR] Usage: sim hdg <deg> [fixed|float|lost]");
            return;
        }
        BearingQuality q = BEARING_FIXED;

        while (*next == ' ' || *next == '\t') next++;
        if (*next != '\0') {
            if (equalsIgnoreCase(next, "float")) q = BEARING_FLOAT;
            else if (equalsIgnoreCase(next, "lost") || equalsIgnoreCase(next, "invalid")) q = BEARING_LOST;
            else q = BEARING_FIXED;
        }

        gpsManager_.setHeading(h, q, nowMs);
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);

        const char *qStr = (q == BEARING_FIXED) ? "FIXED" : ((q == BEARING_FLOAT) ? "FLOAT" : "LOST");
        printFormat("[SIM] Heading set to %5.1f deg (Quality: %s)\r\n", h, qStr);
        return;
    }

    if (startsWithIgnoreCase(arg, "pos")) {
        const char *p = skipSubcommandPrefix(arg, 3);
        char *endPtr = nullptr;
        double lat = std::strtod(p, &endPtr);
        if (endPtr && *endPtr != '\0') {
            double lon = std::strtod(endPtr, nullptr);
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
        size_t gLen = std::strlen(g);
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
        char ogBuf[10] = "";
        float distKmF = 0.0f;
        const char *sp = std::strchr(p, ' ');
        if (sp) {
            size_t ogLen = static_cast<size_t>(sp - p);
            if (ogLen >= sizeof(ogBuf) || !parseStrictFloat(sp + 1, distKmF) || distKmF < 0.0f) {
                println("[SIM ERROR] Usage: sim rover [origin_grid] <dist_km>");
                return;
            }
            std::strncpy(ogBuf, p, ogLen);
            ogBuf[ogLen] = '\0';
        } else if (!parseStrictFloat(p, distKmF) || distKmF < 0.0f) {
            println("[SIM ERROR] Usage: sim rover [origin_grid] <dist_km>");
            return;
        }
        double distKm = static_cast<double>(distKmF);

        if (roverSimCb_) {
            roverSimCb_(ogBuf, distKm, roverSimContext_);
            return;
        }

        GeoPoint origin = gpsManager_.hasLocation() ? gpsManager_.getLocation() : GeoPoint(38.35412, -119.45821, true);
        GeoPoint simulated = GpsSimulator::calculateRoverDisplacement(origin, distKm, 45.0);
        gpsManager_.setLocation(simulated, nowMs);
        if (stateChangedCb_) stateChangedCb_(stateChangedContext_);
        printFormat("[SIM ROVER] Distance: %5.1f km\r\n", distKm);
        return;
    }

    if (startsWithIgnoreCase(arg, "time")) {
        const char *t = arg + 4;
        while (*t == ' ' || *t == '\t') t++;
        const char *colon = std::strchr(t, ':');
        if (colon) {
            int h = std::atoi(t);
            int m = std::atoi(colon + 1);
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

    println("[SIM ERROR] Unknown sim command.");
    printSimHelp();
}

void SerialCli::doFactoryReset(unsigned long nowMs) {
    factoryResetInProgress_ = true;
    pendingFactoryReset_ = false;
    pendingFactoryResetExec_ = false;
    usbLine_.reset();
    bleLine_.reset();

    println("[SYSTEM] Factory reset complete. Restarting ESP32 controller...");
    if (bleManager_) {
        bleManager_->flushAndDisconnect(nowMs);
    }

    if (prepareRestartCb_) {
        prepareRestartCb_(prepareRestartContext_);
    }

#if defined(ARDUINO)
    // Give the LCD backlight a moment to go dark before flash writes.
    // Do not call Serial.flush(): on ESP32-S3 USB CDC it can block when no host is reading.
    delay(20);

    // Erase all application NVS namespaces without calling nvs_flash_deinit() while Core 0
    // Bluedroid / system tasks are still running, and without erasing 512 KB of raw flash
    // sectors (which starves RGB LCD DMA and risks watchdog timeouts).
    const char *kNamespaces[] = {
        "compass",
        ROVER_CFG_NAMESPACE,
        ROVER_LOG_NAMESPACE,
        QSO_CFG_NAMESPACE,
        QSO_LOG_NAMESPACE,
        TARGET_CFG_NAMESPACE,
        TARGET_LOG_NAMESPACE
    };
    for (size_t i = 0; i < sizeof(kNamespaces) / sizeof(kNamespaces[0]); ++i) {
        nvs_handle_t handle = 0;
        if (nvs_open(kNamespaces[i], NVS_READWRITE, &handle) == ESP_OK) {
            nvs_erase_all(handle);
            nvs_commit(handle);
            nvs_close(handle);
        }
    }
#else
    calibration_.reset();
    if (bleManager_) {
        bleManager_->resetDeviceSuffix(true);
    }
    if (roverLogManager_) {
        roverLogManager_->clearHistory();
        roverLogManager_->setMaxStops(DEFAULT_MAX_ROVER_STOPS, true);
    }
    if (qsoLogManager_) {
        qsoLogManager_->resetDefaults();
    }
    if (targetPresetManager_) {
        targetPresetManager_->clearAll();
        targetPresetManager_->setMaxTargets(DEFAULT_MAX_TARGET_PRESETS, true);
    }
    NvsConfig::resetHostStorage();
    if (stateChangedCb_) {
        stateChangedCb_(stateChangedContext_);
    }
#endif

    if (restartCb_) {
        restartCb_(restartContext_);
    } else {
#if defined(ARDUINO)
        delay(100);
        esp_restart();
#endif
    }
}
