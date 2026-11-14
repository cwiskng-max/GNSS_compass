#pragma once

#include <cstdint>
#include <cstddef>
#include "../types.h"
#include "../gnss/gps_manager.h"
#include "../system/calibration.h"
#include "../gnss/simulator.h"
#include "../ui/rover_card.h"
#include "../system/rover_log_manager.h"
#include "../system/qso_entry.h"
#include "../system/qso_log_manager.h"
#include "ble_serial_manager.h"
#include "line_buffer.h"

// Forward declaration of listener callback types
typedef void (*CliCommandOutputFn)(const char *str, void *context);
typedef void (*CliRunTestCallback)(void *context);
typedef void (*CliStateChangedCallback)(void *context);
typedef void (*CliRoverSimCallback)(const char *originGrid, double distKm, void *context);
typedef void (*CliRoverActionCallback)(void *context, unsigned long nowMs);
typedef void (*CliRestartCallback)(void *context);
typedef void (*CliTargetGridCallback)(const char *grid, unsigned long nowMs, void *context);
typedef void (*CliQsoLoggedCallback)(const QsoEntry &entry, size_t qsoIndex, unsigned long nowMs, void *context);

/**
 * @brief Input source of the CLI command
 */
enum CliSource {
    CLI_SOURCE_USB = 0,
    CLI_SOURCE_BLE,
    CLI_SOURCE_INTERNAL
};

/**
 * @brief Navigation domain context query for live CLI reports, calibration snap,
 *        and Rover transit status.
 */
struct CliNavContext {
    double targetBearing = 0.0;
    bool isRoverMode = false;
    double roverDistanceMeters = 0.0;
    double originLat = 0.0;
    double originLon = 0.0;
    char originGrid[9] = {0};
    char currentGrid[9] = {0};
    char targetGrid[9] = {0};
    RoverState roverState = RoverState::InTransitUnder16k;
    double lastValidDisplacementKm = 0.0;
    bool isGpsFresh = false;
};
typedef CliNavContext (*CliNavContextProviderFn)(void *context);

/**
 * @brief Host USB Serial CLI Parser for GNSS diagnosis, antenna calibration,
 *        Rover Mode contest tracking, simulation injection, and visual self-test triggering.
 */
class SerialCli {
public:
    SerialCli(GpsManager &gpsManager, Calibration &calibration, GpsSimulator &simulator,
              RoverLogManager *roverLogManager = nullptr,
              BleSerialManager *bleManager = nullptr,
              QsoLogManager *qsoLogManager = nullptr);

    // Initializes CLI state
    void begin();

    // Non-blocking character ingest from host USB Serial
    void update(unsigned long nowMs = 0);

    // Ingest data from wireless BLE connection
    void handleBleInput(const char *data, size_t len, unsigned long nowMs = 0);

    // BLE link came up or went down: drop any partial BLE input line and send replies to USB
    void handleBleConnectionChanged(bool connected);

    // Directly execute a command line (from serial stream, BLE, scripts, or unit tests)
    void executeCommand(const char *cmdLine, unsigned long nowMs = 0, CliSource source = CLI_SOURCE_USB);

    // Transport configuration
    void setBleManager(BleSerialManager *mgr);
    BleSerialManager* getBleManager() const { return bleManager_; }

    // Source of the most recent command; print() replies go there. Reset to USB when the
    // BLE link changes state.
    CliSource getActiveSource() const { return activeSource_; }
    void setActiveSource(CliSource src) { activeSource_ = src; }

    // Output routing:
    //   - print()/println()/printFormat() (private) reply to whoever sent the current command.
    //     BLE replies are mirrored to the USB console.
    //   - broadcast*() goes to the USB console and to the BLE client if one is connected.
    // An output handler set with setOutputHandler() replaces the USB console (Serial/stdout)
    // in both cases; BLE delivery is unaffected.
    void broadcast(const char *str);
    void broadcastln(const char *str = "");
    void broadcastFormat(const char *fmt, ...);

    // Callbacks & listeners
    void setRunTestCallback(CliRunTestCallback cb, void *context = nullptr) { runTestCb_ = cb; runTestContext_ = context; }
    void setRestartCallback(CliRestartCallback cb, void *context = nullptr) { restartCb_ = cb; restartContext_ = context; }
    void setStateChangedCallback(CliStateChangedCallback cb, void *context = nullptr) { stateChangedCb_ = cb; stateChangedContext_ = context; }
    void setRoverSimCallback(CliRoverSimCallback cb, void *context = nullptr) { roverSimCb_ = cb; roverSimContext_ = context; }
    void setOutputHandler(CliCommandOutputFn fn, void *context = nullptr) { outputFn_ = fn; outputContext_ = context; }

    // Navigation domain context provider (queries live values on-demand)
    void setNavigationContextProvider(CliNavContextProviderFn fn, void *context = nullptr) { navContextFn_ = fn; navContext_ = context; }
    CliNavContext getNavContext() const;

    // Direct context overrides (useful for standalone unit tests)
    void setTargetBearing(double bearingDeg) { fallbackNavContext_.targetBearing = bearingDeg; }
    double getTargetBearing() const;

    void setRoverMode(bool active) { fallbackNavContext_.isRoverMode = active; }
    bool isRoverMode() const;

    void setRoverDistance(double distMeters) { fallbackNavContext_.roverDistanceMeters = distMeters; }
    double getRoverDistance() const;

    void setOriginPoint(double lat, double lon) {
        fallbackNavContext_.originLat = lat;
        fallbackNavContext_.originLon = lon;
    }
    void setOriginGrid(const char *grid);
    void setCurrentGrid(const char *grid);
    void setTargetGrid(const char *grid);
    void setRoverState(RoverState state) { fallbackNavContext_.roverState = state; }
    void setLastValidDisplacementKm(double km) { fallbackNavContext_.lastValidDisplacementKm = km; }
    void setGpsFresh(bool fresh) { fallbackNavContext_.isGpsFresh = fresh; }

    // Rover Log & Actions
    void setRoverLogManager(RoverLogManager *mgr) { roverLogManager_ = mgr; }
    RoverLogManager* getRoverLogManager() const { return roverLogManager_; }

    // QSO Log & Actions
    void setQsoLogManager(QsoLogManager *mgr) { qsoLogManager_ = mgr; }
    QsoLogManager* getQsoLogManager() const { return qsoLogManager_; }

    void setQsoLoggedCallback(CliQsoLoggedCallback cb, void *context = nullptr) {
        qsoLoggedCb_ = cb;
        qsoLoggedContext_ = context;
    }

    void setRoverCallbacks(CliRoverActionCallback onStart,
                           CliRoverActionCallback onStop,
                           CliRoverActionCallback onUndo,
                           void *context = nullptr) {
        roverStartCb_ = onStart;
        roverStopCb_ = onStop;
        roverUndoCb_ = onUndo;
        roverActionContext_ = context;
    }

    void setTargetGridCallback(CliTargetGridCallback cb, void *context = nullptr) {
        targetGridCb_ = cb;
        targetGridContext_ = context;
    }

    // Formatted diagnostic reports
    void printHelp();
    void printGpsStatus(unsigned long nowMs = 0);
    void printCalibrationStatus();
    void printSimulationStatus(unsigned long nowMs = 0);
    void printRoverStatus(unsigned long nowMs = 0);
    void printRoverHistory();
    void printRoverHelp();
    void printPrompt();
    void handleTargetCommand(const char *arg, unsigned long nowMs = 0);
    void handleLogCommand(const char *arg, unsigned long nowMs = 0);
    void handleLogUndo();
    void handleLogDelete(const char *arg);
    void handleLogClear(const char *arg);
    void doLogClear();
    void handleLogList();
    void handleLogStats();
    void handleLogExport(const char *arg);
    void handleLogDefault(const char *arg);
    void handleLogCall(const char *arg);
    void printLogHelp();
    void printGpsHelp();
    void printSimHelp();
    void printCalHelp();

    // Receiver audit query & parsing
    struct GpsAuditState {
        bool inProgress = false;
        unsigned long startTimeMs = 0;
        unsigned long lastRespTimeMs = 0;
        CliSource source = CLI_SOURCE_USB;
        bool receivedVersion = false;
        bool receivedMode = false;
        bool receivedUniloglist = false;
        bool receivedHeadingStatus = false;
        int expectedLogs = -1;
        int collectedLogs = 0;
        char model[32] = {0};
        char fwVersion[48] = {0};
        char hwPn[32] = {0};
        char serialNum[48] = {0};
        char buildDate[16] = {0};
        char mode[32] = {0};
        char com1Logs[128] = {0};
        float baselineLengthCm = 0.0f;
        float baselineTolCm = 0.0f;
        bool hasBaseline = false;

        void reset(unsigned long nowMs, CliSource src) {
            inProgress = true;
            startTimeMs = nowMs;
            lastRespTimeMs = nowMs;
            source = src;
            receivedVersion = false;
            receivedMode = false;
            receivedUniloglist = false;
            receivedHeadingStatus = false;
            expectedLogs = -1;
            collectedLogs = 0;
            model[0] = '\0';
            fwVersion[0] = '\0';
            hwPn[0] = '\0';
            serialNum[0] = '\0';
            buildDate[0] = '\0';
            mode[0] = '\0';
            com1Logs[0] = '\0';
            baselineLengthCm = 0.0f;
            baselineTolCm = 0.0f;
            hasBaseline = false;
        }

        bool isComplete() const {
            return receivedVersion && receivedMode && receivedUniloglist && receivedHeadingStatus;
        }
    };

private:
    void print(const char *str);
    void println(const char *str = "");
    void printFormat(const char *fmt, ...);
    void consoleWrite(const char *str);
    static void printSink(const char *str, void *ctx);
    static void broadcastSink(const char *str, void *ctx);

    void registerGpsCallbacks();
    void attachBle();

    void startGpsAudit(unsigned long nowMs);
    bool handleAuditResponse(const char *resp);
    void renderAuditReport();

    void handleSimCommand(const char *arg, unsigned long nowMs);
    void handleCalCommand(const char *arg);
    void handleGpsCommand(const char *arg, unsigned long nowMs);
    void handleRoverCommand(const char *arg, unsigned long nowMs);
    void handleRoverStart(unsigned long nowMs);
    void handleRoverStop(unsigned long nowMs);
    void handleRoverUndo(unsigned long nowMs);
    void handleRoverMax(const char *arg);
    void handleRoverClear(const char *arg);
    void doRoverClear();

    GpsManager &gpsManager_;
    Calibration &calibration_;
    GpsSimulator &simulator_;
    RoverLogManager *roverLogManager_;
    QsoLogManager *qsoLogManager_;
    BleSerialManager *bleManager_;
    CliSource activeSource_;

    CliNavContextProviderFn navContextFn_;
    void *navContext_;
    CliNavContext fallbackNavContext_;

    CliRunTestCallback runTestCb_;
    void *runTestContext_;
    CliStateChangedCallback stateChangedCb_;
    void *stateChangedContext_;
    CliRoverSimCallback roverSimCb_;
    void *roverSimContext_;
    CliRoverActionCallback roverStartCb_;
    CliRoverActionCallback roverStopCb_;
    CliRoverActionCallback roverUndoCb_;
    void *roverActionContext_;
    CliTargetGridCallback targetGridCb_;
    void *targetGridContext_;
    CliQsoLoggedCallback qsoLoggedCb_;
    void *qsoLoggedContext_;
    CliCommandOutputFn outputFn_;
    void *outputContext_;
    CliRestartCallback restartCb_;
    void *restartContext_;

    bool pendingRoverClear_;
    bool pendingQsoClear_;
    bool pendingQsoInput_;
    CliSource pendingQsoSource_;
    bool pendingAudit_;
    CliSource pendingAuditSource_;
    bool pendingConfig_;
    CliSource pendingConfigSource_;
    float pendingConfigBaselineCm_;
    float pendingConfigTolCm_;
    GpsAuditState auditState_;
    LineBuffer usbLine_;
    LineBuffer bleLine_;
};

