#pragma once

#include <cstdint>
#include <cstddef>
#include "../types.h"
#include "../gnss/gps_manager.h"
#include "../system/calibration.h"
#include "../gnss/simulator.h"

// Forward declaration of listener callback types
typedef void (*CliCommandOutputFn)(const char *str, void *context);
typedef void (*CliRunTestCallback)(void *context);
typedef void (*CliStateChangedCallback)(void *context);
typedef void (*CliRoverSimCallback)(const char *originGrid, double distKm, void *context);

/**
 * @brief Navigation domain context query for live CLI reports and calibration snap.
 */
struct CliNavContext {
    double targetBearing = 0.0;
    bool isRoverAlertActive = false;
    double roverDistanceMeters = 0.0;
};
typedef CliNavContext (*CliNavContextProviderFn)(void *context);

/**
 * @brief Host USB Serial CLI Parser for GNSS diagnosis, antenna calibration,
 *        simulation injection, and visual self-test triggering.
 */
class SerialCli {
public:
    SerialCli(GpsManager &gpsManager, Calibration &calibration, GpsSimulator &simulator);

    // Initializes CLI state
    void begin();

    // Non-blocking character ingest from host USB Serial
    void update(unsigned long nowMs = 0);

    // Directly execute a command line (from serial stream, scripts, or unit tests)
    void executeCommand(const char *cmdLine, unsigned long nowMs = 0);

    // Callbacks & listeners
    void setRunTestCallback(CliRunTestCallback cb, void *context = nullptr) { runTestCb_ = cb; runTestContext_ = context; }
    void setStateChangedCallback(CliStateChangedCallback cb, void *context = nullptr) { stateChangedCb_ = cb; stateChangedContext_ = context; }
    void setRoverSimCallback(CliRoverSimCallback cb, void *context = nullptr) { roverSimCb_ = cb; roverSimContext_ = context; }
    void setOutputHandler(CliCommandOutputFn fn, void *context = nullptr) { outputFn_ = fn; outputContext_ = context; }

    // Navigation domain context provider (queries live values on-demand)
    void setNavigationContextProvider(CliNavContextProviderFn fn, void *context = nullptr) { navContextFn_ = fn; navContext_ = context; }
    CliNavContext getNavContext() const;

    // Direct context overrides (useful for standalone unit tests)
    void setTargetBearing(double bearingDeg) { fallbackNavContext_.targetBearing = bearingDeg; }
    double getTargetBearing() const;

    void setRoverAlertActive(bool active) { fallbackNavContext_.isRoverAlertActive = active; }
    bool isRoverAlertActive() const;

    void setRoverDistance(double distMeters) { fallbackNavContext_.roverDistanceMeters = distMeters; }
    double getRoverDistance() const;

    // Formatted diagnostic reports
    void printHelp();
    void printGpsStatus(unsigned long nowMs = 0);
    void printCalibrationStatus();
    void printSimulationStatus(unsigned long nowMs = 0);

private:
    void print(const char *str);
    void println(const char *str = "");
    void printFormat(const char *fmt, ...);

    void handleSimCommand(const char *arg, unsigned long nowMs);
    void handleCalCommand(const char *arg);

    GpsManager &gpsManager_;
    Calibration &calibration_;
    GpsSimulator &simulator_;

    CliNavContextProviderFn navContextFn_;
    void *navContext_;
    CliNavContext fallbackNavContext_;

    CliRunTestCallback runTestCb_;
    void *runTestContext_;
    CliStateChangedCallback stateChangedCb_;
    void *stateChangedContext_;
    CliRoverSimCallback roverSimCb_;
    void *roverSimContext_;
    CliCommandOutputFn outputFn_;
    void *outputContext_;

    char inputBuffer_[256];
    size_t inputIndex_;
};

