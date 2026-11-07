#pragma once

#include <cstdint>
#include <cstddef>
#include "../types.h"
#include "../gnss/gps_manager.h"
#include "../system/calibration.h"
#include "../gnss/simulator.h"

// Forward declaration of listener callback types
typedef void (*CliCommandOutputFn)(const char *str);
typedef void (*CliRunTestCallback)();
typedef void (*CliStateChangedCallback)();
typedef void (*CliRoverSimCallback)(const char *originGrid, double distKm);

/**
 * @brief Navigation domain context query for live CLI reports and calibration snap.
 */
struct CliNavContext {
    double targetBearing = 0.0;
    bool isRoverAlertActive = false;
    double roverDistanceMeters = 0.0;
};
typedef CliNavContext (*CliNavContextProviderFn)();

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
    void setRunTestCallback(CliRunTestCallback cb) { runTestCb_ = cb; }
    void setStateChangedCallback(CliStateChangedCallback cb) { stateChangedCb_ = cb; }
    void setRoverSimCallback(CliRoverSimCallback cb) { roverSimCb_ = cb; }
    void setOutputHandler(CliCommandOutputFn fn) { outputFn_ = fn; }

    // Navigation domain context provider (queries live values on-demand)
    void setNavigationContextProvider(CliNavContextProviderFn fn) { navContextFn_ = fn; }
    CliNavContext getNavContext() const {
        return navContextFn_ ? navContextFn_() : fallbackNavContext_;
    }

    // Direct context overrides (useful for standalone unit tests)
    void setTargetBearing(double bearingDeg) { fallbackNavContext_.targetBearing = bearingDeg; }
    double getTargetBearing() const { return getNavContext().targetBearing; }

    void setRoverAlertActive(bool active) { fallbackNavContext_.isRoverAlertActive = active; }
    bool isRoverAlertActive() const { return getNavContext().isRoverAlertActive; }

    void setRoverDistance(double distMeters) { fallbackNavContext_.roverDistanceMeters = distMeters; }
    double getRoverDistance() const { return getNavContext().roverDistanceMeters; }

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
    CliNavContext fallbackNavContext_;

    CliRunTestCallback runTestCb_;
    CliStateChangedCallback stateChangedCb_;
    CliRoverSimCallback roverSimCb_;
    CliCommandOutputFn outputFn_;

    char inputBuffer_[256];
    size_t inputIndex_;
};

