#include "serial_cli.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <cmath>

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

SerialCli::SerialCli(GpsManager &gpsManager, Calibration &calibration, GpsSimulator &simulator)
    : gpsManager_(gpsManager),
      calibration_(calibration),
      simulator_(simulator),
      navContextFn_(nullptr),
      navContext_(nullptr),
      runTestCb_(nullptr),
      runTestContext_(nullptr),
      stateChangedCb_(nullptr),
      stateChangedContext_(nullptr),
      roverSimCb_(nullptr),
      roverSimContext_(nullptr),
      outputFn_(nullptr),
      outputContext_(nullptr),
      inputIndex_(0)
{
    inputBuffer_[0] = '\0';
}

void SerialCli::begin() {
    inputIndex_ = 0;
    inputBuffer_[0] = '\0';
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

void SerialCli::print(const char *str) {
    if (!str) return;
    if (outputFn_) {
        outputFn_(str, outputContext_);
    } else {
#if defined(ARDUINO)
        Serial.print(str);
#else
        std::cout << str;
#endif
    }
}

void SerialCli::println(const char *str) {
    print(str);
    print("\r\n");
}

void SerialCli::printFormat(const char *fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    print(buf);
}

void SerialCli::update(unsigned long nowMs) {
#if defined(ARDUINO)
    if (!nowMs) nowMs = millis();
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (inputIndex_ > 0) {
                if (inputBuffer_[0] != '$') {
                    Serial.println(); // Terminal newline echo for interactive commands
                }
                inputBuffer_[inputIndex_] = '\0';
                executeCommand(inputBuffer_, nowMs);
                inputIndex_ = 0;
            }
        } else if (c == '\b' || c == 127) { // Backspace or DEL
            if (inputIndex_ > 0) {
                inputIndex_--;
                Serial.print("\b \b");
            }
        } else if (inputIndex_ < sizeof(inputBuffer_) - 1) {
            if (c >= 32 && c <= 126) {
                inputBuffer_[inputIndex_++] = c;
                if (inputBuffer_[0] != '$') {
                    Serial.write(c); // Terminal echo
                }
            }
        }
    }
#else
    (void)nowMs;
#endif
}

void SerialCli::executeCommand(const char *cmdLine, unsigned long nowMs) {
    if (!cmdLine) return;
    while (*cmdLine == ' ' || *cmdLine == '\t') cmdLine++;
    if (*cmdLine == '\0') return;

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

    if (startsWithIgnoreCase(cmdLine, "sim")) {
        const char *arg = cmdLine + 3;
        while (*arg == ' ' || *arg == '\t') arg++;
        handleSimCommand(arg, nowMs);
        return;
    }

    if (equalsIgnoreCase(cmdLine, "help") || strcmp(cmdLine, "?") == 0) {
        printHelp();
        return;
    }

    if (equalsIgnoreCase(cmdLine, "status") || equalsIgnoreCase(cmdLine, "gps") ||
        equalsIgnoreCase(cmdLine, "diag")   || equalsIgnoreCase(cmdLine, "s")   ||
        equalsIgnoreCase(cmdLine, "g")) {
        printGpsStatus(nowMs);
        return;
    }

    if (equalsIgnoreCase(cmdLine, "test") || equalsIgnoreCase(cmdLine, "demo") ||
        equalsIgnoreCase(cmdLine, "t")) {
        if (runTestCb_) {
            runTestCb_(runTestContext_);
        } else {
            println("[CLI] Test runner callback not registered.");
        }
        return;
    }

    if (startsWithIgnoreCase(cmdLine, "cal")) {
        const char *arg = cmdLine + 3;
        while (*arg == ' ' || *arg == '\t') arg++;
        handleCalCommand(arg);
        return;
    }

    printFormat("Unknown command: '%s'. Type 'help' for available commands.\r\n", cmdLine);
}

void SerialCli::printHelp() {
    println();
    println("=== GNSS Compass Serial Commands ===");
    println("  status (or gps)       - Live GNSS receiver stream diagnostic");
    println("  test (or demo)        - Run visual display self-test demo");
    println("  cal [snap|zero|<val>] - View or configure antenna calibration");
    println("  sim [on|off|hdg|pos|grid|rover|status] - GPS simulation commands");
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

