#include "serial_cli.h"
#include "serial_cli_helpers.h"
#include "../geo/maidenhead.h"
#include <cstdio>
#include <cstring>
#include <ctime>

using namespace cli_helpers;

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

    if (equalsIgnoreCase(arg, "history") || equalsIgnoreCase(arg, "list") ||
        equalsIgnoreCase(arg, "show") || equalsIgnoreCase(arg, "log")) {
        printRoverHistory();
        return;
    }

    if (equalsIgnoreCase(arg, "undo")) {
        handleRoverUndo(nowMs);
        return;
    }

    if (startsWithIgnoreCase(arg, "max")) {
        const char *p = skipSubcommandPrefix(arg, 3);
        handleRoverMax(p);
        return;
    }

    if (startsWithIgnoreCase(arg, "clear")) {
        const char *p = arg + 5;
        while (*p == ' ' || *p == '\t') p++;
        handleRoverClear(p);
        return;
    }

    if (equalsIgnoreCase(arg, "help") || std::strcmp(arg, "?") == 0) {
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
            std::snprintf(fallbackNavContext_.currentGrid, sizeof(fallbackNavContext_.currentGrid), "%s", fallbackNavContext_.originGrid);
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
            std::snprintf(fallbackNavContext_.originGrid, sizeof(fallbackNavContext_.originGrid), "%s", restored->grid);
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
    int val = 0;
    if (!parseStrictPositiveInt(arg, val)) {
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
    pendingRoverClearSource_ = activeSource_;
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
            std::snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02d %02d:%02d:%02d",
                          tmBuf.tm_year + 1900, tmBuf.tm_mon + 1, tmBuf.tm_mday,
                          tmBuf.tm_hour, tmBuf.tm_min, tmBuf.tm_sec);
        } else {
            int h = (st->timestamp / 3600) % 24;
            int m = (st->timestamp / 60) % 60;
            int s = st->timestamp % 60;
            std::snprintf(tsBuf, sizeof(tsBuf), "--- %02d:%02d:%02d UTC   ", h, m, s);
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
    println("  rover history (list/show/log) - Formatted ASCII table of contest location stops");
    println("  rover undo            - Discards last stop and auto-rearms Rover Mode");
    println("  rover max [n]         - Gets or sets max stops capacity (10-500) in NVS");
    println("  rover clear [confirm] - Clears rover location history in NVS");
    println("===================================");
    println();
}
