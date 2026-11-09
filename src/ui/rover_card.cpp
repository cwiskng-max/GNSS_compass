#include "rover_card.h"
#include <cstdio>
#include <cmath>

RoverState evaluateRoverState(double displacementKm, bool isRevisit, bool isGpsFresh) {
    if (!isGpsFresh) {
        return RoverState::GpsLostStale;
    }
    if (displacementKm < ROVER_ALERT_THRESHOLD_KM) {
        return RoverState::InTransitUnder16k;
    }
    if (isRevisit) {
        return RoverState::QualifiedRevisitedGrid;
    }
    return RoverState::QualifiedNewGrid;
}

uint16_t getRoverStateTextColor(RoverState state) {
    switch (state) {
        case RoverState::InTransitUnder16k:
            return COLOR_ROVER_TRANSIT_RED;
        case RoverState::QualifiedNewGrid:
            return COLOR_ROVER_QUALIFIED_GREEN;
        case RoverState::QualifiedRevisitedGrid:
            return COLOR_ROVER_REVISIT_BLUE;
        case RoverState::GpsLostStale:
            return COLOR_ROVER_NOGPS_AMBER;
    }
    return COLOR_ROVER_TRANSIT_RED;
}

uint16_t calculateRoverDistanceColor(double displacementKm, bool isRevisit) {
    if (displacementKm < ROVER_ALERT_THRESHOLD_KM) {
        return COLOR_ROVER_TRANSIT_RED;
    }
    if (isRevisit) {
        return COLOR_ROVER_REVISIT_BLUE;
    }
    return COLOR_ROVER_QUALIFIED_GREEN;
}

const char* getRoverStateStatusWord(RoverState state) {
    switch (state) {
        case RoverState::InTransitUnder16k:
            return "TRANSIT  ";
        case RoverState::QualifiedNewGrid:
            return "QUALIFIED";
        case RoverState::QualifiedRevisitedGrid:
            return "REVISIT  ";
        case RoverState::GpsLostStale:
            return "NO GPS   ";
    }
    return "TRANSIT  ";
}

bool formatRoverCard(RoverState state, double displacementKm, char *outBuf, size_t bufSize) {
    if (!outBuf || bufSize < 17) {
        return false;
    }
    if (displacementKm < 0.0) {
        displacementKm = 0.0;
    } else if (displacementKm > 999.9) {
        displacementKm = 999.9;
    }

    // Floor-truncate to tenths (0.1 km) to prevent rounding up unearned distance
    double displayKm = std::floor(displacementKm * 10.0 + 1e-9) / 10.0;
    const char *statusWord = getRoverStateStatusWord(state);
    int written = snprintf(outBuf, bufSize, "%-9s%5.1f k", statusWord, displayKm);
    return (written == 16);
}
