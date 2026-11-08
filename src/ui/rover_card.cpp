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

    const char *statusWord = getRoverStateStatusWord(state);
    int written = snprintf(outBuf, bufSize, "%-9s%5.1f k", statusWord, displacementKm);
    return (written == 16);
}
