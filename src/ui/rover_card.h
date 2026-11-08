#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include "../config.h"

/**
 * @brief Dynamic qualification states for the Rover Mode 16 km transit rule.
 */
enum class RoverState {
    InTransitUnder16k,      // State A: < 16.0 km (Red on White)
    QualifiedNewGrid,       // State B: >= 16.0 km, unvisited grid (Forest Green on White)
    QualifiedRevisitedGrid, // State C: >= 16.0 km, visited earlier (Royal Blue on White)
    GpsLostStale            // State D: GPS fix lost / stale > 3s (Deep Amber on White)
};

/**
 * @brief Evaluates the dynamic 4-state qualification matrix.
 *
 * @param displacementKm Geodesic straight-line displacement in km from origin.
 * @param isRevisit True if current grid was visited earlier in the contest log.
 * @param isGpsFresh True if GPS fix is active and fresh (<= 3 seconds old).
 * @return RoverState Current operational qualification state.
 */
RoverState evaluateRoverState(double displacementKm, bool isRevisit, bool isGpsFresh);

/**
 * @brief Returns the RGB565 text foreground color for a RoverState.
 */
uint16_t getRoverStateTextColor(RoverState state);

/**
 * @brief Returns the 9-character padded status word for a RoverState.
 */
const char* getRoverStateStatusWord(RoverState state);

/**
 * @brief Formats the exact 16-character string for the Row 5 Rover Qualification Card.
 *
 * Output format: "%-9s%5.1f k", e.g. "TRANSIT    6.4 k", "QUALIFIED 18.2 k".
 *
 * @param state Current RoverState.
 * @param displacementKm Displacement in km (formatted to 1 decimal place).
 * @param outBuf Target character buffer (minimum 17 bytes for 16 chars + null terminator).
 * @param bufSize Total capacity of outBuf.
 * @return true if successfully formatted within bounds, false on invalid buffer.
 */
bool formatRoverCard(RoverState state, double displacementKm, char *outBuf, size_t bufSize);
