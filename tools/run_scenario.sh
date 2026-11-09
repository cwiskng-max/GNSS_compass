#!/usr/bin/env bash
# ==============================================================================
# GNSS Compass - Simulation Scenario Dispatcher
# ==============================================================================
#
# Interactive menu or CLI dispatcher for all GNSS Compass test scenarios:
#   1) first-transit : Accelerated Rover first transit & backtrack (< 1 min)
#   2) duplicate-grid: Duplicate grid square detection & revisit (< 1 min)
#   3) rover         : Real-time Rover transit (ARRL 16 km rule)
#   4) backtrack     : Mountain switchback road scenario
#   5) sweep         : 360-degree antenna rotator continuous sweep
#   6) rtk           : RTK Fixed -> Float -> Lost -> Timeout cycle
#   7) drive         : Interactive WASD keyboard driving simulator
#
# Usage:
#   ./tools/run_scenario.sh                   # Interactive scenario menu
#   ./tools/run_scenario.sh first-transit     # Run specific scenario
#   ./tools/run_scenario.sh duplicate-grid    # Run duplicate grid scenario
#   ./tools/run_scenario.sh sweep --speed 12  # Pass custom scenario arguments
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SIM="${SCRIPT_DIR}/gps_sim.py"

if [ ! -f "${PYTHON_SIM}" ]; then
    echo "[ERROR] Simulator script not found: ${PYTHON_SIM}" >&2
    exit 1
fi

# If a scenario name was passed as first argument, forward it
if [ $# -gt 0 ]; then
    exec python3 "${PYTHON_SIM}" "$@"
fi

# Otherwise, present an interactive menu
echo "===================================================================="
echo "          GNSS COMPASS - SIMULATION SCENARIO RUNNER                 "
echo "===================================================================="
echo "  1) Rover First Transit & Backtrack (< 1 min, clean slate, 16 km)"
echo "  2) Rover Duplicate Grid / Revisit (< 1 min: Red -> Green -> Blue -> Green)"
echo "  3) Mountain Switchback / Backtrack (climb -> canyon -> breakout)"
echo "  4) 360° Antenna Rotator Sweep (continuous rotation)"
echo "  5) RTK Quality Degradation & Watchdog (Fixed -> Float -> Lost)"
echo "  6) Real-Time Rover Transit (25 km journey from origin)"
echo "  7) Interactive Drive Controller (WASD keyboard driving)"
echo "  8) Custom Coordinate / Heading Injection (inject)"
echo "  Q) Quit"
echo "===================================================================="
read -rp "Select a scenario [1-8, Q]: " choice

case "${choice}" in
    1)
        exec python3 "${PYTHON_SIM}" first-transit
        ;;
    2)
        exec python3 "${PYTHON_SIM}" duplicate-grid
        ;;
    3)
        exec python3 "${PYTHON_SIM}" backtrack
        ;;
    4)
        exec python3 "${PYTHON_SIM}" sweep
        ;;
    5)
        exec python3 "${PYTHON_SIM}" rtk
        ;;
    6)
        exec python3 "${PYTHON_SIM}" rover
        ;;
    7)
        exec python3 "${PYTHON_SIM}" drive
        ;;
    8)
        read -rp "Enter Maidenhead grid (e.g. CM87wa): " grid
        read -rp "Enter heading in degrees (0-360) [0]: " hdg
        hdg="${hdg:-0}"
        exec python3 "${PYTHON_SIM}" inject --grid "${grid}" --hdg "${hdg}" --duration 10
        ;;
    [qQ]*)
        echo "Exiting."
        exit 0
        ;;
    *)
        echo "Invalid selection: ${choice}"
        exit 1
        ;;
esac

