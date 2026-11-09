#!/usr/bin/env bash
# ==============================================================================
# GNSS Compass - Rover Mode First Transit & Backtrack Scenario Runner
# ==============================================================================
#
# Complete fast scenario validating Rover Mode transit rules in < 1 minute:
#   1. Clean Slate: Erases contest history in NVS (0 stops logged).
#   2. Stop #1 Origin: Injects initial coordinates (default CM87wa) for 2s,
#      then stops transmission to verify 'NO GPS' (State D Amber) tolerance.
#   3. Arm Rover Mode: Prompts you to start Rover Mode on the device touchscreen
#      (or type 's' to start over serial) to snapshot the origin location.
#   4. Accelerated Transit & Backtrack Simulation (~45 seconds total):
#      - Phase 1: Rapid approach & cross 16 km (15.2 -> 16.8 km, Red -> Green)
#      - Phase 2: Mountain canyon backtrack (16.8 -> 14.8 km, Green -> Red)
#      - Phase 3: Ridge breakout past 16 km (14.8 -> 17.0 km, Red -> Green)
#   5. Holds final qualified position for 8 seconds and displays status summary.
#
# Usage:
#   ./tools/run_rover_first_transit.sh               # Run on connected ESP32
#   ./tools/run_rover_first_transit.sh --dry-run     # Run in offline stdout mode
#   ./tools/run_rover_first_transit.sh --origin FN20  # Use custom origin grid
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PYTHON_SIM="${SCRIPT_DIR}/gps_sim.py"

if [ ! -f "${PYTHON_SIM}" ]; then
    echo "[ERROR] Simulator script not found: ${PYTHON_SIM}" >&2
    exit 1
fi

# Forward all arguments to the first-transit subcommand
exec python3 "${PYTHON_SIM}" first-transit "$@"

