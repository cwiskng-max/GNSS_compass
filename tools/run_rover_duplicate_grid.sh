#!/usr/bin/env bash
# ==============================================================================
# GNSS Compass - Duplicate Grid Square (State C REVISIT) Scenario Runner
# ==============================================================================
#
# Complete scenario validating duplicate grid detection in < 1 minute:
#   1. Clean slate reset & automatic seeding of 3 stops in NVS:
#      - Stop #1: CM87wa (0.0 km, ORIGIN)
#      - Stop #2: CM97ae (23.7 km, QUALIFIED)
#      - Stop #3: CM97ee (29.6 km, QUALIFIED)
#   2. Arms Leg 4 Rover Mode from Stop #3 (CM97ee).
#   3. Phase 1: Transit (< 16 km, Red) -> Crosses 16 km into fresh grid CM97cd (Green).
#   4. Phase 2: High-speed jump to 43.5 km into fresh grid CM87xa (outside CM87wa).
#   5. Phase 3: Moves across boundary into duplicate grid CM87wa -> REVISIT (Royal Blue)!
#   6. Phase 4: Moves back out of CM87wa into fresh grid CM87xa -> QUALIFIED (Green)!
#   7. Stops Rover Mode, commits Stop #4, and displays final 4-stop history log.
#
# Usage:
#   ./tools/run_rover_duplicate_grid.sh               # Run on connected ESP32
#   ./tools/run_rover_duplicate_grid.sh --pause 3.0   # Hold each state for 3.0s (for video recording)
#   ./tools/run_rover_duplicate_grid.sh --dry-run     # Offline simulation
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SIM="${SCRIPT_DIR}/gps_sim.py"

if [ ! -f "${PYTHON_SIM}" ]; then
    echo "[ERROR] Simulator script not found: ${PYTHON_SIM}" >&2
    exit 1
fi

exec python3 "${PYTHON_SIM}" duplicate-grid "$@"

