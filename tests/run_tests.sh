#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== Compiling unit test suite ==="
g++ -O2 -Wall -Wextra -I"$REPO_DIR" \
    "$REPO_DIR/src/geo/geo_math.cpp" \
    "$REPO_DIR/src/geo/maidenhead.cpp" \
    "$REPO_DIR/src/geo/bearing_filter.cpp" \
    "$REPO_DIR/src/protocol/nmea_parser.cpp" \
    "$REPO_DIR/src/protocol/serial_cli.cpp" \
    "$REPO_DIR/src/system/board.cpp" \
    "$REPO_DIR/src/system/calibration.cpp" \
    "$REPO_DIR/src/system/rover_log_manager.cpp" \
    "$REPO_DIR/src/system/self_test.cpp" \
    "$REPO_DIR/src/system/app_controller.cpp" \
    "$REPO_DIR/src/gnss/simulator.cpp" \
    "$REPO_DIR/src/gnss/gps_manager.cpp" \
    "$REPO_DIR/src/ui/grid_editor.cpp" \
    "$REPO_DIR/src/ui/input_manager.cpp" \
    "$REPO_DIR/src/ui/cal_modal.cpp" \
    "$REPO_DIR/src/ui/display_ui.cpp" \
    "$SCRIPT_DIR/test_calculations.cpp" -o "$SCRIPT_DIR/run_tests"

echo "=== Running unit tests ==="
"$SCRIPT_DIR/run_tests"

echo "=== Cleaning test artifacts ==="
rm -f "$SCRIPT_DIR/run_tests"
echo "Tests completed successfully."
