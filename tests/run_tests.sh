#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== Extracting pure functions from gps_compass.ino ==="
cat << 'HEADER' > "$SCRIPT_DIR/extracted_math.h"
#pragma once
#include <cmath>
#include <cctype>
#include <cstring>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
HEADER

sed -n '/void calcLocator/,/^}/p' "$REPO_DIR/gps_compass.ino" >> "$SCRIPT_DIR/extracted_math.h"
sed -n '/double toRad/,/void calculateWGS84/p' "$REPO_DIR/gps_compass.ino" | sed '$d' >> "$SCRIPT_DIR/extracted_math.h"
sed -n '/void calculateWGS84/,/^}/p' "$REPO_DIR/gps_compass.ino" >> "$SCRIPT_DIR/extracted_math.h"

echo "=== Compiling unit test suite ==="
g++ -O2 -Wall -Wextra -I"$SCRIPT_DIR" "$SCRIPT_DIR/test_calculations.cpp" -o "$SCRIPT_DIR/run_tests"

echo "=== Running unit tests ==="
"$SCRIPT_DIR/run_tests"

echo "=== Cleaning test artifacts ==="
rm -f "$SCRIPT_DIR/run_tests" "$SCRIPT_DIR/extracted_math.h" "$SCRIPT_DIR/extract.sh"
echo "Tests completed successfully."
