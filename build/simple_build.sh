#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

BUILD_DIR="$REPO_ROOT/build"
SCRATCH_DIR="$BUILD_DIR/tmp/gps_compass"
OUTPUT_BIN="$REPO_ROOT/gps_compass.ino.esp32s3.bin"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"

echo "Building the binary..."
echo

# Prepare scratch folder for arduino-cli
mkdir -p "$SCRATCH_DIR"
cp -r "$REPO_ROOT/gps_compass.ino" "$REPO_ROOT/src" "$SCRATCH_DIR/"

# Compile using modern --output-dir (replaces deprecated --export-binaries)
arduino-cli compile \
    --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc \
    --output-dir "$BUILD_DIR" \
    "$SCRATCH_DIR"

if [ -f "$BUILD_DIR/gps_compass.ino.bin" ]; then
    cp "$BUILD_DIR/gps_compass.ino.bin" "$OUTPUT_BIN"
fi

# Clean ONLY intermediate compiler artifacts; preserve build/ and this script!
rm -rf "$BUILD_DIR/tmp" \
       "$BUILD_DIR/gps_compass.ino.bin" \
       "$BUILD_DIR/gps_compass.ino.partitions.bin" \
       "$BUILD_DIR/gps_compass.ino.bootloader.bin" \
       "$BUILD_DIR/gps_compass.ino.elf" \
       "$BUILD_DIR/gps_compass.ino.map"

if [ -f "$OUTPUT_BIN" ]; then
    echo
    echo "The binary is in the root folder named: $(basename "$OUTPUT_BIN")"
else
    echo "Something went wrong. There's no binary here."
    exit 1
fi

echo
read -p "Do you want to install the binary to port $SERIAL_PORT? [Y/n] " resp
resp="${resp,,}"
resp="${resp:-y}"

if [[ "$resp" == "y" ]]; then
    esptool --chip esp32s3 --port "$SERIAL_PORT" --baud 921600 write-flash 0x10000 "$OUTPUT_BIN"
else
    echo "Skipping install."
fi

