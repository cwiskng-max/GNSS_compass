#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# Ensure ~/.local/bin is in PATH if arduino-cli is installed there
if [ -d "$HOME/.local/bin" ] && [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
    export PATH="$HOME/.local/bin:$PATH"
fi

BUILD_DIR="$REPO_ROOT/build"
SCRATCH_DIR="$BUILD_DIR/tmp/gps_compass"
FACTORY_BIN="$REPO_ROOT/gps_compass-factory.bin"
UPDATE_BIN="$REPO_ROOT/gps_compass-update.bin"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"

echo "Building the binary..."
echo

# Prepare scratch folder for arduino-cli
rm -rf "$SCRATCH_DIR"
rm -f "$FACTORY_BIN" "$UPDATE_BIN"
mkdir -p "$SCRATCH_DIR"
cp -r "$REPO_ROOT/gps_compass.ino" "$REPO_ROOT/partitions.csv" "$REPO_ROOT/src" "$SCRATCH_DIR/"

# Inject Git short hash into scratch config.h for local dev identification
GIT_HASH="$(git rev-parse --short HEAD 2>/dev/null || echo 'dev')"
DIRTY=""
if ! git diff-index --quiet HEAD -- 2>/dev/null; then
    DIRTY="-dirty"
fi
sed -i "s/#define FW_VERSION .*/#define FW_VERSION           \"v0.5-dev+${GIT_HASH}${DIRTY}\"/" "$SCRATCH_DIR/src/config.h"

# Compile using modern --output-dir (replaces deprecated --export-binaries)
# Match upload.maximum_size to app0/app1 (0x380000 = 3670016 bytes) in partitions.csv
arduino-cli compile \
    --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc \
    --build-property upload.maximum_size=3670016 \
    --output-dir "$BUILD_DIR" \
    "$SCRATCH_DIR"

# Locate boot_app0.bin for OTA data partition
BOOT_APP0=""
if [ -f "$REPO_ROOT/tools/boot_app0.bin" ]; then
    BOOT_APP0="$REPO_ROOT/tools/boot_app0.bin"
else
    for candidate in \
        "$HOME/.arduino15/packages/esp32/hardware/esp32"/*/tools/partitions/boot_app0.bin \
        "$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" \
        "/etc/arduino15/packages/esp32/hardware/esp32"/*/tools/partitions/boot_app0.bin; do
        if [ -f "$candidate" ]; then
            BOOT_APP0="$candidate"
            break
        fi
    done
fi

# Detect esptool command syntax compatibility (v3/v4 vs v5)
MERGE_CMD="merge_bin"
FLASH_MODE_FLAG="--flash_mode"
FLASH_FREQ_FLAG="--flash_freq"
FLASH_SIZE_FLAG="--flash_size"
WRITE_CMD="write_flash"

if esptool merge-bin --help >/dev/null 2>&1; then
    MERGE_CMD="merge-bin"
    FLASH_MODE_FLAG="--flash-mode"
    FLASH_FREQ_FLAG="--flash-freq"
    FLASH_SIZE_FLAG="--flash-size"
fi
if esptool write-flash --help >/dev/null 2>&1; then
    WRITE_CMD="write-flash"
fi

if [ -f "$BUILD_DIR/gps_compass.ino.bin" ]; then
    cp "$BUILD_DIR/gps_compass.ino.bin" "$UPDATE_BIN"
fi

# Generate merged factory binary (File A - Fresh Install at 0x0)
if [ -f "$BUILD_DIR/gps_compass.ino.bootloader.bin" ] && \
   [ -f "$BUILD_DIR/gps_compass.ino.partitions.bin" ] && \
   [ -f "$BUILD_DIR/gps_compass.ino.bin" ] && \
   [ -n "$BOOT_APP0" ] && [ -f "$BOOT_APP0" ]; then
    echo "Creating merged factory image (0x0)..."
    esptool --chip esp32s3 $MERGE_CMD \
        $FLASH_MODE_FLAG dio $FLASH_FREQ_FLAG 80m $FLASH_SIZE_FLAG 16MB \
        -o "$FACTORY_BIN" \
        0x0 "$BUILD_DIR/gps_compass.ino.bootloader.bin" \
        0x8000 "$BUILD_DIR/gps_compass.ino.partitions.bin" \
        0x89000 "$BOOT_APP0" \
        0x100000 "$BUILD_DIR/gps_compass.ino.bin"
fi

# Clean ONLY intermediate compiler artifacts; preserve build/ and this script!
rm -rf "$BUILD_DIR/tmp" \
       "$BUILD_DIR/gps_compass.ino.bin" \
       "$BUILD_DIR/gps_compass.ino.partitions.bin" \
       "$BUILD_DIR/gps_compass.ino.bootloader.bin" \
       "$BUILD_DIR/gps_compass.ino.elf" \
       "$BUILD_DIR/gps_compass.ino.map"

if [ -f "$FACTORY_BIN" ] || [ -f "$UPDATE_BIN" ]; then
    echo
    echo "=========================================================="
    echo " Build Complete! Firmware Packages Available:"
    echo "=========================================================="
    if [ -f "$UPDATE_BIN" ]; then
        echo " [File A - Upgrade Only]  : $(basename "$UPDATE_BIN")   (Flash at 0x100000, preserves NVS)"
    fi
    if [ -f "$FACTORY_BIN" ]; then
        echo " [File B - Fresh Install] : $(basename "$FACTORY_BIN")  (Flash at 0x0)"
    fi
    echo "=========================================================="
else
    echo "Something went wrong. No binary was produced."
    exit 1
fi

echo
echo "Flash Installation Options for port $SERIAL_PORT:"
echo "  1) Upgrade: Flash update binary to 0x100000 (Requires 16MB partition table; preserves NVS) [default]"
echo "  2) Fresh Install: Flash full factory image to 0x0 (Required once when upgrading from <=v0.4; repartitions flash)"
echo "  3) Skip installation"
echo
read -p "Select option [1/2/3, default: 1]: " resp
resp="${resp:-1}"

if [[ "$resp" == "1" ]]; then
    echo "Flashing upgrade binary to 0x100000..."
    esptool --chip esp32s3 --port "$SERIAL_PORT" --baud 921600 $WRITE_CMD 0x100000 "$UPDATE_BIN"
elif [[ "$resp" == "2" ]]; then
    if [ -f "$FACTORY_BIN" ]; then
        echo "Flashing full factory image to 0x0..."
        esptool --chip esp32s3 --port "$SERIAL_PORT" --baud 921600 $WRITE_CMD 0x0 "$FACTORY_BIN"
    else
        echo "Error: Factory binary not found."
        exit 1
    fi
else
    echo "Skipping install."
fi

