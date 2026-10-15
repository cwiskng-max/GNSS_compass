# Firmware Build Guide

This guide describes how to set up the build environment and compile the GNSS Compass firmware from source code using the [`arduino-cli`](https://arduino.github.io/arduino-cli/) command-line tool.

---

## 1. Prerequisites & Host Configuration

The target board is the **Elecrow 2.1" ESP32 Rotary Display** based on the **ESP32-S3** microcontroller (16MB Flash, 8MB OPI PSRAM).

### Host Operating System Setup

#### Fedora / Red Hat Enterprise Linux (Tested & Verified)
1. Ensure base dependencies are present:
   ```bash
   sudo dnf install curl python3 git gcc-c++
   ```
2. Install `arduino-cli` into your user binary directory (`~/.local/bin`):
   ```bash
   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
   ```
3. Ensure `~/.local/bin` is in your `PATH` (add to `~/.bashrc` if not already present):
   ```bash
   export PATH="$HOME/.local/bin:$PATH"
   ```
4. If you plan to flash directly from `arduino-cli`, grant serial access permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   *(Log out and back in or run `newgrp dialout` for permissions to take effect).*

#### Ubuntu / Debian Linux (Verified via CI)
1. Install base tools:
   ```bash
   sudo apt update && sudo apt install -y curl python3 git g++
   ```
2. Install `arduino-cli`:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
   export PATH="$HOME/.local/bin:$PATH"
   ```
3. Serial permissions (if uploading):
   ```bash
   sudo usermod -aG dialout $USER
   newgrp dialout
   ```

#### macOS (Reference)
Using [Homebrew](https://brew.sh/):
```bash
brew install arduino-cli
```
*(Or use the official install script above).*

#### Windows (Placeholder)
> [!NOTE]
> Windows build workflows have not been directly tested on this system.
> Windows users can install `arduino-cli` via `winget install Arduino.ArduinoCLI` or download releases from the [Arduino CLI GitHub](https://github.com/arduino/arduino-cli/releases). Command shell syntax (PowerShell / Command Prompt) and paths will differ from the bash examples below.

---

## 2. Arduino CLI Configuration & Core Installation

Initialize the Arduino CLI configuration and add the official Espressif ESP32 board manager index URL:

```bash
arduino-cli config init
arduino-cli config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
```

### Install ESP32 Core (Exact Version Requirement)
> [!IMPORTANT]
> You **must** install version **`2.0.14`** of the ESP32 core. Newer major releases (e.g., ESP32 core 3.x) introduce breaking API changes that will fail to compile with Elecrow's display drivers.

```bash
arduino-cli core install esp32:esp32@2.0.14
```

---

## 3. Install Required Libraries

Install the required support libraries, paying close attention to the version lock on the graphics library:

```bash
arduino-cli lib install "GFX Library for Arduino"@1.3.1
arduino-cli lib install "PCF8574 library"
arduino-cli lib install "Adafruit CST8XX Library"
arduino-cli lib install "Adafruit BusIO"
```

> [!WARNING]
> Do not omit `@1.3.1` for `GFX Library for Arduino`. Newer versions have incompatible class definitions for this board's ST7701 round display panel.

---

## 4. Run Pre-Build Unit Tests

Before building firmware images, run the standalone host-side test suite to verify mathematical projections and formatting:

```bash
./tests/run_tests.sh
```

All 8 tests must pass before compiling.

---

## 5. Compiling the Firmware

Arduino CLI requires sketches to reside in a directory whose name matches the sketch filename (e.g., `gps_compass/gps_compass.ino`).

### Step 1: Stage Sketch Directory
```bash
mkdir -p build/gps_compass
cp gps_compass.ino build/gps_compass/
```

### (Optional) Inject Firmware Version
To set a custom version identifier displayed on the boot footer:
```bash
sed -i 's/#define FW_VERSION .*/#define FW_VERSION           "v0.2-custom"/' build/gps_compass/gps_compass.ino
```

### Step 2: Compile
Compile targeting the ESP32-S3 board with 16MB Flash and Octal (OPI) PSRAM:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi build/gps_compass
```

### Compiling & Exporting Binaries
If you want to produce reusable binary files (matching what CI produces for releases):
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi --export-binaries build/gps_compass
```
Exported artifacts will be placed under `build/gps_compass/build/esp32.esp32.esp32s3/`:
* `gps_compass.ino.bin`: Application firmware (flashed at offset `0x10000`).
* `gps_compass.ino.bootloader.bin`: Second-stage bootloader (offset `0x0`).
* `gps_compass.ino.partitions.bin`: Partition table (offset `0x8000`).

---

## 6. Uploading Directly to Hardware

Connect the Elecrow 2.1" display via USB-C to your host machine.

### Compile and Upload in One Step
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi -p /dev/ttyACM0 -u build/gps_compass
```

### Upload a Previously Compiled Build
```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi build/gps_compass
```

*(Adjust `/dev/ttyACM0` if your board enumerates as `/dev/ttyACM1` or `/dev/ttyUSB0`)*.

---

## 7. Cleaning Up Build Artifacts

To remove the temporary staged sketch folder:
```bash
rm -rf build
```
