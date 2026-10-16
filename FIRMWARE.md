# Getting the Firmware

To program the GNSS Compass display, you need the compiled firmware binary (`gps_compass.ino.esp32s3.bin`). You can obtain this file in one of two ways:

* **[Option A: Download Pre-Compiled Binary (Recommended)](#option-a-download-pre-compiled-binary-recommended)**: Fast and straightforward. Recommended for most users who simply want to flash and run the compass.
* **[Option B: Compile from Source Code](#option-b-compile-from-source-code)**: For developers who want to inspect the source, modify features, or build locally using `arduino-cli`.

Once you have the binary file from either option, proceed to the **[Firmware Installation Guide](INSTALL.md)** to flash it onto your board.

---

## Option A: Download Pre-Compiled Binary (Recommended)

1. Go to the repository's [Releases](../../releases) page.
2. Under the latest release, download the following two files:
   * **`gps_compass.ino.esp32s3.bin`** (the compiled application firmware)
   * **`CHECKSUMS`** (SHA-256 verification hashes)

3. *(Optional but Recommended)* Verify the downloaded file integrity in your terminal:
   ```bash
   sha256sum -c CHECKSUMS
   ```
   Confirm the output reports:
   ```text
   gps_compass.ino.esp32s3.bin: OK
   ```

4. **Next Step:** You have your firmware binary. Proceed to the **[Firmware Installation Guide](INSTALL.md)** to flash it.

---

## Option B: Compile from Source Code

If you prefer to build the firmware from source, follow the instructions below to configure the `arduino-cli` environment and compile the sketch.

### 1. Host Toolchain Setup

#### Fedora / Red Hat Enterprise Linux (Tested & Verified)
```bash
sudo dnf install curl python3 git gcc-c++
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH="$HOME/.local/bin:$PATH"
```

#### Ubuntu / Debian Linux (Verified via CI)
```bash
sudo apt update && sudo apt install -y curl python3 git g++
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH="$HOME/.local/bin:$PATH"
```

#### macOS (Reference)
```bash
brew install arduino-cli
```

#### Windows (Placeholder)
> [!NOTE]
> Windows builds can be performed with `arduino-cli` installed via `winget install Arduino.ArduinoCLI` or downloaded from the [Arduino CLI GitHub](https://github.com/arduino/arduino-cli/releases). Command shell syntax and paths will vary by shell.

---

### 2. Configure Arduino CLI & Install ESP32 Core

Initialize Arduino CLI and configure the Espressif board manager URL:
```bash
arduino-cli config init
arduino-cli config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
```

Install version **`2.0.14`** of the ESP32 core:
```bash
arduino-cli core install esp32:esp32@2.0.14
```

> [!IMPORTANT]
> Version **`2.0.14`** is strictly required. ESP32 core 3.x introduces breaking changes incompatible with Elecrow's display drivers.

---

### 3. Install Required Libraries

Install the required support libraries, locking the graphics library to version `1.3.1`:
```bash
arduino-cli lib install "GFX Library for Arduino"@1.3.1
arduino-cli lib install "PCF8574 library"
arduino-cli lib install "Adafruit CST8XX Library"
arduino-cli lib install "Adafruit BusIO"
```

> [!WARNING]
> Do not omit `@1.3.1` for `GFX Library for Arduino`. Newer versions contain breaking changes for this board's ST7701 round panel driver.

---

### 4. Run Pre-Build Unit Tests

Run the standalone host unit test suite to verify mathematical projections and formatting:
```bash
./tests/run_tests.sh
```

---

### 5. Compile the Firmware

Arduino CLI requires sketches to reside inside a folder matching the sketch name:

```bash
mkdir -p build/gps_compass
cp gps_compass.ino build/gps_compass/
```

*(Optional) Set a custom version identifier displayed on the screen footer:*
```bash
sed -i 's/#define FW_VERSION .*/#define FW_VERSION           "v0.2-custom"/' build/gps_compass/gps_compass.ino
```

Compile and export the binary artifacts:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi --export-binaries build/gps_compass
```

Your compiled application binary will be located at:
```text
build/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bin
```

Copy or rename it to `gps_compass.ino.esp32s3.bin`:
```bash
cp build/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bin ./gps_compass.ino.esp32s3.bin
```

Clean up temporary build files when done:
```bash
rm -rf build
```

---

## Next Step: Installing the Firmware

Whether you downloaded the pre-compiled binary via **Option A** or compiled your own via **Option B**, proceed to the **[Firmware Installation Guide](INSTALL.md)** for instructions on flashing your board.
