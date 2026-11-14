# Getting the Firmware

To program the GNSS Compass display, you need the compiled firmware binary. The project provides two primary packages for different deployment scenarios:

* **[File A: `gps_compass-factory.bin` (Fresh Install)](#option-a-download-pre-compiled-binary-recommended)**: Complete, all-in-one merged binary (bootloader, 16MB partition table, and application) flashed at address **`0x0`**. Recommended for initial setup on a blank or newly purchased board.
* **[File B: `gps_compass-update.bin` (Upgrade)](#option-a-download-pre-compiled-binary-recommended)**: Application-only binary flashed at address **`0x100000`**. Upgrades the firmware while **preserving your NVS storage** (all contest QSO logs, antenna calibration offset, and rover transit history remain intact).

> [!IMPORTANT]
> **Upgrading from an older single-`.bin` release?** You **must** download and flash **`gps_compass-factory.bin` at `0x0`** once to install the new 16MB partition table (`app0` moved from `0x10000` to `0x100000` with 512KB NVS). Note your antenna calibration offset (`status` in the serial CLI) beforehand, as this one-time repartition resets NVS. Future upgrades can then use `gps_compass-update.bin`.

You can obtain these files in one of two ways:
* **[Option A: Download Pre-Compiled Binary (Recommended)](#option-a-download-pre-compiled-binary-recommended)**: Fast and straightforward. Recommended for most users.
* **[Option B: Compile from Source Code](#option-b-compile-from-source-code)**: For developers who want to modify features or build locally using `arduino-cli`.

Once you have your binary, proceed to the **[Firmware Installation Guide](INSTALL.md)** to flash it onto your board.

---

## Option A: Download Pre-Compiled Binary (Recommended)

1. Go to the repository's [Releases](https://github.com/cwiskng-max/GNSS_compass/releases) page.
2. Under the latest release, download the file matching your needs:
   * **Fresh Install:** [**`gps_compass-factory.bin`**](https://github.com/cwiskng-max/GNSS_compass/releases/latest/download/gps_compass-factory.bin) (flashed to `0x0`)
   * **Upgrade:** [**`gps_compass-update.bin`**](https://github.com/cwiskng-max/GNSS_compass/releases/latest/download/gps_compass-update.bin) (flashed to `0x100000`, preserves NVS logs)
   * [**`CHECKSUMS`**](https://github.com/cwiskng-max/GNSS_compass/releases/latest/download/CHECKSUMS) (SHA-256 verification hashes)

3. *(Optional but Recommended)* Verify the downloaded file integrity in your terminal:
   ```bash
   sha256sum -c CHECKSUMS
   ```
   Confirm the output reports `OK` for your downloaded file:
   ```text
   gps_compass-factory.bin: OK
   gps_compass-update.bin: OK
   ```

4. **Next Step:** Proceed to the **[Firmware Installation Guide](INSTALL.md)** to flash it.

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

### 5. Automated Build & Packaging Script (Recommended)

The repository provides an automated script that compiles the sketch and packages both `gps_compass-factory.bin` and `gps_compass-update.bin`:

```bash
./build/simple_build.sh
```

The script:
1. Compiles the sketch targeting the ESP32-S3 with 16MB flash and OPI PSRAM.
2. Packages `gps_compass-update.bin` for application upgrades at `0x100000`.
3. Merges the bootloader, custom 16MB partition table, and application into `gps_compass-factory.bin` for fresh installation at `0x0`.
4. Optionally offers to flash the connected board directly.

---

### 6. Manual Compilation & Export (Alternative)

If you prefer to run the steps manually:

```bash
mkdir -p build/tmp/gps_compass
cp -r gps_compass.ino partitions.csv src build/tmp/gps_compass/
```

Compile and export the binary artifacts:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,CDCOnBoot=cdc --build-property upload.maximum_size=3670016 --export-binaries build/tmp/gps_compass
```

Your compiled application and partition binaries will be located at:
```text
build/tmp/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bin
build/tmp/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.partitions.bin
build/tmp/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bootloader.bin
```

Copy the application update binary to the repository root:
```bash
cp build/tmp/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bin ./gps_compass-update.bin
```

Merge into a single factory image at `0x0`:
```bash
esptool --chip esp32s3 merge_bin \
    --flash_mode dio --flash_freq 80m --flash_size 16MB \
    -o ./gps_compass-factory.bin \
    0x0 build/tmp/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bootloader.bin \
    0x8000 build/tmp/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.partitions.bin \
    0x89000 tools/boot_app0.bin \
    0x100000 build/tmp/gps_compass/build/esp32.esp32.esp32s3/gps_compass.ino.bin
```

Clean up temporary build files when done:
```bash
rm -rf build/tmp
```

---

## Next Step: Installing the Firmware

Whether you downloaded the pre-compiled binary via **Option A** or compiled your own via **Option B**, proceed to the **[Firmware Installation Guide](INSTALL.md)** for instructions on flashing your board.
