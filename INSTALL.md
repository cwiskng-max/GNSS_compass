# Firmware Installation Guide

This guide covers flashing the firmware onto the Elecrow 2.1" ESP32 Rotary Display using `esptool`. The project provides two dedicated binary packages:

* **File A: `gps_compass-factory.bin` (Fresh Installation)**: Flashed to offset **`0x0`**. Contains the complete bootloader, custom 16MB partition table, and application. Use this for initial setup on a blank or new board.
* **File B: `gps_compass-update.bin` (Firmware Upgrade)**: Flashed to offset **`0x100000`**. Updates the application while preserving your non-volatile storage (NVS) data partition (retaining all contest QSO logs, antenna calibration, and rover history).

> [!TIP]
> If you do not yet have the firmware binaries, see **[Getting the Firmware](FIRMWARE.md)** first.

---

## 1. Prerequisites

* **Hardware:** [Elecrow 2.1" ESP32 Rotary Display](BOM.md) (ESP32-S3 with 16MB Flash / 8MB OPI PSRAM).
* **Cable:** USB Type-C data cable (ensure it supports data transfer, not power-only).
* **Firmware Binary:** Either `gps_compass-factory.bin` (for fresh installs) or `gps_compass-update.bin` (for upgrades), acquired via [FIRMWARE.md](FIRMWARE.md).

---

## 2. Installation on Fedora / Red Hat (Fully Tested & Verified)

Fedora is the primary development and tested environment for this project.

### Step 1: Install `esptool`
Fedora provides `esptool` directly from official repositories:
```bash
sudo dnf install esptool
```
*(Note: This package provides both `/usr/bin/esptool` and the `/usr/bin/esptool.py` wrapper).*

### Step 2: Grant Serial Port Permissions
To access serial devices without requiring `sudo`, add your user account to the `dialout` group:
```bash
sudo usermod -a -G dialout $USER
```
> [!IMPORTANT]
> You must log out and back into your desktop session (or run `newgrp dialout` in your current terminal) for this group change to take effect.

### Step 3: Connect Hardware and Identify Serial Port
Plug the Elecrow 2.1" display into your computer using the USB-C cable.

Identify the assigned device node:
```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```
The ESP32-S3 native USB CDC interface typically enumerates as:
* `/dev/ttyACM0` (standard for ESP32-S3 native USB)

*(If your board uses an external USB-to-UART bridge, it may appear as `/dev/ttyUSB0` instead).*

### Step 4: Flash Firmware

Choose the command matching your installation scenario:

#### Scenario A: Fresh Installation / Blank Board (Recommended for First-Time Setup)
Flashes the complete merged factory image at offset **`0x0`**. This writes the bootloader, custom 16MB partition table, and application in a single operation:

```bash
esptool --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash 0x0 gps_compass-factory.bin
```

#### Scenario B: Firmware Upgrade (Preserves Settings & Contest Logs)
Flashes only the application partition at offset **`0x100000`**. This updates the firmware code while **preserving your NVS storage partition** (`0x9000`–`0x89000`), ensuring your logged contest QSOs, antenna calibration offset, and rover history are not lost:

```bash
esptool --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash 0x100000 gps_compass-update.bin
```

* `--chip esp32s3`: Targets the ESP32-S3 microcontroller.
* `-p /dev/ttyACM0`: Specifies your serial port (adjust if yours is `/dev/ttyACM1` or `/dev/ttyUSB0`).
* `-b 921600`: High-speed baud rate for fast transfer (you can fall back to `460800` or `115200` if your cable experiences errors).

### Bootloader Mode Troubleshooting
The board usually resets into the ROM bootloader automatically. If `esptool` reports `A fatal error occurred: Failed to connect to ESP32-S3: No serial data received`:
1. Press and hold the **BOOT** button on the back of the board.
2. Press and release the **RESET** button (or connect the USB cable while holding BOOT).
3. Release the **BOOT** button.
4. Re-run the `esptool` command.

---

## 3. Installation on Other Operating Systems (Reference)

> [!NOTE]
> The author actively develops and tests on **Fedora**. The instructions below use standard Espressif tooling for other platforms; please report any issues or submit improvements via pull requests.

### Ubuntu / Debian Linux
1. Install `esptool`:
   ```bash
   sudo apt update && sudo apt install esptool
   ```
2. Add your user to the `dialout` group and refresh permissions:
   ```bash
   sudo usermod -aG dialout $USER
   newgrp dialout
   ```
3. Connect the board and identify the device (`/dev/ttyACM0` or `/dev/ttyUSB0`).
4. Flash using:
   * **Fresh install:**
     ```bash
     esptool --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash 0x0 gps_compass-factory.bin
     ```
   * **Upgrade (preserves settings/logs):**
     ```bash
     esptool --chip esp32s3 -p /dev/ttyACM0 -b 921600 write_flash 0x100000 gps_compass-update.bin
     ```

### macOS & Windows (via Python / pip)
On systems without a native package manager package for `esptool`, install it via Python:

1. Install `esptool`:
   ```bash
   pip install esptool
   ```
   *(Or using `pipx install esptool` if your Python environment restricts system-wide pip).*

2. **macOS Port:**
   * Plug in the device and locate the port:
     ```bash
     ls /dev/cu.usbmodem*
     ```
   * **Fresh install:**
     ```bash
     esptool.py --chip esp32s3 -p /dev/cu.usbmodemXXXX -b 921600 write_flash 0x0 gps_compass-factory.bin
     ```
   * **Upgrade:**
     ```bash
     esptool.py --chip esp32s3 -p /dev/cu.usbmodemXXXX -b 921600 write_flash 0x100000 gps_compass-update.bin
     ```

3. **Windows Port:**
   * Open **Device Manager** and expand **Ports (COM & LPT)** to find the assigned COM port (e.g., `COM3`, `COM4`).
   * Open Command Prompt or PowerShell and flash:
     * **Fresh install:**
       ```cmd
       esptool --chip esp32s3 -p COM3 -b 921600 write_flash 0x0 gps_compass-factory.bin
       ```
     * **Upgrade:**
       ```cmd
       esptool --chip esp32s3 -p COM3 -b 921600 write_flash 0x100000 gps_compass-update.bin
       ```

---

## 4. Post-Installation Verification

Once flashing completes, the display will restart automatically:

1. **Screen Initialization:** The 480×480 round LCD display should illuminate and show:
   * **Row 1:** Local Maidenhead grid (defaults to `JJ00AA00` until GPS position lock is acquired).
   * **Row 2:** Target Maidenhead grid (defaults to `ZA00AA` with the yellow editing caret `^`).
   * **Row 3:** Antenna heading (`---.-` until GPS true heading is received) and calculated target bearing.
   * **Row 4:** UTC time (`--:--Z` until GPS time is received).
   * **Row 5:** Calculated reverse bearing and distance (`<bear> deg  <dist> k`) computed between the local and target grids.
   * **Row 6:** Calculated ARRL contest distance (`ARRL <dist> k`).
   * **Footer:** Firmware version string (e.g., `v0.2`).
2. **On-Device Self-Test:** Tap directly on the firmware version string at the bottom center of the screen. The display will run through the 5-step visual self-test and display `SELF-TEST PASSED` in green. See [tests/README.md](tests/README.md) for details.
3. **Connect & Configure Receiver:** Connect and configure the Unicore UM982 GNSS receiver following [RECEIVER.md](RECEIVER.md). If using the standard 4-pin GX12 harness with the TX line wired (ESP32 GPIO 43 $\to$ UM982 COM1 RX), receiver setup is performed in-system using the `gps config` command.
