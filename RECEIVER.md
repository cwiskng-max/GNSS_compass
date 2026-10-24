# GNSS Receiver Configuration & Antenna Setup

This document describes how to configure the Unicore UM982 dual-antenna RTK receiver and properly mount the antennas for the GNSS Compass.

---

## 1. Receiver Overview & Output Messages

The compass firmware parses messages from the UM982 receiver on its `COM1` port:
* **`$KSXT`** *(Recommended)*: High-precision heading, baseline, and solution quality indicator (updated at 5 Hz / 0.2 s).
* **`$GNGLL`**: Geographic latitude and longitude position (updated at 0.1 Hz / 10 s).
* **`$GNHDT` / `$GNTHS`** *(Legacy)*: Supported for backward compatibility with previous configurations.

All other messages should be disabled or left at defaults so the ESP32 serial buffer is not flooded.

### Heading Display Color States
The displayed antenna bearing (Row 3, left) dynamically color codes reception quality:
* ⚪ **Pure White (`0xFFFF`)**: **RTK Fixed (High Precision)** — Full differential carrier-phase ambiguity resolved (~0.1° accuracy). Also used during initial satellite acquisition (`---.-`) and for legacy `$GNHDT` valid tracking.
* 🟡 **Bright Yellow (`0xFFE0`)**: **RTK Float (Reduced Quality)** — Satellites acquired, but degraded sky view, obstructions, or floating baseline ambiguities. Heading may be noisy or slightly off.
* 🔴 **Bright Red (`0xF800`)**: **Lost / Stale** — Differential signal lost, empty heading sentence, or no update received for > 1.5 seconds. Protects the operator against frozen stale readings.

---

## 2. Receiver Configuration Procedure

The UM982 module must be configured manually via its USB-C port before connecting it to the compass display:

1. Connect the UM982 module to your computer using a USB Type-C data cable.
2. Open a serial terminal emulator (e.g., `picocom`, `minicom`, PuTTY, or Serial Studio) configured for:
   * **Baud rate:** `115200`
   * **Data bits:** `8`
   * **Parity:** `None`
   * **Stop bits:** `1` (`8N1`)
   * **Flow control:** None
3. Once connected, you will see raw NMEA/binary output streaming from the receiver.
4. Send the following commands one by one to configure the operational mode, signal constellation, message rates, and commit settings to non-volatile storage:

```text
MODE ROVER UAV
CONFIG SIGNALGROUP 3 6
GPGLL com1 10
KSXT com1 0.2
SAVECONFIG
```

### Explanation of Commands
* `MODE ROVER UAV`: Configures the receiver in moving baseline / heading mode.
* `CONFIG SIGNALGROUP 3 6`: Selects multi-frequency constellations (GPS, BDS, GLONASS, Galileo).
* `GPGLL com1 10`: Outputs `$GNGLL` positioning sentence on `COM1` every 10 seconds.
* `KSXT com1 0.2`: Outputs `$KSXT` heading, baseline, and RTK quality status on `COM1` every 0.2 seconds (5 Hz).
* `SAVECONFIG`: Saves the current configuration to receiver flash so it persists across power cycles.

*(Note for existing setups: If your receiver is already configured with `GPHDT com1 0.2`, the firmware will continue to function normally. Reconfiguring to `KSXT com1 0.2` enables the Bright Yellow RTK Float warning).*

### Upgrading an Existing Receiver
If your UM982 was previously configured for `GPHDT`, connect via USB-C and run these three commands to unlog (disable) the old heading sentence, activate `$KSXT`, and save:

```text
UNLOG COM1 GPHDT
KSXT com1 0.2
SAVECONFIG
```
*(The `UNLOG COM1 GPHDT` command disables `$GNHDT` output on `COM1` so you avoid duplicate heading streams).*

---

## 3. Antenna Installation & Alignment

* **Separation Baseline:** Mount the two tri-band antennas at least **3 feet (~1 meter)** apart. Longer baselines improve heading accuracy.
* **Orientation:** The baseline between the antennas must be mounted **perpendicular** to the microwave antenna heading direction.
* **180° Heading Correction:** On first power-up, if the displayed antenna heading is 180° backwards, simply swap the two antenna RF cables at the receiver (or physically swap antenna positions).

---

## 4. Next Steps: Firmware Setup & Installation

* **[Getting the Firmware](FIRMWARE.md):** Covers both downloading the pre-compiled release binary (recommended) and building from source code.
* **[Firmware Installation Guide](INSTALL.md):** Step-by-step flashing instructions using `esptool` for everyone once the firmware binary is ready.
