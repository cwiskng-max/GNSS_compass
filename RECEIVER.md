# GNSS Receiver Configuration & Antenna Setup

This document describes how to configure the Unicore UM982 dual-antenna RTK receiver and properly mount the antennas for the GNSS Compass.

---

## 1. Receiver Overview & Output Messages

The compass firmware parses messages from the UM982 receiver on its `COM1` port:
* **`$KSXT`** *(Recommended)*: All-in-one message streaming heading, RTK differential quality, precise decimal latitude/longitude, and UTC time at 5 Hz (every 0.2 s). With `$KSXT`, no other message logs are needed.
* **`$GNGLL` & `$GNHDT` / `$GNTHS`** *(Legacy)*: Fully supported for backward compatibility with existing receiver setups or generic NMEA receivers, but redundant when `$KSXT` is active.

All other messages should be disabled so the ESP32 serial buffer is not flooded.

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
4. Send the following commands one by one to configure the operational mode, signal constellations, message stream, and commit settings to non-volatile storage:

```text
MODE ROVER UAV
CONFIG SIGNALGROUP 3 6
KSXT com1 0.2
SAVECONFIG
```

### Explanation of Commands
* `MODE ROVER UAV`: Configures the receiver in moving baseline / dual-antenna heading mode.
* `CONFIG SIGNALGROUP 3 6`: Selects multi-frequency constellations (GPS, BDS, GLONASS, Galileo).
* `KSXT com1 0.2`: Outputs the unified `$KSXT` sentence on `COM1` every 0.2 seconds (5 Hz), providing dual-antenna heading, RTK quality, decimal coordinates, and UTC time in a single stream.
* `SAVECONFIG`: Saves the current configuration to receiver flash so it persists across power cycles.

*(Note: `GPGLL` is no longer needed because `$KSXT` provides position and time 5 times per second).*

### Modifying / Upgrading an Existing Receiver
If your UM982 was previously configured with legacy commands (`GPHDT com1 0.2` and/or `GPGLL com1 10`), connect via USB-C and run these commands to unlog (disable) the obsolete sentences, activate `$KSXT`, and save:

```text
UNLOG COM1 GPHDT
UNLOG COM1 GPGLL
KSXT com1 0.2
SAVECONFIG
```
* `UNLOG COM1 GPHDT`: Disables the legacy `$GNHDT` heading sentence to prevent duplicate heading data.
* `UNLOG COM1 GPGLL`: Disables the redundant 10-second `$GNGLL` position message to eliminate unnecessary serial traffic.
* `KSXT com1 0.2`: Enables the unified 5 Hz stream for instantaneous time sync, position lock, and color-coded bearing status.
* `SAVECONFIG`: Commits the updated configuration to non-volatile memory.

---

## 3. Antenna Installation & Alignment

* **Mounting Hardware:** Complete parts, sourcing links, and fitment tips for the 1-meter carbon fiber boom, split tube clamps, crossover mast clamp, bulkhead adapters, and RF cables are detailed in **[BOM.md Section 2 (Mast, Mounting & Hardware)](BOM.md#2-mast-mounting--hardware)**.
* **Separation Baseline:** Mount the two tri-band antennas at least **3 feet (~1 meter)** apart (the 1m carbon fiber tube in the BOM provides this baseline). Longer baselines improve heading accuracy.
* **Orientation:** The baseline between the antennas must be mounted **perpendicular** to the microwave antenna heading direction.
* **180° Heading Correction:** On first power-up, if the displayed antenna heading is 180° backwards, simply swap the two antenna RF cables at the receiver (or physically swap antenna positions).

---

## 4. Next Steps: Firmware Setup & Installation

* **[Getting the Firmware](FIRMWARE.md):** Covers both downloading the pre-compiled release binary (recommended) and building from source code.
* **[Firmware Installation Guide](INSTALL.md):** Step-by-step flashing instructions using `esptool` for everyone once the firmware binary is ready.
