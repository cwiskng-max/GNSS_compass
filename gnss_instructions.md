# GNSS Receiver Configuration & Antenna Setup

This document describes how to configure the Unicore UM982 dual-antenna RTK receiver and properly mount the antennas for the GNSS Compass.

---

## 1. Receiver Overview & Output Messages

The compass firmware parses only two NMEA messages from the UM982 receiver on its `COM1` port:
* **`$GNHDT`**: True heading between the primary and secondary antennas (updated at 5 Hz / 0.2 s).
* **`$GNGLL`**: Geographic latitude and longitude position (updated at 0.1 Hz / 10 s).

All other messages should be disabled or left at defaults so the ESP32 serial buffer is not flooded.

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
GPHDT com1 0.2
SAVECONFIG
```

### Explanation of Commands
* `MODE ROVER UAV`: Configures the receiver in moving baseline / heading mode.
* `CONFIG SIGNALGROUP 3 6`: Selects multi-frequency constellations (GPS, BDS, GLONASS, Galileo).
* `GPGLL com1 10`: Outputs `$GNGLL` positioning sentence on `COM1` every 10 seconds.
* `GPHDT com1 0.2`: Outputs `$GNHDT` true heading sentence on `COM1` every 0.2 seconds (5 Hz).
* `SAVECONFIG`: Saves the current configuration to receiver flash so it persists across power cycles.

---

## 3. Antenna Installation & Alignment

* **Separation Baseline:** Mount the two tri-band antennas at least **3 feet (~1 meter)** apart. Longer baselines improve heading accuracy.
* **Orientation:** The baseline between the antennas must be mounted **perpendicular** to the microwave antenna heading direction.
* **180° Heading Correction:** On first power-up, if the displayed antenna heading is 180° backwards, simply swap the two antenna RF cables at the receiver (or physically swap antenna positions).

---

## 4. Software Build Note

If building the display firmware from source rather than installing the pre-compiled binary, ensure you use the exact library versions specified by Elecrow (see the [Build Guide](BUILD.md) for complete build toolchain instructions).
