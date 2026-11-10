# GNSS_compass

This repository contains the schematic, source code, bill of materials and instructions for building a GNSS compass suitable for amateur radio microwave operation.
The compass provides antenna azimuth heading accurate to < 0.5 degrees with 0.1 degree resolution without needing any calibration.
In addition to antenna heading several other functions are included:
- current maidenhead grid square to 8 digits
- target grid square to 6 digits
- azimuth  heading to target grid
- UTC time HH:MM
- reverse bearing from target grid
- distance between target and current grids in km
- wireless Bluetooth Low Energy (BLE) serial CLI for smartphones and web browsers

---

## Documentation

* **[Hardware Schematic](Schematic.pdf):** Wiring schematic for the display, power regulation, receiver, and connectors.
* **[Bill of Materials (BOM)](BOM.md):** Complete hardware list, electronics, mast/boom mounting hardware, sourcing links, and the [GX12 display ↔ receiver connector pinout](BOM.md#3-gx12-interconnect-pinout-display--receiver).
* **[GNSS Receiver Configuration](RECEIVER.md):** Instructions for configuring the Unicore UM982 receiver messages (`$KSXT`) and dual-antenna baseline.
* **[Getting the Firmware](FIRMWARE.md):** How to obtain the firmware binary ([`gps_compass.ino.esp32s3.bin`](https://github.com/cwiskng-max/GNSS_compass/releases/latest/download/gps_compass.ino.esp32s3.bin)) — either by downloading a pre-compiled release (recommended) or compiling from source code.
* **[Firmware Installation Guide](INSTALL.md):** How to flash the firmware binary onto the ESP32-S3 display using `esptool` (covers Fedora, Ubuntu, macOS, and Windows).
* **[Testing & Self-Test Guide](tests/README.md):** Information on automated host unit tests and the on-device visual self-test.
* **[Serial Interface & CLI Guide](SERIAL.md):** USB-C and wireless Bluetooth (BLE UART) serial terminal commands, live GNSS receiver diagnostics, visual self-test triggers, and antenna offset calibration.
* **[Design Documentation](docs/design/README.md):** Technical specifications, UI/UX layouts, and architectural designs for compass subsystems and features.
