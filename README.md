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

---

## Documentation

* **[Hardware Schematic](Schematic.pdf):** Wiring schematic for the display, power regulation, receiver, and connectors.
* **[Bill of Materials (BOM)](BOM.md):** Complete hardware list, part descriptions, and sourcing links.
* **[GNSS Receiver Configuration](RECEIVER.md):** Instructions for configuring the Unicore UM982 receiver messages (`$GNHDT`, `$GNGLL`) and dual-antenna baseline.
* **[Getting the Firmware](FIRMWARE.md):** How to obtain the firmware binary — either by downloading a pre-compiled release (recommended) or compiling from source code.
* **[Firmware Installation Guide](INSTALL.md):** How to flash the firmware binary onto the ESP32-S3 display using `esptool` (covers Fedora, Ubuntu, macOS, and Windows).
* **[Testing & Self-Test Guide](tests/README.md):** Information on automated host unit tests and the on-device visual self-test.
