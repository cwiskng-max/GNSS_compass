# GNSS Compass Design Documentation

This directory contains technical design specifications, architectural decision records (ADRs), UI/UX mockups, and mathematical models for features and hardware revisions in the GNSS Compass project.

Each major subsystem or feature design is organized into its own subfolder to keep documentation, diagrams, calculations, and prototype photos self-contained.

---

## Design Index

| Feature / Subsystem | Directory | Status | Summary |
| :--- | :--- | :--- | :--- |
| **Antenna Heading Calibration** | [`antenna-calibration/`](antenna-calibration/DESIGN.md) | **Implemented** | Non-volatile antenna alignment offset via NVS (`Preferences`), touch UI pill, snap modal, rotary knob fine-tuning, and serial CLI. |
| **Visual Self-Test & GNSS Diagnostics** | [`visual-self-test/`](visual-self-test/DESIGN.md) | **Implemented** | On-device visual display self-test demo sequence, link health state machine (`GpsLinkVerdict`), and serial stream diagnostics. |
| **Rover Station Transit Check** | [`rover-transit/`](rover-transit/DESIGN.md) | **Implemented** | ARRL 10 GHz rover $<16\text{ km}$ proximity alert (highlighted Red-on-White), 1-tap origin grid copy, and auto-reset lifecycle. |
| **GPS Simulation & Test Framework** | [`gps-simulation/`](gps-simulation/DESIGN.md) | **Implemented** | Host-driven GPS data simulation, pre-canned rover transit journeys, interactive drive simulator, raw sentence injection, and serial CLI. |
| **Dual-Antenna GNSS Heading Pipeline** | [`gnss-heading-pipeline/`](gnss-heading-pipeline/DESIGN.md) | **Implemented** | Unicore UM982 `$KSXT` / NMEA parsing, circular mean vector smoothing ($\sin/\cos$), 1.5s staleness watchdog, and 4-tier color states. |
| **Maidenhead Geodetics & ARRL Rules** | [`maidenhead-geodetics/`](maidenhead-geodetics/DESIGN.md) | **Implemented** | Double-precision Maidenhead subsquare center calculation, great-circle azimuths, ARRL 1.0 km contest clamp, and host C++ unit testing. |
| **Circular Display Touch Interface** | [`circular-touch-ui/`](circular-touch-ui/DESIGN.md) | **Implemented** | 480x480 circular layout budget, 6-character touch Maidenhead editor with field rules, 5-second auto-lock, and rotary encoder input. |
| **Target Grid Presets & History** | [`target-grid-history/`](target-grid-history/DESIGN.md) | **Proposed** | Non-volatile target Maidenhead grid history, multi-summit preset manager for rover operations, live bearing/distance preview, rotary/touch selection, and protected hold-to-clear. |
| **Firmware Architecture & Modularization** | [`firmware-modularization/`](firmware-modularization/DESIGN.md) | **Implemented** | Modular C++ firmware architecture under `src/`, explicit `GeoPoint` lat/lon source of truth vs UI grid abstractions, test decoupling, and simplified loop. |
| **CI Build & Release Pipeline** | [`ci-firmware-pipeline/`](ci-firmware-pipeline/DESIGN.md) | **Implemented** | Headless `arduino-cli` ESP32-S3 CI builds, dynamic git version injection (`FW_VERSION`), release tag validation, and binary checksums. |

---

## Directory Organization Guidelines

When authoring new feature designs, create a dedicated subfolder:

```text
docs/design/
├── README.md                          # This index
└── <feature-name>/
    ├── DESIGN.md                      # Engineering spec, UI layout, math, and data flow
    ├── images/                        # (Optional) Screen captures, hardware photos, mockups
    └── ...
```

