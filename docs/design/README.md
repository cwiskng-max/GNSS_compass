# GNSS Compass Design Documentation

This directory contains technical design specifications, architectural decision records (ADRs), UI/UX mockups, and mathematical models for features and hardware revisions in the GNSS Compass project.

Each major subsystem or feature design is organized into its own subfolder to keep documentation, diagrams, calculations, and prototype photos self-contained.

---

## Design Index

| Feature / Subsystem | Directory | Status | Summary |
| :--- | :--- | :--- | :--- |
| **Antenna Heading Calibration** | [`antenna-calibration/`](antenna-calibration/DESIGN.md) | **Implemented** | Non-volatile antenna alignment offset via NVS (`Preferences`), touch UI pill, snap modal, rotary knob fine-tuning, and serial CLI. |

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

