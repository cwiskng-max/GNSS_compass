# Feature Ideas & Backlog Pool

This document serves as an unordered pool of ideas and potential enhancements for the GNSS Compass. Items here are deliberately unranked and unordered—features can be picked at will for design and implementation, and removed from this list once addressed or no longer needed.

---

## Candidate Feature Ideas

- [ ] **Implement power-saving calls in ESP32 code** — *Parked; see [Power Management Design](docs/design/power-management/DESIGN.md)*
  - Leverage ESP32-S3 power management APIs (e.g. display backlight dimming/timeout, BLE TX power & advertising backoff, I2C polling throttling, CPU frequency scaling) to reduce field battery draw.

- [x] **Configurable heading averaging depth** — *Phase 1 implemented (`CircularEmaFilter` + `cal avg [low|med|high]` + NVS); Phase 2 Closed-Loop Kalman deferred to rotator work. See [Heading Averaging Design](docs/design/heading-averaging/DESIGN.md)*
  - Runtime configuration (via Serial/BLE CLI + NVS, with 3 discrete levels: `low`, `med`, `high`) of the heading circular EMA filter.

- [ ] **Closed-loop rotator PID tuning parameters**
  - Store and configure proportional, integral, and derivative ($K_p$, $K_i$, $K_d$) gain values in non-volatile storage (NVS) for antenna rotator position control.

- [ ] **Device operating mode: Open Loop / Closed Loop**
  - Provide a toggle or operational state switch between:
    - **Open Loop**: Monitor / display compass azimuth, geodetics, and target bearings passively for manual operator alignment.
    - **Closed Loop**: Actively command and drive rotator hardware to steer the antenna array toward the target.

- [ ] **Elevation sensor configuration**
  - Configurable elevation sensing options:
    - Elevation sensor presence: Enabled (`Yes`) / Disabled (`No`).
    - Elevation control mode: Open Loop (monitoring/readout only) vs. Closed Loop (active tilt drive).

- [ ] **Closed-loop tracking targets**
  - Selectable tracking targets for closed-loop pointing:
    - **Grid**: Point to designated Maidenhead grid square target (azimuth and optional elevation / tilt).
    - **Sun**: Real-time solar tracking calculation for microwave solar noise calibration / dish alignment.
    - **Moon**: Earth-Moon-Earth (EME) lunar ephemeris tracking (azimuth and elevation).

