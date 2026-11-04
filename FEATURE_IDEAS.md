# Feature Ideas & Backlog Pool

This document serves as an unordered pool of ideas and potential enhancements for the GNSS Compass. Items here are deliberately unranked and unordered—features can be picked at will for design and implementation, and removed from this list once addressed or no longer needed.

---

## Candidate Feature Ideas

- [ ] **Implement power-saving calls in ESP32 code**
  - Leverage ESP32-S3 power management APIs (e.g. automatic light sleep / modem sleep, dynamic frequency scaling between 240 MHz / 80 MHz, display backlight dimming or sleep timeout during inactivity) to reduce field battery draw.

- [ ] **Configurable heading averaging depth**
  - Allow runtime configuration (via UI menu / settings or serial CLI) of the number of heading values to average in the circular mean filter.

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

