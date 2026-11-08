# Rover Mode & Transit Tracking — Phased Implementation Plan

* **Target Branch:** `rover-mode-design`
* **Specification Document:** [`docs/design/rover-transit/DESIGN.md`](./DESIGN.md)
* **Status:** Ready for Implementation
* **Hardware Target:** ESP32-S3 (16 MB Flash, 8 MB OPI PSRAM), 480×480 Round Touch Display, Rotary Encoder

---

## 1. Architecture Overview & File Manifest

The implementation builds directly upon the modular C++ architecture established in PR #10 and PR #12:

```text
src/
├── geo/
│   ├── geo_point.h            # Existing GeoPoint (lat, lon)
│   ├── geo_math.h / .cpp      # Existing calculateWgs84, isRoverTransitInsideAlert
│   └── maidenhead.h / .cpp    # Existing Maidenhead projection
├── system/
│   ├── rover_log_manager.h    # [NEW] Multi-stop history stack, NVS persistence, undo
│   ├── rover_log_manager.cpp  # [NEW] Implementation of NVS storage and queries
│   ├── app_controller.h       # Update operational modes and gesture routing
│   └── app_controller.cpp     # Coordinate origin snapshots, 2-step exit, auto-rearm
├── ui/
│   ├── rover_card.h           # [NEW] 4-state qualification evaluator & string formatting
│   ├── rover_card.cpp         # [NEW] Textual status word formatter (%5.1f k precision)
│   ├── display_ui.h / .cpp    # Row 5 white card rendering (Y=376) and partial refresh
│   └── rover_history_modal.h  # [NEW] One-shot undo modal ([DISCARD LAST STOP] / [CANCEL])
├── protocol/
│   ├── serial_cli.h / .cpp    # Extend CLI: rover history, rover undo, rover max, rover clear
└── config.h                   # Add ROVER_CARD_*, ROVER_CFG_* constants
```

---

## 2. Phased Implementation Breakdown

### Phase 1: Rover History Engine & NVS Storage (`RoverLogManager`)
* **Objective:** Implement persistent multi-stop contest history stack in NVS with bounds management, sequential undo, and duplicate grid search.
* **Key Tasks:**
  1. Define `struct RoverStop` in `src/system/rover_log_manager.h`:
     - Fields: `uint32_t timestamp`, `double latitude`, `double longitude`, `char grid[9]`, `float displacement_km`, `bool qualified_16km`.
  2. Implement `class RoverLogManager`:
     - NVS storage namespaces: `"rover_log"` for stop blobs/array, `"rover_cfg"` for capacity.
     - Persistent capacity: `maxStops` loaded from NVS (default: 100, clamped 10–500).
     - `appendStop(const RoverStop &stop)`: Appends new stop (commits to NVS).
     - `popLastStop()`: Pops topmost stop, falls back to prior entry, updates NVS.
     - `getStop(size_t index)` & `getStopCount()`: Stack inspection.
     - `getLastStop()` & `getPriorStop()`: Safe accessors for origin fallback.
     - `hasVisitedGrid(const char *grid)`: Checks 6-char subsquare against past stops for State C duplicate warnings.
     - `clearHistory()`: Resets contest history log only.
  3. **Unit Tests (`tests/test_calculations.cpp`):**
     - Validate append, capacity rollover, sequential pop, empty stack protection, and grid duplicate search.

---

### Phase 2: Qualification Evaluator & Row 5 White Card UI
* **Objective:** Implement the 4-state qualification matrix and Row 5 white card renderer ($Y=376$) with tenths-of-a-kilometer precision.
* **Key Tasks:**
  1. Create `src/ui/rover_card.h` & `rover_card.cpp`:
     - Define `enum class RoverState { InTransitUnder16k, QualifiedNewGrid, QualifiedRevisitedGrid, GpsLostStale }`.
     - `evaluateRoverState(double displacementKm, bool isRevisit, bool isGpsFresh)`: Pure function returning `RoverState`.
     - `formatRoverCard(RoverState state, double displacementKm, char *outBuf, size_t bufSize)`: Formats exact 16-character string:
       - State A: `TRANSIT  %5.1f k ` (Red `0xF800` on White `0xFFFF`)
       - State B: `QUALIFIED%5.1f k ` (Forest Green `0x03E0` on White `0xFFFF`)
       - State C: `REVISIT  %5.1f k ` (Royal Blue `0x011C` on White `0xFFFF`)
       - State D: `NO GPS   %5.1f k ` (Deep Amber `0xDC00` on White `0xFFFF` — holds last valid value)
  2. Integrate into `DisplayUi`:
     - `renderRoverCard(RoverState state, double displacementKm)`:
       - Background card: `gfx_->fillRoundRect(40, 372, 400, 44, 8, 0xFFFF)`.
       - Text rendering: Centered monospace at size 4 (`REVBEAR_SZ`).
       - Flicker-free updates: Redraws only on state change, $\ge 0.1\text{ km}$ distance delta, or 1 Hz watchdog event.
  3. **Unit Tests:**
     - Truth table assertions for States A–D and string buffer safety.

---

### Phase 3: Application Coordination, State Transitions & Gestures
* **Objective:** Wire mode transitions, 1-tap origin capture, 2-step natural target exit, and 1.5-second long-press undo modal.
* **Key Tasks:**
  1. Update `AppController`:
     - Add `OperationalMode operationalMode_` (`MODE_BEARING`, `MODE_ROVER`).
     - `startRoverMode()`: Triggered by Row 1 tap.
       - If history is empty, record current position as **Stop #1 (`0.0 km`, ORIGIN)**.
       - Captures `originPoint_` and `originGrid_`.
       - Disarms target edit mode and hides cursor.
       - Sets `operationalMode_ = MODE_ROVER`.
     - `onTargetEdited()`: On first rotary encoder tick that changes a character while editing:
       - Calculate final displacement from `originPoint_`.
       - Append completed stop to `RoverLogManager`.
       - Transition `operationalMode_ = MODE_BEARING`.
     - 5-second inactivity timeout (`EDIT_TIMEOUT_MS = 5000`): Auto-aborts editing touches without exiting Rover Mode.
  2. Implement `RoverHistoryModal`:
     - Triggered by 1.5-second long-press on Row 1 (via `InputManager`).
     - Shows last stop details, `[ DISCARD LAST STOP ]` (Red), and `[ CANCEL ]` (Gray).
     - On Discard: pops stop, restores origin baseline to prior site, immediately closes dialog, and auto-rearms Rover Mode.
  3. **Unit Tests:**
     - Validate state machine transitions, gesture events, and auto-rearm upon undo.

---

### Phase 4: Serial CLI & Simulation Verification
* **Objective:** Wire USB serial commands and automated simulation test suites.
* **Key Tasks:**
  1. Extend `SerialCli`:
     - `rover status`: Prints origin, live displacement, grid boundary, state (A–D), and GPS health.
     - `rover start` & `rover stop`: CLI emulation for automated testing.
     - `rover history` / `rover log`: Formatted ASCII table dump.
     - `rover undo`: Pops latest stop and auto-rearms.
     - `rover max [n]`: Gets or sets NVS max stops capacity.
     - `rover clear`: Resets rover history in NVS with confirmation.
  2. Validate with `tools/gps_sim.py`:
     - `./tools/gps_sim.py rover`: State A $\to$ State B transition across 16 km.
     - `./tools/gps_sim.py backtrack`: Canyon retreat with instantaneous drop from State B $\to$ State A.
     - `./tools/gps_sim.py rtk`: Signal drop asserting State D Deep Amber holding last known value.
     - `./tools/gps_sim.py rover --origin CM87wb --dest CM87wa`: Loop course asserting State C (`REVISIT`).

---

## 3. Recommended Execution Sequence

| Phase | Milestone | Estimated Effort | Verification Gate |
| :---: | :--- | :---: | :--- |
| **1** | History Engine (`RoverLogManager`) | ~1 focused session | 100% native host unit tests pass |
| **2** | Qualification Evaluator & Row 5 White Card UI | ~1 focused session | Unit tests pass + visual SVG match |
| **3** | AppController Coordination & Gestures | ~1 focused session | Integration tests pass + touch hit-tests |
| **4** | Serial CLI & `gps_sim.py` End-to-End | ~0.5 session | All 4 simulation scenarios pass over serial |

*Total Implementation Horizon:* ~2 chat sessions.
