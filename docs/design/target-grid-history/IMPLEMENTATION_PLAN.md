# Target Grid Presets & History Manager — Multi-Chat Implementation Plan

**Branch:** `target-grid-history`  
**Design Spec:** [`docs/design/target-grid-history/DESIGN.md`](DESIGN.md)  
**Goal:** Implement the MVP in 4 self-contained chats so each session stays focused, compiles cleanly, and passes `bash tests/run_tests.sh` before moving to the next chat.

---

## Chat 1: Storage Layer & Core Preset Manager (`nvs_store.h` + `TargetPresetManager`)

**Files to touch:**
- `src/config.h` — Add `DEFAULT_MAX_TARGET_PRESETS` (`100`), `MIN_TARGET_PRESETS` (`10`), `ABS_MAX_TARGET_PRESETS` (`500`), `MAX_TARGET_LABEL_LEN` (`12`), `TARGET_CLEAR_HOLD_MS` (`1500`), `TARGET_MODAL_TIMEOUT_MS` (`30000`), NVS namespace/key defines, and Row 2 / modal geometry constants.
- `src/system/nvs_store.h` — New header-only `NvsConfig` & `NvsListStore<T>` helper supporting `#if defined(ARDUINO)` (`Preferences` with indexed keys `t0`, `t1`, ... and orphan cleanup) and `#else` host-test mock storage. *(Leave `RoverLogManager` and `QsoLogManager` untouched for MVP.)*
- `src/system/target_preset_manager.h` & `src/system/target_preset_manager.cpp` — New `TargetPreset` struct (`grid[7]`, `label[12]`, `useCount`, `lastUsedTime`), `REGIONAL_SUMMITS` fallback lookup, `addOrUpdatePreset()`, `recordUse()`, `removePreset()`, `removePresetByGrid()`, `clearAll()`, `setMaxTargets()`, `findByGrid()`, frequency-first sorting (`useCount` desc, `lastUsedTime` desc tie-breaker), and least-used/oldest eviction.
- `tests/run_tests.sh` & `tests/test_calculations.cpp` — Compile `target_preset_manager.cpp` and add unit tests for deduplication, labels with spaces, frequency-first sorting, `10–500` capacity clamping/eviction, deletion/shift, and NVS mock round-trip.

**Exit Criteria:** `bash tests/run_tests.sh` passes all existing + new `TargetPresetManager` tests; commit on `target-grid-history`.

---

## Chat 2: Serial & BLE CLI Provisioning (`SerialCli` + Wiring)

**Files to touch:**
- `src/protocol/serial_cli.h` & `src/protocol/serial_cli.cpp` — Extend `handleTargetCommand()` to dispatch:
  - `target add <GRID> [LABEL...]` (supports spaces in label)
  - `target save [LABEL...]` (saves current active target grid, supports spaces)
  - `target del <INDEX|GRID>`
  - `target list`
  - `target max [N]`
  - `target clear`
  - `target <GRID>` (existing direct-set behavior, including NATO phonetics, unchanged)
- `src/system/app_controller.h` & `gps_compass.ino` — Instantiate `TargetPresetManager` and wire it into `SerialCli` and `AppController`.
- `tests/test_calculations.cpp` — Add `SerialCli` tests for all new `target` subcommands (over both USB and BLE paths) and verify `target <grid>` continues to set the active target directly.

**Exit Criteria:** `bash tests/run_tests.sh` passes; USB/BLE CLI provisioning works end-to-end in host tests; commit on `target-grid-history`.

---

## Chat 3: Modal UI, Row 2 Label Subtitle & Input (`TargetListModal`, `DisplayUi`, `InputManager`)

**Files to touch:**
- `src/ui/target_list_modal.h` & `src/ui/target_list_modal.cpp` — Implement 480×480 circular modal rendering (`drawModal()`, `drawPresetRow()` with live `GeoPoint` bearing/distance via `calculateWgs84()`, scroll track, `[ LOAD / SELECT ]`, `[ DELETE ITEM ]`, `[ + SAVE CURRENT ]`, `[ HOLD TO CLEAR ALL ]` 1.5s progress bar, `[ CANCEL ]`) and host-testable string formatters.
- `src/ui/display_ui.h` & `src/ui/display_ui.cpp` — Extend `renderTargetGrid(grid, cursorCol, isEditing, label)` to display the optional `label` in the empty caret row (`REMSTARTY + CHAR_H(REMSIZE)`) when `isEditing == false` and not in Rover Mode, and clear it when `isEditing == true`. Keep Row 2 touch hitboxes untouched.
- `src/ui/input_manager.h` & `src/ui/input_manager.cpp` — Add Row 2 long-press (`> 500 ms` -> `LongPressTargetGridTriggered`), modal row tap (`TargetRowTouched`), and TargetList modal button/hold events (`HoldClearButtonProgress`, `ClearButtonCommitted`, `[ LOAD ]`, `[ DEL ]`, `[ + SAVE ]`, `[ CANCEL ]`) while preserving existing Row 2 character touch hitboxes and `EncoderButtonPressed` in `Normal` mode.
- `tests/run_tests.sh` & `tests/test_calculations.cpp` — Compile `target_list_modal.cpp` and add tests for `TargetListModal` formatters, `DisplayUi` Row 2 label behavior, and `InputManager` hit-testing / 1.5s hold-to-clear timing.

**Exit Criteria:** `bash tests/run_tests.sh` passes; UI & input layers are fully unit-tested; commit on `target-grid-history`.

---

## Chat 4: `AppController` Integration, End-to-End Verification & Docs

**Files to touch:**
- `src/system/app_controller.h` & `src/system/app_controller.cpp` — Add `UiMode::TargetList`, handle modal open/close, encoder scroll, `[ LOAD / SELECT ]` (increments `useCount`, loads `GridEditor`, calls `stopRoverMode(nowMs)` **only** on load, updates navigation), `[ CANCEL ]` / 30s timeout (preserves Rover Mode), `[ DELETE ITEM ]`, `[ + SAVE CURRENT ]`, `[ HOLD TO CLEAR ALL ]`, and Row 2 label lookup on `renderTargetGrid()`.
- `src/system/self_test.cpp` — Optional invariant check for `TargetPresetManager` in `SelfTest`.
- `tests/test_calculations.cpp` — Add end-to-end `AppController` workflow tests (pre-load -> Row 2 subtitle -> ad-hoc save -> Rover Mode -> `[ CANCEL ]` keeps Rover Mode -> `[ LOAD / SELECT ]` stops Rover Mode & updates `useCount`).
- `docs/design/README.md`, `docs/design/target-grid-history/DESIGN.md`, `SERIAL.md`, `OPERATION.md` — Update status to `Implemented` and document the new `target` subcommands and UI gesture.

**Exit Criteria:**
1. `bash tests/run_tests.sh` passes 100%.
2. ESP32-S3 firmware build passes (`arduino-cli` or CI).
3. Final commit & push on `target-grid-history`.
