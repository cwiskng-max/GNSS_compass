# GNSS Compass Test Suite

This directory contains the testing framework for the GNSS Compass project, featuring both automated host-side unit testing and an on-device visual display test.

---

## 1. Automated Host-Side Unit Tests (CI & Local)

The automated test suite verifies all mathematical calculations, coordinate projections, rounding rules, formatting bounds, and boundary thresholds on the host system without requiring hardware or flashing.

### Files
* **`tests/test_calculations.cpp`**: Standalone C++ test suite containing assertion checks.
* **`tests/run_tests.sh`**: Test runner script that compiles the standalone `src/geo/` and `src/protocol/` modules with `tests/test_calculations.cpp` using `g++`, executes the suite, and validates all mathematical invariants.

### How to Run Locally
From the repository root, run:
```bash
./tests/run_tests.sh
```
Execution takes under 0.1 seconds.

### Continuous Integration (CI)
The test suite runs automatically on every push and pull request in [`.github/workflows/build.yml`](../.github/workflows/build.yml) immediately after repository checkout. If any calculation fails, CI stops immediately before downloading Arduino tools or compiling firmware.

### Test Cases Covered
1. **Trigonometric Helpers:** Validates `toRad()` and `toDeg()` conversions against $\pi$.
2. **Grid-to-Coordinate Math (`gridToGeoPoint`):** Verifies Maidenhead locator center coordinates (e.g. `JJ00AA` at origin $+0.0208^\circ\text{ lat}, +0.0417^\circ\text{ lon}$) and bounds checking.
3. **ARRL 1 km Minimum Rule:** Confirms identical grids (`CM87wa` vs `CM87wa`) return exactly `1.0 km` per ARRL 10 GHz contest rules, while adjacent subsquares return $> 1.0\text{ km}$ ($\sim 4.63\text{ km}$).
4. **Distance Symmetry & Ground Truths:** Verifies $D(A, B) = D(B, A)$ and checks reference great-circle distances.
5. **WGS84 Geodesic Math (`calculateWgs84`):** Verifies distance and cardinal bearings ($0^\circ$ North, $90^\circ$ East, $180^\circ$ South, $270^\circ$ West).
6. **Round-Trip Grid Consistency (`geoPointToGrid`):** Ensures coordinates derived from `gridToGeoPoint` reconstruct the identical Maidenhead grid string when passed through `geoPointToGrid`.
7. **Buffer Bounds & Ghost Character Erasure:** Formats distances across all ranges ($1.0\text{ km}$ to $> 10,000\text{ km}$) using `"ARRL %6.1f k  "`. Verifies string lengths stay strictly within the 24-byte buffer and trailing spaces exist to erase ghost characters on screen redraws.
8. **Rover Transit Threshold:** Asserts that distances $< 16,000\text{ m}$ activate the rover relocation alert, while $\ge 16,000\text{ m}$ clear it.
9. **Field Extraction (`getNthField`):** Validates comma-delimited token parsing, empty fields, and bounds.
10. **`$KSXT` Sentence Parsing:** Validates dual-antenna heading extraction: RTK Fixed (3/4), RTK Float (2/5), and Single-Point / Invalid rejection (0/1).
11. **`$KSXT` Time & Location Extraction:** Validates UTC timestamp (14-digit full and 6-digit short) and geographic decimal coordinate parsing with solution status checking.
12. **Legacy Sentence Parsing:** Tests fallback parsing of `$GNHDT` and `$GNTHS` and detects lost signal from empty fields.
13. **Current Bearing Calculation:** Validates $270^\circ$ orthogonal baseline offset and circular wrap-around $[0, 360^\circ)$.
14. **Antenna Calibration:** Tests offset angle normalization within $[-180^\circ, +180^\circ]$, circular bearing offset addition, and modal calibration math.
15. **GPS Stream Evaluation:** Validates deterministic link verdicts (Disconnected, Corrupt, Acquiring, Position-Only, Float, Fixed).
16. **GeoPoint Domain & Rover Precision:** Validates `GeoPoint` lat/lon unquantized source of truth, `gridToGeoPoint` / `geoPointToGrid` conversions, reverse bearing calculations, and Rover 16 km threshold evaluation.
17. **Configuration Constants (`src/config.h`):** Verifies hardware pin assignments, screen dimensions, layout coordinates, timeout macros, and target grid editing character macros.
18. **Antenna Calibration Class (`src/system/calibration.*`):** Tests two-stage offset staging, nudging (+/-0.1 deg), snap calculation, reset, flash commit, wrap-around normalization, and mock NVS persistence across instances.
19. **GPS Simulator (`src/gnss/simulator.*`):** Tests simulation state activation, virtual sentence injection callbacks, NMEA XOR checksums, synthetic `$KSXT`, `$GNGLL`, and `$GNHDT` sentence generation with roundtrip parser verification, and 64-bit double geodesic rover displacement calculations.
20. **GNSS Manager (`src/gnss/gps_manager.*`):** Tests UART byte ingestion, line assembly buffer bounds, solution extraction, staleness watchdog timeout transition (`HEADING_TIMEOUT_MS`), and stream health evaluation.
21. **GNSS TX Discovery Probe (`src/gnss/gps_manager.*`):** Tests probe initiation (`version\r\n`), state machine transitions (`Unknown` -> `Probing` -> `Available` / `Unavailable`), acknowledgment response parsing (`<OK`, `#VERSIONA`, `$command`), timeout retries, manual re-probe, and response callbacks.
22. **GNSS Configuration Pipeline Engine (`src/gnss/gps_config_pipeline.*`):** Tests step-by-step receiver configuration (`MODE` -> `SIGNALGROUP` -> `UNLOG` -> `KSXT` -> baseline constraint -> `SAVECONFIG`), non-volatile NOR flash write protection (abort before `SAVECONFIG` on error/timeout), and stream health verification.
23. **Host USB Serial CLI (`src/protocol/serial_cli.*`):** Tests USB command execution (`help`, `status`, `cal`, `sim`), terminal echo/backspace handling, antenna calibration snap/zero calculations, simulation parameters, and raw sentence dispatching.
24. **Serial CLI GNSS Commands (`src/protocol/serial_cli.*`):** Tests `gps probe`, `gps config` (standard and baseline constraints), `gps send`, `gps check` / `audit`, bidirectional link reporting, TX capability diagnostics, and BLE transport execution.
25. **Input Touch & Hit-Testing (`src/ui/input_manager.*`):** Verifies bounding box geometry hit tests for local grid, target grid, target grid character columns (with clamping), calibration pill, footer, and calibration modal buttons (Snap, Reset, Save, Cancel).
26. **Input Gesture State Machine (`src/ui/input_manager.*`):** Validates rotary encoder events, normal mode short taps, calibration modal button taps, 1.5s long-press hold on Save with progress calculation, commit events, drift cancellation (>35px), and touch debounce intervals.
27. **UI Formatters & Modal Presentation (`src/ui/cal_modal.*`, `src/ui/display_ui.*`):** Validates string formatters across all operational ranges (target bearing, raw GNSS heading, current/proposed offset, UTC timestamp, reverse bearing/distance, ARRL contest distance, heading string, calibration pill).
28. **Self-Test Failure Formatter (`src/ui/display_ui.*`):** Validates formatting of single-test and multi-test on-screen failure codes (e.g. `FAILED: TEST #3`, `FAILED: #1, #2, #7`), buffer truncation bounds safety, and pass message mapping.
29. **Automated Mathematical Invariant Coverage:** Validates all geodetic projections, NMEA decoders, FreeRTOS queue bounds, and vector heading operations.
30. **Grid Editor State (`src/ui/grid_editor.*`):** Tests target grid character rolling (INCG/DECG), cursor selection bounds clamping, and 5-second inactivity edit mode timeout.
31. **App Controller Coordination (`src/system/app_controller.*`):** Validates whole-application state synchronization, GPS position ingest to Maidenhead grid projection, user interaction events (encoder rotation, calibration modal lifecycle, local station grid sync, and rover alert activation).
32. **BLE Serial Manager (`src/protocol/ble_serial_manager*`):** Connection callbacks (including one state-based callback for a drop and reconnect between updates), output dropped while connected but not yet subscribed, queued output sent only from `update()`, discarding of stale RX bytes on reconnect, and `disconnect()` waiting for queued output.
33. **BLE TX Queue:** MTU-based chunking (20 bytes at MTU 23, capped at 244), backoff and retry after a rejected send, pausing on congestion, a 100-line dump, the overflow policy (drop until drained, then one notice), and discarding queued output on disconnect.
34. **BLE Diagnostics & Text Helpers (`ble_diagnostics.*`, `line_buffer.h`, `text_output.*`):** Always-on versus `diag on` logging, diagnostic queue overflow reporting, line editing, and single-call CRLF delivery for long lines.
35. **CLI Dual Transport & BLE Integration:** Independent USB/BLE line buffers, reply routing, broadcast, output-handler mirroring, `disconnect`, and reset of reply routing and partial BLE input when the link changes.

---

## 2. On-Demand Visual Display Self-Test (Hardware)

The firmware includes an interactive self-test demo to visually verify LCD rendering, font alignment, color highlighting, GPS stream diagnosis, and state recovery directly on the physical 480×480 screen.

### How to Trigger the Test
* **Touchscreen:** Tap directly on the **firmware version string** at the bottom center of the display (`v0.3-dev`).
* **USB Serial:** Send `t` or `test` to `/dev/ttyACM0` at `115200` baud.
* *(Note: To run only the GPS stream diagnosis without the visual demo, send `status` or `gps` via USB Serial).*

### Test Sequence
The test automatically cycles through 7 visual scenarios, displaying the current test name in the bottom footer in cyan/yellow:

| Step | Footer Title | Screen State & Verification |
| :--- | :--- | :--- |
| **1** | `TEST 1: SAME GRID` | Local grid displays `CM87wa00` and target displays `CM87wa`. Verifies ARRL distance reads **`ARRL    1.0 k  `** (bearing remains White). |
| **2** | `TEST 2: ROVER < 16K` | Target grid set to adjacent subsquare `CM87wb` ($\sim 4.6\text{ km}$) with rover transit active. Verifies Row 5 turns **RED text on WHITE background**. |
| **3** | `TEST 3: DIST < 1000K` | Target grid set to `DN13ab` ($\sim 600\text{ km}$). Verifies 3-digit distance formatting. |
| **4** | `TEST 4: DIST > 1000K` | Target grid set to `EL29ja` ($\sim 2,500\text{ km}$). Verifies 4-digit distance formatting without screen wrapping. |
| **5** | `TEST 5: GHOST RECOVERY` | Target grid jumps to $> 10,000\text{ km}$ (`ZA00AA`), then drops back to $1.0\text{ km}$ (`CM87wa`). Verifies position 14 is cleanly erased and no ghost `'k'` remains. |
| **6** | `TEST 6: BEARING / GPS` | Rapidly tests bearing quality colors (1s White $\to$ 1s Yellow $\to$ 1s Red), then evaluates live `Serial2` data stream from UM982 receiver. Displays centered popup card: **`GPS: RTK FIXED (OK)`** (Green), **`GPS: RTK FLOAT (WARN)`** (Yellow), **`GPS: ACQUIRING`** (Yellow), or **`GPS: NO DATA / CABLE`** (Red). Prints full diagnostic report to serial console. |
| **7** | `TEST 7: CAL MODAL` | Demos antenna calibration status pill (uncalibrated gray vs calibrated +4.2 deg cyan) and displays interactive calibration modal dialog preview. |

### Pass/Fail Confirmation & Clean Restoration
* **Programmatic Assertions:** The firmware validates calculations across all stages (ARRL 1 km minimum rule, Rover <16 km alert threshold, mid/long/extreme distance boundaries, baseline offset geometry, and antenna calibration offset normalization).
* **On-Screen Banner:** A centered green **`SELF-TEST PASSED`** banner pops up for 2.5 seconds upon success. If any test fails, an on-screen red card lists the failing test numbers (e.g. **`FAILED: TEST #2`** or **`FAILED: #1, #2`**) for 4.0 seconds, while detailed failure diagnostics are emitted over USB serial.
* **State Restoration:** Erases the banner and cleanly restores your live GPS coordinates, local grid, target grid, heading, and the centered grey version footer (`v0.3-dev`).
