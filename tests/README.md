# GNSS Compass Test Suite

This directory contains the testing framework for the GNSS Compass project, featuring both automated host-side unit testing and an on-device visual display test.

---

## 1. Automated Host-Side Unit Tests (CI & Local)

The automated test suite verifies all mathematical calculations, coordinate projections, rounding rules, formatting bounds, and boundary thresholds on the host system without requiring hardware or flashing.

### Files
* **`tests/test_calculations.cpp`**: Standalone C++ test suite containing assertion checks.
* **`tests/run_tests.sh`**: Test runner script that extracts the pure mathematical functions directly from `gps_compass.ino`, compiles with `g++`, executes the suite, and cleans up artifacts.

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
2. **Grid-to-Coordinate Math (`calcLatLon`):** Verifies Maidenhead locator center coordinates (e.g. `JJ00AA` at origin $+0.0208^\circ\text{ lat}, +0.0417^\circ\text{ lon}$) and bounds checking.
3. **ARRL 1 km Minimum Rule:** Confirms identical grids (`CM87wa` vs `CM87wa`) return exactly `1.0 km` per ARRL 10 GHz contest rules, while adjacent subsquares return $> 1.0\text{ km}$ ($\sim 4.63\text{ km}$).
4. **Distance Symmetry & Ground Truths:** Verifies $D(A, B) = D(B, A)$ and checks reference great-circle distances.
5. **WGS84 Geodesic Math (`calculateWGS84`):** Verifies distance and cardinal bearings ($0^\circ$ North, $90^\circ$ East, $180^\circ$ South, $270^\circ$ West).
6. **Round-Trip Locator Consistency (`calcLocator`):** Ensures coordinates derived from `calcLatLon` reconstruct the identical Maidenhead grid string when passed through `calcLocator`.
7. **Buffer Bounds & Ghost Character Erasure:** Formats distances across all ranges ($1.0\text{ km}$ to $> 10,000\text{ km}$) using `"ARRL %6.1f k  "`. Verifies string lengths stay strictly within the 24-byte buffer and trailing spaces exist to erase ghost characters on screen redraws.
8. **Rover Transit Threshold:** Asserts that distances $< 16,000\text{ m}$ activate the rover relocation alert, while $\ge 16,000\text{ m}$ clear it.
9. **Field Extraction (`getNthField`):** Validates comma-delimited token parsing, empty fields, and bounds.
10. **`$KSXT` Sentence Parsing:** Validates dual-antenna heading extraction: RTK Fixed (3/4), RTK Float (2/5), and Single-Point / Invalid rejection (0/1).
11. **`$KSXT` Time & Location Extraction:** Validates UTC timestamp (14-digit full and 6-digit short) and geographic decimal coordinate parsing with solution status checking.
12. **Legacy Sentence Parsing:** Tests fallback parsing of `$GNHDT` and `$GNTHS` and detects lost signal from empty fields.
13. **Current Bearing Calculation:** Validates $270^\circ$ orthogonal baseline offset and circular wrap-around $[0, 360^\circ)$.
14. **GPS Stream Evaluation:** Validates deterministic link verdicts (Disconnected, Corrupt, Acquiring, Position-Only, Float, Fixed).

---

## 2. On-Demand Visual Display Self-Test (Hardware)

The firmware includes an interactive self-test demo to visually verify LCD rendering, font alignment, color highlighting, GPS stream diagnosis, and state recovery directly on the physical 480×480 screen.

### How to Trigger the Test
* **Touchscreen:** Tap directly on the **firmware version string** at the bottom center of the display (`v0.3-dev`).
* **USB Serial:** Send `t` or `test` to `/dev/ttyACM0` at `115200` baud.
* *(Note: To run only the GPS stream diagnosis without the visual demo, send `status` or `gps` via USB Serial).*

### Test Sequence
The test automatically cycles through 6 visual scenarios, displaying the current test name in the bottom footer in cyan/yellow:

| Step | Footer Title | Screen State & Verification |
| :--- | :--- | :--- |
| **1** | `TEST 1: SAME GRID` | Local grid displays `CM87wa00` and target displays `CM87wa`. Verifies ARRL distance reads **`ARRL    1.0 k  `** (bearing remains White). |
| **2** | `TEST 2: ROVER < 16K` | Target grid set to adjacent subsquare `CM87wb` ($\sim 4.6\text{ km}$) with rover transit active. Verifies Row 5 turns **RED text on WHITE background**. |
| **3** | `TEST 3: DIST < 1000K` | Target grid set to `DN13ab` ($\sim 600\text{ km}$). Verifies 3-digit distance formatting. |
| **4** | `TEST 4: DIST > 1000K` | Target grid set to `EL29ja` ($\sim 2,500\text{ km}$). Verifies 4-digit distance formatting without screen wrapping. |
| **5** | `TEST 5: GHOST RECOVERY` | Target grid jumps to $> 10,000\text{ km}$ (`ZA00AA`), then drops back to $1.0\text{ km}$ (`CM87wa`). Verifies position 14 is cleanly erased and no ghost `'k'` remains. |
| **6** | `TEST 6: BEARING / GPS` | Rapidly tests bearing quality colors (1s White $\to$ 1s Yellow $\to$ 1s Red), then evaluates live `Serial2` data stream from UM982 receiver. Displays centered popup card: **`GPS: RTK FIXED (OK)`** (Green), **`GPS: RTK FLOAT (WARN)`** (Yellow), **`GPS: ACQUIRING`** (Yellow), or **`GPS: NO DATA / CABLE`** (Red). Prints full diagnostic report to serial console. |

### Pass/Fail Confirmation & Clean Restoration
* **Programmatic Assertions:** The firmware computes programmatic verification checks for the ARRL 1 km rule and rover threshold.
* **On-Screen Banner:** A centered green **`SELF-TEST PASSED`** banner pops up across the screen for 2.5 seconds upon completion.
* **State Restoration:** Erases the banner and cleanly restores your live GPS coordinates, local grid, target grid, heading, and the centered grey version footer (`v0.3-dev`).
