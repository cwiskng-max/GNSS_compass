# Adversarial Code Review Findings & Remediation Plan: Rover Mode & Transit Tracking
**Branch:** `rover-mode-design`  
**Commit Inspected:** `7b673ea`  
**Target Hardware:** ESP32-S3 (Xtensa Dual-Core LX7, 16 MB NOR Flash, 8 MB OPI PSRAM, FreeRTOS, 480×480 ST7701S RGB + CST8XX I2C Touch)

---

## 1. Executive Summary

An adversarial systems-level review of the Rover Mode and Transit Tracking feature identified 16 vulnerabilities spanning safety-critical flash wear/power-cut resilience, false-positive GPS watchdog triggers, asynchronous GUI corruption, and geodetic boundary edge cases.

---

## 2. Findings & Concrete Fix Specifications

### 🚨 CRITICAL (Showstoppers)

#### 1. Catastrophic NVS Wipeout on Mid-Write Power-Cut
* **File & Line:** `src/system/rover_log_manager.cpp:231-246`
* **Root Cause:** `saveToNvs()` executes `logPrefs.clear()`, wiping the namespace before writing entries in a loop. A power glitch or vehicle ignition drop leaves `ROVER_LOG_KEY_COUNT = 0`, wiping the entire contest history. Rollovers after stop #100 repeat this on every stop.
* **Remediation:** Remove `clear()`. Overwrite keys `s%u` in place and update `ROVER_LOG_KEY_COUNT`.

#### 2. Stationary Vehicle Triggers False-Alarm "NO GPS" Watchdog (State D)
* **File & Line:** `src/gnss/gps_manager.cpp:242-247`, `src/system/app_controller.cpp:649-656`
* **Root Cause:** `GpsManager::processSentence` suppresses `setLocation()` when coordinates change by $\le 0.00001^\circ$ ($\approx 1.1\text{ m}$). Stationary or slow-moving vehicles fail to trigger `setLocation()`, causing `lastLocationUpdate_` in `AppController` to age out after 3 seconds, falsely displaying `NO GPS` in Deep Amber despite full RTK Fixed lock.
* **Remediation:** Track a `lastLocationFixMillis_` timestamp on every valid decoded location sentence and notify `AppController` of freshness regardless of positional displacement.

#### 3. Screen Corruption: Asynchronous GPS Callbacks Puncture Rover History Modal
* **File & Line:** `src/system/app_controller.cpp:188-196, 312-321, 386-391`
* **Root Cause:** `drawCurrentBearing()` and `onTimeUpdated` only check `currentUiMode_ != UiMode::Calibrating`, omitting `UiMode::RoverHistory`. 5 Hz heading and time updates render directly over the modal dialog at `BEARY=223` and `TIME_Y=300`.
* **Remediation:** Guard all main screen background renderers with `if (currentUiMode_ != UiMode::Normal) return;`.

#### 4. Uninitialized Struct Padding Leaking Stack/Heap RAM into NOR Flash
* **File & Line:** `src/system/rover_log_manager.h:14-39`, `src/system/rover_log_manager.cpp:71, 238`
* **Root Cause:** `RoverStop` contains 10–14 uninitialized alignment padding bytes. Calling `putBytes(..., sizeof(RoverStop))` writes raw memory to NOR flash, causing corruption if alignment attributes change across builds.
* **Remediation:** Zero the struct memory with `std::memset(this, 0, sizeof(*this))` in constructors and enforce deterministic `#pragma pack(push, 1)`.

---

### ⚠️ HIGH (Logic & Geodesy Bugs)

#### 5. False Origin Locked at (0, 0) When Armed Without GPS Fix
* **File & Line:** `src/system/app_controller.cpp:81-108`
* **Root Cause:** `startRoverMode()` does not check `hasLocation_`. If triggered before GPS fix, origin is set to $(0, 0)$ and grid `"JJ00AA00"`. Upon acquiring satellites, distance calculates as ~12,000 km, permanently latching State B `QUALIFIED 999.9 k` in green.
* **Remediation:** Require `hasLocation_ == true` before entering Rover Mode; show `WAIT GPS FIX` warning if unacquired.

#### 6. Boundary Rounding Deception: Displays `16.0 k` for Illegal 15.96 km Moves
* **File & Line:** `src/ui/rover_card.cpp:46-59`, `src/system/app_controller.cpp:120-125`
* **Root Cause:** `snprintf("%5.1f")` rounds 15.96 km up to `16.0 k` in Red (`TRANSIT`). Drivers seeing `16.0 k` pull over and exit, where `finalDistKm >= 16.0` evaluates false, logging an illegal/unqualified stop.
* **Remediation:** Floor-truncate displacement to tenths (`std::floor(disp * 10.0) / 10.0`) so 15.96 km displays as `15.9 k`.

#### 7. Target Grid Overwrite & Accidental Exit via Rotary Knob Button
* **File & Line:** `src/system/app_controller.cpp:99, 476-481`
* **Root Cause:** `startRoverMode()` overwrites `gridEditor_` with `myGrid_`, erasing remote target grids. Furthermore, `EncoderButtonPressed` toggles edit mode in Rover Mode without checking mode, enabling single-click accidental exit.
* **Remediation:** Retain target grid across mode entry; ignore encoder button press in Rover Mode so exit requires deliberate character touch followed by rotation.

#### 8. Maidenhead Encoder Overflow at Poles & Antimeridian
* **File & Line:** `src/geo/maidenhead.cpp:50-84`
* **Root Cause:** At $\text{lon} \ge 180.0^\circ$ or $\text{lat} \ge 90.0^\circ$, `geoPointToGrid` outputs field character `'S'`, which is illegal (Maidenhead fields range from 'A' to 'R').
* **Remediation:** Clamp latitude to $[-89.99999, 89.99999]$ and wrap longitude into $[-180.0, 180.0)$.

#### 9. Sticky Confirmation State Vulnerability in Serial CLI `rover clear`
* **File & Line:** `src/protocol/serial_cli.cpp:168-180, 758-776`
* **Root Cause:** Unconfirmed `rover clear` leaves `pendingRoverClear_ = true`. Any subsequent `yes` input, even hours later, wipes all history.
* **Remediation:** Reset `pendingRoverClear_ = false` on any non-matching input.

---

### 🔍 MEDIUM (Robustness, Flash Wear & Display Artifacts)

#### 10. Circular LCD Chord Overflow & Row 5 Ghost Text Leaks
* **File & Line:** `src/ui/display_ui.cpp:205-224`, `src/config.h:145-148, 227-235`
* **Root Cause:** `REVBEAR_SZ = 4` text width ($384\text{ px}$) exceeds circular chord ($342\text{ px}$) at $Y=408$. Transitioning to Rover Card ($320\text{ px}$) leaves characters outside $X \in [80, 400]$ uncleared.
* **Remediation:** Clear full Row 5 chord ($X \in [0, 480]$) when switching modes; scale Bearing Mode Row 5 to Size 3.

#### 11. Unthrottled Display Redraws at 5–10 Hz Causing LCD Tearing
* **File & Line:** `src/ui/display_ui.cpp:205-219`, `src/system/app_controller.cpp:218-228`
* **Root Cause:** `renderRoverCard()` executes `fillRoundRect(..., WHITE)` on every GPS sentence, flickering the ST7701 RGB panel.
* **Remediation:** Add dirty checking to only redraw card background when state changes or distance delta $\ge 0.1\text{ km}$.

#### 12. Non-Monotonic Seconds-of-Day Timestamps Wrapping at Midnight
* **File & Line:** `src/system/app_controller.cpp:90, 123`
* **Root Cause:** Storing seconds-since-midnight causes timestamp roll-back from 86399 to 0 across contest midnight.
* **Remediation:** Store 32-bit UNIX/GPS epoch seconds.

#### 13. Inverted Destination Revisit Logic in Python Simulator
* **File & Line:** `tools/gps_sim.py:751`
* **Root Cause:** `revisit_mode` is set whenever `dest[:6] != origin[:6]`, falsely tagging all new grids as State C (`REVISIT`).
* **Remediation:** Only activate revisit mode when `--revisit` flag is passed or destination matches a previously activated site.

---

### 💡 LOW / NITPICK
* **14. Maidenhead Casing:** Change `full[4]` and `full[5]` in `geoPointToGrid` to lowercase `'a'` (`src/geo/maidenhead.cpp:74-75`).
* **15. Hitbox Overlap:** Separate `TOUCH_Y_MAX` ($200$) and `CAL_TOUCH_Y_MIN` ($205$) in `src/config.h:102, 117`.
* **16. Geodesic Nomenclature:** Document `calculateWgs84` as Haversine with WGS-84 equatorial radius (`src/geo/geo_math.cpp:14-41`).

