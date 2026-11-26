# Configurable Heading Averaging Depth — Technical Design

* **Feature:** 3-Level Heading Vector Averaging Depth (CLI & NVS)
* **Status:** Proposed
* **Target Hardware:** ESP32-S3, Unicore UM982 Dual-Antenna RTK Receiver (`$KSXT` @ 5 Hz)
* **Architecture Base:** `BearingFilter` (`src/geo/bearing_filter.h`), `GpsManager` (`src/gnss/gps_manager.h`), `SerialCli` (`src/protocol/serial_cli.h`), `NvsConfig` (`src/system/nvs_store.h`)
* **Associated Docs:** [Dual-Antenna GNSS Heading Pipeline](../gnss-heading-pipeline/DESIGN.md), [Serial CLI Guide](../../../SERIAL.md), [Feature Ideas](../../../FEATURE_IDEAS.md)

---

## 1. Motivation & Design Philosophy

`BearingFilter` currently smooths dual-antenna GNSS headings using a hardcoded circular vector averaging window of `WINDOW_SIZE = 5` samples. At the Unicore UM982's `$KSXT` output rate of **5 Hz** (200 ms per sample), this corresponds to a fixed **1.0-second** averaging window.

Different field operating conditions benefit from different damping characteristics:
* **Fast slewing / hand-turned portable masts:** Need minimal lag so the readout tracks antenna rotation immediately.
* **Standard portable operation:** Balanced smoothing (the existing 5-sample / 1.0 s window).
* **High-wind summit deployments / noisy RTK Float:** Benefit from heavier damping to stabilize `0.1°` display jitter.

Rather than exposing a raw integer sample count (which forces the operator to reason about `$KSXT` sample rates and buffer math) or adding touch UI clutter to the 480×480 screen, this design proposes **3 discrete averaging levels** configured exclusively via the **Serial / BLE CLI** and persisted in **NVS**.

---

## 2. Three Averaging Levels

At 5 Hz (`200 ms` per `$KSXT` sentence):

| Level | CLI Token | Window Size ($N$) | Effective Window (@ 5 Hz) | Intended Use Case |
| :--- | :--- | :---: | :---: | :--- |
| **Low** | `low` | **1** | `0.2 s` (raw) | Rapid antenna slewing, rotator testing, zero-lag tracking. |
| **Medium** *(Default)* | `med` / `medium` | **5** | `1.0 s` | General portable/rover operation (matches current behavior). |
| **High** | `high` | **15** | `3.0 s` | High-wind mast flex damping, noisy Float solutions, steady `cal snap`. |

> [!NOTE]
> Preserving `Medium = 5` as the default ensures existing users experience **zero behavioral change** on upgrade unless they explicitly change the setting via CLI.

---

## 3. Firmware Architecture & Data Flow

### 3.1 `BearingFilter` (`src/geo/bearing_filter.h`, `src/geo/bearing_filter.cpp`)

Replace the compile-time `WINDOW_SIZE = 5` array bound with a fixed maximum capacity and a runtime active window size:

```cpp
enum class HeadingAveragingLevel : uint8_t {
    Low    = 0,  // 1 sample
    Medium = 1,  // 5 samples (default)
    High   = 2   // 15 samples
};
```

* **Static Allocation:** Size `sinBuffer_` and `cosBuffer_` to `MAX_WINDOW_SIZE = 15` (`2 × 15 × 4 = 120 bytes`, no heap allocation).
* **Accumulation Precision:** Upgrade `sinSum_` and `cosSum_` from `float` to `double` so incremental subtract/add updates (`sinSum_ -= old; sinSum_ += new`) do not accumulate floating-point drift over long operating sessions.
* **Runtime Window Changes (`setWindowSize(uint8_t size)`):**
  * Clamp `size` to `[1, MAX_WINDOW_SIZE]`.
  * If `hasSamples()` is true when changing levels at runtime, **re-seed** the resized window with the current uncalibrated smoothed heading (`seed(getRawHeading() - 270.0)`) rather than calling `reset()`. This avoids flashing `---.-` or mixing stale samples from a previously larger window.
  * If `!hasSamples()`, simply `reset()` and update `windowSize_`.

### 3.2 NVS Persistence (`NvsConfig`)

* **Namespace:** `"compass"` (shared with `cal_offset`)
* **Key:** `"avg_level"` (`uint8_t`, `0 = Low`, `1 = Medium`, `2 = High`, default `1`)
* Loaded at boot during `AppController::begin()` (or `GpsManager` / `Calibration` init) and applied to `gpsManager_.getBearingFilter()`.

---

## 4. Proposed CLI Interface (USB & BLE UART)

No touch UI or modal changes are included in this design. Configuration is handled through `SerialCli` (available over both USB-C and BLE UART):

| Command | Behavior |
| :--- | :--- |
| `cal avg` | Print the active averaging level and window depth (e.g., `Averaging: MEDIUM (5 samples, ~1.0s @ 5Hz)`). |
| `cal avg low` | Set averaging to `Low` (1 sample), save to NVS, and confirm. |
| `cal avg med` *(or `medium`)* | Set averaging to `Medium` (5 samples), save to NVS, and confirm. |
| `cal avg high` | Set averaging to `High` (15 samples), save to NVS, and confirm. |

Additionally, `status` and `help` output should reflect the active averaging level and document `cal avg [low|med|high]`.

---

## 5. Operational & Performance Impact

* **CPU:** Negligible. `BearingFilter::push()` remains $O(1)$ per sample regardless of $N$, and `getCalibratedHeading()` still performs a single `atan2()`.
* **RAM:** +80 bytes in `BearingFilter` (`MAX_WINDOW_SIZE = 15` vs. `5`).
* **Watchdog Independence:** `HEADING_TIMEOUT_MS` (1500 ms) in `GpsManager::checkStaleness()` triggers on packet arrival, independent of averaging depth.
* **Calibration (`cal snap`) Note:** When operating in `High` mode (`3.0 s` window), the operator should hold the antenna stationary for ~3 seconds before running `cal snap` so the moving window fully settles.

---

## 6. Verification Plan

1. **Host Unit Tests (`tests/test_calculations.cpp`):**
   * Verify `Low (1)`, `Medium (5)`, and `High (15)` window sizes and `359° / 1°` circular wrap-around at each depth.
   * Verify runtime transitions (`High -> Low`, `Low -> High`) preserve heading continuity without stale-slot corruption.
   * Verify `SerialCli` parsing of `cal avg`, `cal avg low|med|medium|high`, invalid arguments, and NVS persistence round-trip.
2. **On-Device Verification:**
   * Verify `gps_compass-update.bin` compiles cleanly via `./build/simple_build.sh` and test `cal avg` over USB and BLE UART.
