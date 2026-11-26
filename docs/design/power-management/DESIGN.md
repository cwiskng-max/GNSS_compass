# ESP32-S3 Power Management — Technical Design

* **Feature:** Field Power-Saving Optimizations (Backlight, BLE Radio, I2C Polling, CPU Scaling)
* **Status:** Proposed (Deferred / Parked)
* **Target Hardware:** ESP32-S3 (16 MB Flash, 8 MB OPI PSRAM), Elecrow 2.1" Round Touch Display (480×480 ST7701 RGB)
* **Architecture Base:** Modular C++ Architecture (`src/system/board.h`, `src/system/app_controller.h`, `src/ui/input_manager.h`, `src/protocol/ble_serial_manager.h`)
* **Associated Docs:** [Feature Ideas](../../../FEATURE_IDEAS.md), [BOM](../../../BOM.md), [Firmware Build Guide](../../../FIRMWARE.md), [Bluetooth Serial Design](../bluetooth-serial/DESIGN.md)

---

## 1. Executive Summary & System Power Budget Reality

Field operations (amateur VHF/UHF/microwave roving and portable summit deployments) often run from 12 V LiFePO4 or USB battery packs. Reducing power consumption on the Elecrow 2.1" display unit lowers battery draw and reduces thermal buildup inside the 3D-printed display housing.

However, any firmware power optimization must be evaluated against the **full system power budget** ([BOM.md](../../../BOM.md)):

| Subsystem | Typical Load | Controllable by ESP32 Firmware? | Notes |
| :--- | :--- | :--- | :--- |
| **Unicore UM982 + 2× Active Helix Antennas** | Dominant system draw (continuous RTK engine + LNA bias) | **No** (always on via 5 V rail) | Powered over the 4-pin GX12 cable; not gated by the display MCU. |
| **5 V Voltage Regulator** | Linear `LM7805` wastes $(V_{\text{in}} - 5\text{V}) \times I$ as heat | **No** (hardware choice) | Switching to a buck converter yields larger system savings than MCU sleep. |
| **480×480 IPS Backlight (`BKL_PIN`)** | Largest controllable draw on the display unit | **Yes** | Currently fixed at `250/255` duty cycle. |
| **ESP32-S3 + OPI PSRAM + ST7701 RGB DMA** | Moderate continuous draw | **Partial** | RGB parallel panel requires continuous PSRAM/GDMA pixel clocking while active. |
| **BLE Radio (Bluedroid)** | Periodic TX/RX bursts | **Yes** | Currently configured for maximum TX power and 20–30 ms advertising. |

**Takeaway:** Display-side firmware optimizations will meaningfully extend battery life and cool the display enclosure, but total system runtime remains bounded primarily by the UM982 receiver and 5 V regulator efficiency.

---

## 2. Current Firmware Baseline

The modular firmware refactoring established clean architectural seams for power management, though no active power-saving policies are currently enabled:

* **Main Loop ([gps_compass.ino](../../../gps_compass.ino)):** Non-blocking `AppController::update()` followed by `delay(1)` cooperative FreeRTOS yield.
* **Backlight ([board.cpp](../../../src/system/board.cpp)):** `Board::setBacklight(uint8_t brightness)` drives `BKL_PIN` (GPIO 6) via `analogWrite()`. Called once at boot (`250`) and set to `0` only during `Board::restart()` / `prepareRestart`.
* **BLE Radio ([ble_serial_manager_esp32.cpp](../../../src/protocol/ble_serial_manager_esp32.cpp)):** Explicitly configured under the assumption that power is unconstrained:
  * TX power set to `ESP_PWR_LVL_P9` (+9 dBm) for both `ESP_BLE_PWR_TYPE_ADV` and `ESP_BLE_PWR_TYPE_DEFAULT`.
  * Advertising intervals set to `0x20`–`0x30` (20–30 ms) and preferred connection interval `0x000C`–`0x0018` (15–30 ms).
* **Input Polling ([input_manager.cpp](../../../src/ui/input_manager.cpp)):** `InputManager::update()` polls `tsPanel_.touched()` (CST8XX) and `pcf8574_->digitalRead(P5)` (PCF8574) on every `loop()` iteration (~1,000 Hz I2C polling), while rotary encoder rotation (`ENCODER_CLK`) is already interrupt-driven (`encoderIrq`).
* **Display ([display_ui.cpp](../../../src/ui/display_ui.cpp)):** Uses `Arduino_ESP32RGBPanel` + `Arduino_ST7701_RGBPanel` (no internal panel GRAM; requires continuous GDMA pixel streaming from OPI PSRAM).

---

## 3. Prioritized Power-Saving Architecture

Changes are ordered from highest return-on-effort to lowest/most constrained.

### 3.1 High Impact: Backlight Dimming & Inactivity Timeout

The LED backlight is the largest controllable load on the display board.

* **Mechanism:** Track `lastUserActivityMs_` in `AppController` whenever:
  * `InputManager` emits any `UserEvent` (touch, bezel rotation, button press).
  * A high-priority alert occurs (e.g., QSO Pop-Up Card, Rover 16 km qualification transition).
* **Two-Stage Policy (Configurable):**
  1. **Active Brightness:** User-configurable (default `250`, or lower field/night preset).
  2. **Idle Dim:** After `dimTimeoutSec` of inactivity (e.g., 60 s), ramp/set `board_.setBacklight(dimLevel)` (e.g., `50` / ~20%).
  3. **Optional Screen Off:** After `offTimeoutSec` (if enabled, `0` = never off so compass remains readable at a glance), call `board_.setBacklight(0)`.
* **Wake:** Any touch, encoder step, button press, or alert immediately restores active brightness. (If waking from screen-off, the first touch should wake the display without triggering an unintended UI action.)

### 3.2 High Impact: BLE TX Power & Advertising Backoff

* **TX Power:** Lower default `esp_ble_tx_power_set()` from `ESP_PWR_LVL_P9` (+9 dBm) to `ESP_PWR_LVL_N0` (0 dBm) or `ESP_PWR_LVL_P3` (+3 dBm). Typical operator distance (mast/dash to phone/laptop) is 1–3 m. Expose a CLI override if extended range is needed.
* **Two-Phase Advertising Interval:**
  * **Fast Discovery (0–30 s after boot or disconnect):** Keep `20–30 ms` (`0x20`–`0x30`) for snappy Web Bluetooth / phone pairing.
  * **Idle Advertising (> 30 s disconnected):** Reconfigure to `150–300 ms` (`0x00F0`–`0x01E0`) to cut radio wakeups by ~5–10× while remaining discoverable.
* **Radio Disable Option:** Allow `ble off` / `ble on` (or NVS startup preference) when operating strictly standalone without a phone/PWA.

### 3.3 Medium Impact: I2C Polling Throttling

* **Current:** `InputManager::update()` runs I2C transactions to the CST8XX touch controller and PCF8574 expander every ~1 ms.
* **Proposed:** Gate I2C touch and encoder-switch polling behind a `20–25 ms` (40–50 Hz) non-blocking interval check:
  * Rotary encoder steps remain zero-latency via `encoderIrq`.
  * 40–50 Hz touch sampling is well below human tap/hold perception (`LONG_PRESS_MS` is 1000+ ms) and cuts I2C bus activity by ~20×.

### 3.4 Medium Impact (Hardware-Dependent): CPU Frequency Scaling

* **Constraint (`arduino-esp32` 2.0.14):** The precompiled ESP32 Arduino core 2.0.14 generally ships without `CONFIG_PM_ENABLE=y`, meaning `esp_pm_configure()` Dynamic Frequency Scaling (DFS) is unavailable without custom-building ESP-IDF libraries.
* **Alternative:** Test static `setCpuFrequencyMhz(160)` or `setCpuFrequencyMhz(80)` in `setup()`.
  * Geodesy (`calculateWgs84`), NMEA parsing at 115200 baud, and sparse UI updates require minimal CPU headroom.
  * **Required Validation:** Confirm `Arduino_ESP32RGBPanel` pixel clock (`PCLK`) and OPI PSRAM DMA bandwidth remain stable at lower APB/CPU clocks with zero visual tearing or flicker.

### 3.5 Low Impact / Not Recommended: Automatic Light Sleep

* **Why Light Sleep Conflicts with the Display:** Because the ST7701 RGB panel lacks an internal framebuffer, entering ESP32-S3 light sleep gates the LCD/GDMA and PSRAM clocks, causing immediate display blanking or corruption.
* **Only Viable for a "Pocket / Screen-Off" Mode:** Implementing light sleep would require:
  1. `board_.setBacklight(0)`
  2. Powering down the LCD panel via PCF8574 `P3` (`LOW`) and tearing down/pausing the RGB DMA driver
  3. Configuring `esp_sleep_enable_ext0_wakeup` / `gpio_wakeup_enable` on `ENCODER_CLK`, touch `INT`, and `RXD2`
  4. Full LCD/touch re-initialization on wake
* Given the UM982 remains powered on the 5 V rail, the marginal mA saved over simply turning off the backlight does not justify the driver complexity unless battery life becomes a primary product requirement.

---

## 4. Proposed Configuration & CLI Surface (If Implemented)

Settings would persist in NVS via `NvsConfig("power")`:

| NVS Key | Default | Description |
| :--- | :--- | :--- |
| `bkl_bright` | `250` | Active backlight level (`10–255`) |
| `bkl_dim` | `50` | Idle dimmed backlight level (`0–255`) |
| `dim_sec` | `0` (disabled) or `60` | Seconds of inactivity before dimming |
| `off_sec` | `0` (disabled) | Seconds of inactivity before screen off |
| `ble_pwr` | `0` (0 dBm) | BLE TX power level |
| `ble_enabled` | `1` | Enable/disable BLE stack at boot |

Candidate Serial CLI commands:
* `power status` — Show current backlight level, BLE TX power/state, CPU MHz, and idle timers.
* `power brightness <10-255>` / `power dim <sec> [level]` / `power off <sec>`
* `ble power <low|med|high>` / `ble on` / `ble off`

---

## 5. Verification Plan (If Picked Up Later)

1. **Bench Current Profiling:** Measure 5 V display-unit current draw (isolated from UM982) across baseline (`250` backlight, BLE P9, 240 MHz) vs. dimmed backlight, throttled I2C, reduced BLE power, and 160/80 MHz CPU.
2. **Display Stability:** Verify zero flicker, tearing, or touch lag during 160/80 MHz and 25 ms I2C polling tests.
3. **BLE Reconnect Latency:** Verify Web Bluetooth ([HTML-BLEserial](https://ednieuw.github.io/HTML-BLEserial/)) and smartphone terminal discovery/reconnect times under relaxed advertising intervals.
4. **Host Unit Tests:** Add `AppController` inactivity-timer and wake-event tests (verifying first touch from screen-off wakes without triggering underlying button actions).
