# Antenna Heading Calibration & Offset Storage — Design Specification

* **Feature:** Antenna Alignment Offset Calibration & Non-Volatile Storage
* **Status:** Implemented (v0.3-dev)
* **Target Hardware:** ESP32-S3 (16 MB Flash, 8 MB OPI PSRAM), 480x480 Round Touch Display (ST7701 + CST8XX), Rotary Encoder

---

## 1. Problem Statement & Motivation

The GNSS Compass derives true geographic heading using a dual-antenna RTK receiver (Unicore UM982) measuring the carrier-phase baseline between two antennas.

In real-world amateur radio operations (VHF/UHF/microwave, rover contests, portable summits), the GNSS antenna baseline may not be mounted in exact mechanical parallel with the RF antenna boom or dish feed horn. Small mechanical tolerances, mast clamp twists, or vehicle rack offsets can introduce a constant azimuth error (e.g., $1^\circ$ to $5^\circ$).

### Operational Use Case
Rather than loosening mast clamps with wrenches in the field to attempt micro-adjustments:
1. The operator enters a known target Maidenhead grid (e.g., a distant beacon or mountain-top station).
2. The operator points the antenna toward the target, using the GNSS compass bearing to get close.
3. The operator peaks the receiver signal on an S-meter or waterfall.
4. At peak signal, the RF antenna is physically on target. The operator taps the calibration pill on screen to snap the compass heading to the target bearing.
5. The offset is saved to non-volatile flash and automatically applied to all subsequent heading displays.

---

## 2. Storage Architecture: ESP32 NVS (`Preferences.h`)

### Why ESP-IDF NVS?
* **Wear Leveling:** NOR flash has a limit of ~10,000 to 100,000 erase cycles. ESP-IDF Non-Volatile Storage (NVS) implements log-structured wear leveling across designated flash pages.
* **Power-Loss Resiliency:** Brownouts or unplugging power during a field contest will not corrupt the partition or brick settings.
* **Key-Value Simplicity:** Native typed getters and setters (`getFloat`, `putFloat`) without manual byte offsets or struct alignment bugs.

### Storage Parameters
* **Namespace:** `"compass"`
* **Key:** `"cal_offset"` (Type: `float`)
* **Default Value:** `0.0f` (uncalibrated)
* **Write Optimization:** Writes to NVS only if `fabs(new_offset - stored_offset) > 0.01f`.

```cpp
void loadCalibration() {
  Preferences prefs;
  prefs.begin("compass", true);
  calibrationOffset = prefs.getFloat("cal_offset", 0.0f);
  prefs.end();
}

void saveCalibration(float offset) {
  offset = normalizeOffset(offset);
  if (fabs(offset - calibrationOffset) > 0.01f) {
    calibrationOffset = offset;
    Preferences prefs;
    prefs.begin("compass", false);
    prefs.putFloat("cal_offset", calibrationOffset);
    prefs.end();
  }
}
```

---

## 3. Mathematical Model & Normalization

### Calibration Snap Calculation
When snapping to target bearing:
$$\Delta = \text{TargetBearing} - \text{RawHeading}$$

The offset is normalized to $[-180.0^\circ, +180.0^\circ]$:
```cpp
float normalizeOffset(float delta_deg) {
  while (delta_deg > 180.0f) delta_deg -= 360.0f;
  while (delta_deg < -180.0f) delta_deg += 360.0f;
  return delta_deg;
}
```
* **Example:** Target = $005.0^\circ$, Raw GNSS Heading = $355.0^\circ$.
  $$\Delta = 5.0 - 355.0 = -350.0^\circ \xrightarrow{\text{normalized}} +10.0^\circ$$

### Displayed Bearing Calculation
$$\text{CalibratedBearing} = (\text{RawBearing} + \text{Offset}) \pmod{360^\circ}$$
```cpp
double applyBearingOffset(double uncalibrated_bearing, float offset_deg) {
  double b = fmod(uncalibrated_bearing + (double)offset_deg, 360.0);
  if (b < 0.0) b += 360.0;
  return b;
}
```

---

## 4. UI Layout & Visual Specifications

Screen resolution: **480 x 480** round display.

### Row 3 Rebalance
Row 3 ($Y = 223$, Height = $48\text{ px}$) displays the bearings at text size `BEARSZ = 6` ($36\text{ px}$ per char, $180\text{ px}$ per 5-character string `%5.1f`):

```text
+-------------------------------------------------------------------+
|  X: 20 -> 200            X: 200 -> 280             X: 280 -> 460  |
|                                                                   |
|   +-----------+          +----------------+        +-----------+  |
|   |           |          |  .----------.  |        |           |  |
|   |   120.3   |          |  |   CAL    |  |        |   124.5   |  |
|   |           |          |  |  +4.2    |  |        |           |  |
|   |           |          |  '----------'  |        |           |  |
|   +-----------+          +----------------+        +-----------+  |
|                                                                   |
|  Antenna Heading          Center CAL Pill         Target Bearing  |
+-------------------------------------------------------------------+
```

* **Left Margin:** $20\text{ px}$ ($X = 20\dots 200$)
* **Right Margin:** $20\text{ px}$ ($X = 280\dots 460$)
* **Center Slot:** $80\text{ px}$ wide $\times 44\text{ px}$ high ($X = 200\dots 280$, $Y = 225$)

### Center Status Pill States
* **Uncalibrated ($0.0^\circ$):** Dark gray rounded frame (`0x39E7`), dim gray text (`0x7BEF`) reading `CAL / OFF`.
* **Calibrated ($\neq 0.0^\circ$):** Vibrant cyan frame (`0x07FF`) and cyan text reading `CAL / %+4.1f` (e.g. `CAL / +4.2`).
* **Touch Hitbox:** Extended padding covering $X \in [180, 300]$, $Y \in [210, 280]$ ($120 \times 70\text{ px}$).
* **Tap Action:** A direct single tap opens the Calibration Modal Dialog immediately.

---

## 5. Calibration Modal Dialog

Tapping the center `CAL` pill opens a high-contrast modal dialog card with layered safety protections:

```text
+---------------------------------------------------+
|               ANTENNA CALIBRATION                 |
| ------------------------------------------------- |
|   Target Bearing :  124.5                         |
|   Raw GNSS Hdg   :  120.3                         |
|   Current Offset :   +0.0                         |
|   Proposed Offset:   +4.2                         |
|                                                   |
|   [ SNAP TARGET ]          [ RESET (0.0) ]        |
|                                                   |
|             [ HOLD TO SAVE OFFSET ]               |
|                                                   |
|                   [ CANCEL ]                      |
|                                                   |
|             (Turn knob to nudge +-0.1)            |
+---------------------------------------------------+
```

### Safety Interaction Model
* **`[ SNAP TARGET ]` (Tap):** Snaps `Proposed Offset` on screen to `Target - Raw`. **Does NOT write to flash.**
* **`[ RESET (0.0) ]` (Tap):** Sets `Proposed Offset` on screen to `+0.0`. **Does NOT write to flash.**
* **Rotary Knob (Turn):** Nudges `Proposed Offset` on screen by $\pm 0.1^\circ$. **Does NOT write to flash.**
* **`[ HOLD TO SAVE OFFSET ]` (Long-Press 1.5s):** Commits the proposed offset to NVS flash and exits to the main screen. Displays dynamic filling progress and text `"HOLDING TO SAVE..."`. A short tap does not save.
* **`[ CANCEL ]` (Tap):** Discards any proposed changes immediately and returns to the main screen without writing to flash.
* **Auto-Timeout:** 15 seconds of inactivity automatically cancels and returns to the normal screen without saving.

### Touch Hitboxes
* **`[ SNAP TARGET ]`**: $X \in [55, 235]$, $Y \in [230, 278]$ ($180 \times 48\text{ px}$).
* **`[ RESET (0.0) ]`**: $X \in [245, 425]$, $Y \in [230, 278]$ ($180 \times 48\text{ px}$).
* **`[ HOLD TO SAVE OFFSET ]`**: $X \in [55, 425]$, $Y \in [284, 332]$ ($370 \times 48\text{ px}$).
* **`[ CANCEL ]`**: $X \in [130, 350]$, $Y \in [340, 385]$ ($220 \times 45\text{ px}$).

### Rotary Encoder Nudging
When in the calibration modal, turning the physical rotary knob (`ENCODER_CLK 42, ENCODER_DT 4`) increments or decrements the proposed offset by $\pm 0.1^\circ$ in real-time, giving manual fine-tuning capability without requiring an on-screen keypad.

---

## 6. Serial Console CLI

The calibration engine can be fully inspected and operated via the USB-C serial console ($115,200\text{ baud}$):

* **`cal` / `cal status`:** Prints a diagnostic report with Stored Offset, Target Bearing, Raw GNSS Heading, and Calibrated Heading.
* **`cal snap`:** Snaps the current raw heading to target bearing and saves to NVS.
* **`cal zero` / `cal reset`:** Resets the stored offset to $0.0^\circ$.
* **`cal set <float>` / `cal <float>`:** Manually sets the offset (e.g. `cal 4.2` or `cal -2.5`).

---

## 7. Verification & Testing

* **Host Unit Tests (`tests/test_calculations.cpp`):**
  * Automated testing of `normalizeOffset` across wrap-around boundaries (e.g., $355^\circ \to 5^\circ$).
  * Testing `applyBearingOffset` across cardinal points and negative offsets.
  * Testing `calcCurrentBearing` with default parameters and explicit offsets.
  * Verified: 15 / 15 unit tests pass via `tests/run_tests.sh`.
* **On-Device Self-Test Demo (`runVisualSelfTest`):**
  * Added Test Step 7 cycling through `CAL OFF`, `CAL +4.2`, and displaying the calibration modal on the round screen.

