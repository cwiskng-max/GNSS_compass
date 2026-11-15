# Field Operation & Display Guide

This guide covers day-to-day operation of the GNSS Compass—what is shown on the circular display, how to use the touchscreen and rotary bezel, how to calibrate antenna heading offsets, and how Rover Mode and contest logging work in the field.

* **Hardware build & wiring:** [BOM.md](BOM.md)
* **Receiver setup & antenna mounting:** [RECEIVER.md](RECEIVER.md)
* **USB-C / Bluetooth (BLE) CLI & voice QSO logging:** [SERIAL.md](SERIAL.md)

---

## 1. Overview & 30-Second Quick Start

### Why Dual-Antenna GNSS?
Operating weak-signal VHF, UHF, and microwave bands—especially portable on mountaintops or as a mobile rover—demands accurate antenna pointing. At microwave frequencies (such as 10 GHz and up), dish beamwidths can be just a few degrees wide. Traditional pointing methods have drawbacks:
* **Beacon Calibration Delays:** Hunting for distant beacons to align mechanical rotators fails when beacons are off the air or terrain-obstructed.
* **Magnetic Distortion:** Magnetic compasses and fluxgate sensors are unreliable near vehicle sheet metal, masts, iron towers, and high-current power cables.
* **Cluttered Operating Desks:** Field operators often juggle a rotator indicator, handheld GPS for grid squares, a laptop/phone for bearing and distance, and a separate UTC clock.

**The GNSS Compass combines these into one self-contained instrument:**
* **Zero Magnetic Calibration:** Uses carrier-phase interferometry between two GNSS antennas to compute true azimuth heading accurate to $< 0.5^\circ$ ($0.1^\circ$ resolution) from first principles.
* **All-in-One Telemetry:** Displays live antenna heading, target azimuth, 8-digit local Maidenhead grid, 6-digit target grid, UTC time, reverse bearing/distance, Rover 16 km qualification status, and calibration offset on a single 480×480 circular screen.

### 30-Second Quick Start
1. **Apply Power:** Connect 5 V / 12 V DC power. The display initializes in under 2 seconds.
2. **Acquire GNSS Heading Lock:**
   * On initial boot, heading shows `---.-` (White) while the receiver acquires satellites.
   * Outdoors, heading transitions to **Yellow** (RTK Float) and then **Pure White** (RTK Fixed).
3. **Set Your Target Grid:** Tap any character in **Row 2** (Target Grid) and turn the outer rotary bezel to dial in the destination 6-digit Maidenhead grid (or send `target <grid>` over USB-C / BLE).
4. **Aim Your Antenna:** Turn your antenna rotator until the **Live Heading (Row 3, left)** matches the **Target Azimuth (Row 3, right)**.

---

## 2. Display Layout & Screen Anatomy

The 480×480 circular display organizes station telemetry into functional rows and interactive touch zones:

| Row / Zone | Function | Example | Description |
| :--- | :--- | :---: | :--- |
| **Row 1** | **Local Grid** | `CM87wa00` | 8-digit Maidenhead locator from live GNSS coordinates (`JJ00AA00` before fix). **Tap** to start Rover Mode; **long-press (1.5 s)** to open Rover History. |
| **Row 2** | **Target Grid** | `FN31pr` | 6-digit destination grid square (`ZA00AA` default). **Tap** any character to select it for rotary editing (yellow caret `^`). |
| **Row 3** | **Live Heading & Target Azimuth** | `142.6  142.8` | **Left:** Real-time antenna heading ($0.0^\circ$–$359.9^\circ$, color-coded by RTK state).<br>**Right:** Great-circle azimuth from your position to the target grid. |
| **Cal Pill** | **Calibration Offset Pill** | `OFF` / `+4.2°` | Centered pill between Row 3 and Row 4. Shows active heading offset (`OFF` in gray when `0.0°`, or offset in cyan). **Tap** to open the Calibration Modal. |
| **Row 4** | **UTC Clock** | `19:45 Z` | Universal Coordinated Time derived from GNSS (`--:--Z` before fix). |
| **Row 5** | **Reverse Bearing & Distance** *(Normal Mode)* / **Rover Card** *(Rover Mode)* | `322.8  3892 k` or `TRANSIT    6.4 k` | **Normal Mode:** Reverse bearing from target back to you and geodesic distance in km.<br>**Rover Mode:** 4-state 16 km qualification card (`TRANSIT`, `QUALIFIED`, `REVISIT`, `NO GPS`). |
| **Footer** | **Version Footer** | `v0.5-dev` | Firmware version identifier. **Tap** to run the 9-step on-device visual self-test. |

---

## 3. Heading Color Indicators (RTK Quality)

Row 3 (left) dynamically color-codes the carrier-phase RTK solution quality:

* **Pure White (`142.6`) — RTK Fixed (Optimal):** Both antennas have resolved full integer carrier-phase ambiguity. Accuracy is $< 0.5^\circ$ ($0.1^\circ$ resolution), suitable for narrow-beam microwave dishes. *(Also shown for `---.-` during initial boot acquisition and for legacy `$GNHDT` streams.)*
* **Bright Yellow (`142.6`) — RTK Float (Acquiring):** Dual-antenna baseline is tracking satellites with floating ambiguities. Usable for rough VHF/UHF orientation while converging to Fixed.
* **Bright Red (`142.6` or `---.-`) — Lost / Stale Watchdog:** Heading packets are missing, invalid, or older than **1.5 seconds**. Protects against pointing with frozen stale data.

---

## 4. Touchscreen & Rotary Bezel Controls

### 4.1 Setting the Target Grid Square (Row 2)
1. **Select Character (Touch) or Toggle Edit Mode (Encoder Press):**
   * **Tap** directly on any of the 6 characters in **Row 2**, or **press the rotary knob** (in Normal Mode).
   * A yellow caret (`^`) appears beneath the active character and the character turns yellow.
2. **Change Value (Rotary Bezel):** Turn the outer rotary bezel clockwise/counter-clockwise:
   * **Positions 1 & 2 (Field):** `A`–`R`
   * **Positions 3 & 4 (Square):** `0`–`9`
   * **Positions 5 & 6 (Subsquare):** `A`–`X`
3. **Auto-Lock Timeout:** After **5 seconds** of inactivity, edit mode locks and the yellow caret disappears to prevent accidental changes.

### 4.2 Antenna Heading Calibration (`CalModal`)
If your GNSS boom has a small mechanical alignment offset relative to your RF feed/dish, you can trim it on-device without moving hardware:
1. **Open Modal:** Tap the **Calibration Pill** between Row 3 and Row 4.
2. **Adjust Proposed Offset:**
   * **Rotary Bezel:** Turn to nudge the proposed offset in **$0.1^\circ$ steps**.
   * **`SNAP TARGET`:** Tap to automatically compute the offset that aligns your current live GNSS heading with the current Target Azimuth (Row 3, right).
   * **`RESET (0.0)`:** Tap to reset the proposed offset to `0.0°`.
3. **Save or Cancel:**
   * **`HOLD TO SAVE OFFSET`:** Press and **hold for 1.5 seconds** (watch the green progress fill) to commit the offset to non-volatile storage (NVS).
   * **`CANCEL`:** Tap to discard changes and return to the main screen. The modal also auto-closes after **15 seconds** of inactivity without saving.

*(Calibration can also be inspected or adjusted via CLI with `cal`, `cal snap`, `cal <deg>`, and `cal zero`; see [SERIAL.md](SERIAL.md#35-antenna-heading-calibration-cal).)*

### 4.3 Visual Self-Test (Version Footer)
Tap the **Version Footer** at the bottom of the screen to launch the 9-step on-device visual self-test, which verifies distance math, formatting, bearing colors, live GNSS link health, calibration UI, and Rover Mode cards, then restores live telemetry.

---

## 5. Rover Mode & 16 km Contest Transit Tracking

In VHF/UHF/Microwave contests, rovers must travel at least **16 km** (~9.9 miles) between operating stops when re-activating or counting contacts from the same grid square.

### 5.1 Starting Rover Mode
* **Tap Row 1 (Local Grid):** Snapshots your current coordinates as the departure origin, copies your 6-digit departure grid into Row 2 (Target Grid), logs the stop in NVS history, and switches Row 5 to the **Rover Qualification Card**.

### 5.2 Row 5 Rover Qualification Card (4-State Matrix)
Row 5 displays a high-contrast white card with a status word and straight-line displacement in kilometers:

| State | Card Text | Text Color | Meaning |
| :--- | :--- | :--- | :--- |
| **In Transit (< 16 km)** | `TRANSIT   6.4 k` | **Red** on White | Straight-line displacement from departure point is under `16.0 km`. |
| **Qualified (New Grid)** | `QUALIFIED 18.2 k` | **Forest Green** on White | Displacement is `≥ 16.0 km` and the current 6-digit grid has not been visited earlier in this session. |
| **Qualified (Revisit)** | `REVISIT   18.2 k` | **Royal Blue** on White | Displacement is `≥ 16.0 km`, but the current 6-digit grid was already visited earlier in the Rover log. |
| **GPS Lost / Stale** | `NO GPS    6.4 k` | **Deep Amber** on White | GNSS position fix is lost or stale (`> 3 seconds` old). |

### 5.3 Two-Step Target Exit Protection
While in Rover Mode, accidental touches won't wipe out your transit tracking:
* Pressing the rotary knob is locked out.
* Tapping a character in **Row 2** arms the yellow caret so you can inspect or prepare a target change, **without** exiting Rover Mode.
* Rover Mode only exits when you physically **rotate the bezel** to change the target grid.

### 5.4 Rover History & 1-Touch Undo (`RoverHistoryModal`)
If you tapped Row 1 accidentally before leaving a site:
1. **Long-press Row 1 (Local Grid) for 1.5 seconds** to open the **Rover History Modal**, which shows the most recent stop, displacement/qualification status, and total logged stops.
2. Press and **hold `HOLD TO DISCARD LAST STOP` for 1.5 seconds** to remove the last stop and re-arm tracking from the previous stop, or tap **`[ CANCEL ]`** to close.

*(Rover tracking can also be controlled via BLE/USB CLI using `rover status|start|stop|history|undo|max|clear`; see [SERIAL.md](SERIAL.md).)*

---

## 6. Wireless Smartphone Control & Voice QSO Logging

For mast-mounted or dash-mounted operation, the display advertises a **Bluetooth Low Energy (BLE UART)** service (`GNSS-Compass-xxxxxx`) alongside its USB-C port. From a smartphone terminal app or Web Bluetooth browser:
* Run diagnostics (`status`, `gps config`, `gps check`).
* Set target grids remotely (`target FN31pr`).
* Dictate contest contacts hands-free using NATO phonetics (`log whiskey six oscar papa stroke rover 10`), which displays a confirmation pop-up card on the round screen (auto-dismisses after 5 seconds by default, or tap `OK` / touch screen / turn or press the knob to dismiss immediately) and exports to **Cabrillo**, **ADIF**, or **CSV**.

See **[SERIAL.md](SERIAL.md)** for full BLE setup, voice dictation tips, macro profiles, and command syntax.

---

## 7. Quick Reference Cheat Sheet

### Touch & Rotary Bezel Actions
| Control | Action |
| :--- | :--- |
| **Tap Row 1 (Local Grid)** | Start Rover Mode / snapshot departure origin |
| **Long-press Row 1 (1.5 s)** | Open Rover History & Undo Modal |
| **Tap Row 2 (Target Grid)** | Select digit (`^`) for bezel editing (auto-locks after 5 s) |
| **Press Rotary Knob** | Toggle Target Grid edit mode (Normal Mode) / dismiss QSO card |
| **Rotate Bezel** | Edit selected grid character / nudge Calibration Offset (`±0.1°`) |
| **Tap Calibration Pill** | Open Antenna Calibration Modal (15 s timeout) |
| **Tap Version Footer** | Run 9-step visual self-test |

### Color Reference
| Element & Color | Meaning |
| :--- | :--- |
| **White Heading (Row 3)** | RTK Fixed lock ($< 0.5^\circ$ accuracy) or initial `---.-` |
| **Yellow Heading (Row 3)** | RTK Float (converging) |
| **Red Heading / `---.-` (Row 3)** | Lost / stale GNSS heading (`> 1.5 s` watchdog) |
| **Yellow Character & Caret `^` (Row 2)** | Target Grid edit mode active |
| **Cyan Pill (between Row 3 & 4)** | Non-zero antenna calibration offset active |
| **Red / Green / Blue / Amber on White (Row 5)** | Rover Mode: `TRANSIT` (<16 km) / `QUALIFIED` (≥16 km new) / `REVISIT` (≥16 km visited) / `NO GPS` (>3 s stale) |
