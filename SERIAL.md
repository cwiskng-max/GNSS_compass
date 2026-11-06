# ESP32 Serial Interface & CLI Guide

The GNSS Compass firmware features an interactive command-line interface (CLI) accessible over the USB-C serial port. The CLI provides real-time GNSS receiver diagnostics, display self-test triggers, and antenna heading offset calibration stored in non-volatile storage (NVS).

---

## 1. Connection Settings

Connect the ESP32-S3 rotary display to your computer using a USB-C data cable.

| Parameter | Setting | Notes |
| :--- | :--- | :--- |
| **Port (Linux)** | `/dev/ttyACM0` (or `/dev/ttyACM1`) | Default USB CDC serial device |
| **Port (macOS)** | `/dev/cu.usbmodem*` | Native USB CDC port |
| **Port (Windows)** | `COMx` | Appears under Device Manager > Ports (COM & LPT) |
| **Baud Rate** | **115,200** | 8N1 (8 data bits, no parity, 1 stop bit) |
| **Flow Control** | None | Hardware and software flow control disabled |
| **Line Endings** | Any (`\r`, `\n`, or `\r\n`) | Firmware handles CR, LF, and CRLF transparently |

### Terminal Emulator Recommendations
* **Linux / macOS:** `tio /dev/ttyACM0 -b 115200`, `picocom -b 115200 /dev/ttyACM0`, or `minicom -D /dev/ttyACM0 -b 115200`
* **Windows:** PuTTY, Tera Term, or Arduino Serial Monitor
* **Cross-Platform:** Visual Studio Code Serial Monitor extension

The CLI includes **local character echoing** and supports **backspace/DEL** editing directly in terminal emulators.

---

## 2. Boot Output

Upon power-up or reboot, the ESP32 outputs initial startup identification and a summary prompt:

```text
GNSS Compass v1.0.0
Commands: 'status' (GPS stream diagnosis), 'test' (visual demo), 'sim' (simulation)
```

---

## 3. Command Reference

All commands are case-insensitive and can be issued interactively or piped via script.

### 3.1 Help Menu (`help` or `?`)
Displays a summary of available serial commands.

```text
> help

=== GNSS Compass Serial Commands ===
  status (or gps)       - Live GNSS receiver stream diagnostic
  test (or demo)        - Run visual display self-test demo
  cal [snap|zero|<val>] - View or configure antenna calibration
  sim [on|off|hdg|pos|grid|rover|status] - GPS simulation commands
  $<NMEA/KSXT sentence> - Inject raw GNSS sentence directly
  help (or ?)           - Show this help menu
====================================
```

---

### 3.2 GNSS Stream Diagnostics (`status`, `gps`, `diag`, `s`, `g`)
Inspects the live incoming UART stream from the Unicore UM982 GNSS receiver (`Serial2` / GPIO 44). Outputs byte counters, packet reception rates, fix conditions, and a link health verdict.

#### Example: Normal / RTK Fixed Lock
```text
> status

=== GNSS Link Diagnostic Report ===
Physical Link:  CONNECTED (Hardware Serial Link Active)
Total Bytes:    48291 (last byte 12 ms ago)

Message Streams:
  - $GNGLL (Position):   14 pkts (last 0.8s ago)
  - $KSXT (Dual-Ant):    28 pkts (last 34 ms ago)
  - $GNHDT/$THS:         0 pkts

Fix Status:
  - Time:                23:42 UTC
  - Position:            LOCKED (Grid: CM87wa92)
  - Dual-Antenna Heading:RTK FIXED (Pure White - High Precision)

Overall Verdict:        [PASS] FULLY OPERATIONAL (RTK Fixed)
====================================
```

#### Diagnostic Verdicts & Troubleshooting
| Verdict | Cause | Suggested Action |
| :--- | :--- | :--- |
| **`[PASS] FULLY OPERATIONAL (RTK Fixed)`** | Normal high-precision dual-antenna RTK heading. | None — operational. |
| **`[WARN] RTK HEADING FLOAT`** | Carrier phase ambiguity unresolved (obstructed sky view or multipath). | Check for local obstructions or RF reflections near antennas. |
| **`[WARN] HEADING ACQUIRING`** | Position fix acquired, waiting for dual-antenna baseline carrier lock. | Allow 10–30 seconds with clear view of the sky. |
| **`[FAIL] NO GNSS DATA`** | No bytes received across UART in > 3000 ms. | 1. Verify 4-pin GX12 cable is plugged in.<br>2. Check 5V power to receiver enclosure.<br>3. Check RX/TX wiring. |
| **`[FAIL] CORRUPTED DATA STREAM`** | Bytes received, but no valid NMEA/Unicore packets decoded. | Check receiver baud rate (must match 115,200 baud). |

---

### 3.3 Visual Display Self-Test (`test`, `demo`, `t`)
Triggers an automated visual demonstration on the round LCD screen. Runs sequentially through 7 test phases:

1. **Test 1: Primary UI Framework** (Grid layout and static elements)
2. **Test 2: Maidenhead Grid Resolution** (Local & remote grid squares)
3. **Test 3: Dual-Antenna Bearing Angles** (Cardinal headings $0^\circ \to 90^\circ \to 180^\circ \to 270^\circ$)
4. **Test 4: ARRL VHF/Microwave Distance** (Geodetic distance calculation)
5. **Test 5: Live Satellite Clock** (UTC time display)
6. **Test 6: Bearing Quality & Live Stream** (Color coding check and hardware link test)
7. **Test 7: Antenna Offset Calibration** (Pill rendering and modal dialog demo)

After completion, the self-test prints `Result: PASS` or `Result: FAIL` to the serial console and automatically restores previous live compass operation.

```text
> test

=== Running GNSS Compass Visual Self-Test ===
  -> Testing Primary UI Layout Framework
  -> Testing Maidenhead Grid Calculations
  -> Testing Bearing Needle Color Transitions
  -> Testing ARRL Contest Distance Computations
  -> Testing UTC Time Sync Display
  -> Testing Live GNSS Link & NMEA Parser
  -> Testing Calibration Pill Display (Calibrated: +4.2 deg)
  -> Demonstrating Calibration Modal Dialog
Result: PASS
=== Visual Self-Test Complete (Live state restored) ===
```

---

### 3.4 Antenna Heading Calibration (`cal`)
Manages the non-volatile antenna alignment offset in degrees. The offset corrects for mechanical misalignment between the dual-antenna cross-boom and the microwave antenna feedline.

#### Display Current Calibration (`cal`, `cal status`, `cal show`)
```text
> cal

=== Antenna Calibration Status ===
  Stored Offset :  +3.5 deg
  Target Bearing:  48.2 deg
  Raw GNSS Hdg  :  44.7 deg
  Calibrated Hdg:  48.2 deg
==================================
```

#### Snap Calibration to Target (`cal snap`)
Aligns the compass heading to match the current target bearing (requires an active GNSS heading fix). Calculates $\text{offset} = \text{target} - \text{raw\_heading}$, normalizes to $[-180^\circ, +180^\circ]$, updates display, and saves to NVS.

```text
> cal snap
[CAL] Snapped! New offset: +3.5 deg
```
*If no GNSS heading fix is active, the command responds with:*
```text
[CAL ERROR] Cannot snap: No valid GNSS heading fix.
```

#### Reset Calibration to Zero (`cal zero`, `cal reset`, `cal 0`)
Clears any stored offset back to $0.0^\circ$ and updates NVS.

```text
> cal zero
[CAL] Offset reset to 0.0 deg (Cleared).
```

#### Set Specific Offset (`cal <value>` or `cal set <value>`)
Sets an exact manual offset in degrees. Angles are automatically normalized to $[-180^\circ, +180^\circ]$.

```text
> cal set +4.2
[CAL] Offset set to  +4.2 deg

> cal -12.5
[CAL] Offset set to -12.5 deg
```

---

### 3.5 GPS Data Simulation & Rover Test (`sim` and `$<sentence>`)

The simulation interface allows desktop verification of **Rover Mode**, transit displacement tracking (evaluating the **16 km legal rule** and grid boundary crossings), antenna needle motion, and RTK quality states without connecting the physical UM982 receiver or driving outside.

#### Simulation State Control
* `sim` or `sim status`: Displays simulation mode state, injected packet counts, simulated coordinates, heading, and rover alert status.
* `sim on`: Enables simulation mode and **suppresses physical receiver input on `Serial2`**, preventing real GNSS data from interfering with tests.
* `sim off`: Disables simulation mode and restores live communication with the UM982 receiver.

```text
> sim status

=== GPS Simulation Mode Status ===
  Simulation Active : YES (Hardware Serial2 suppressed)
  Sim Packets Recv  : 142
  Last Sim Packet   : 180 ms ago
  Current Heading   : 145.2 deg (Quality: FIXED)
  Current Position  :  38.35400, -119.45800 (Grid: CM87wa)
  Rover Mode Active : YES (<16k alert tracking)
  Rover Distance    :  14.8 km (Alert: ACTIVE (RED on WHITE))
==================================
```

#### Simulated Values & Quick Commands
* `sim hdg <deg> [fixed|float|lost]`: Updates simulated antenna heading and color quality.
  * Example: `sim hdg 145.2 fixed` (Pure white needle)
  * Example: `sim hdg 90.0 float` (Bright yellow needle)
  * Example: `sim hdg 270.0 lost` (Bright red needle)
* `sim pos <lat> <lon>`: Sets current geographic coordinates and calculates Maidenhead grid.
  * Example: `sim pos 38.354 -119.458`
* `sim grid <grid>`: Moves current position to the center of a Maidenhead grid.
  * Example: `sim grid CM87wa`
* `sim rover [origin_grid] <dist_km>`: Configures rover tracking from an origin grid and sets simulated vehicle distance.
  * Example: `sim rover CM87wa 14.5` (Row 5 highlights **RED on WHITE**, alert active: $< 16\text{ km}$)
  * Example: `sim rover CM87wa 16.5` (Alert clears, distance $\ge 16\text{ km}$)
* `sim time <hh:mm>`: Sets the simulated satellite UTC clock.
  * Example: `sim time 18:30`

#### Direct Sentence Injection (`$<sentence>`)
Any line sent across serial beginning with `$` is passed directly to the unified sentence parser, exactly as if received from the Unicore receiver:

```text
$KSXT,20261106202506.00,-122.12500000,37.02083333,2100.00,145.20,0.00,0.00,0.022,,1,3,28,28*1C
```

Injecting raw sentences automatically enables simulation mode (`simModeActive = true`).

---

## 4. Host Simulator Script (`tools/gps_sim.py`)

A comprehensive scenario-based simulation script is available in `tools/gps_sim.py`. It communicates directly with the ESP32 over `/dev/ttyACM0` (or auto-detected port).

### 4.1 Running Pre-Canned Scenarios

```bash
# 1. Rover transit journey (drives from CM87wa across 16 km boundary at 65 km/h)
./tools/gps_sim.py rover --origin CM87wa --speed 65 --distance 25

# 2. Mountain canyon switchback / backtrack scenario (handles distance decreases)
./tools/gps_sim.py backtrack --origin CM87wa

# 3. 360-degree antenna rotator sweep (validates needle fluidity and 0/360 wrap)
./tools/gps_sim.py sweep --speed 6.0

# 4. RTK quality degradation & watchdog timeout cycle
./tools/gps_sim.py rtk

# 5. Interactive terminal drive controller (WASD keyboard control)
./tools/gps_sim.py drive --origin CM87wa

# 6. Dry run (prints generated NMEA/$KSXT sentences to stdout without serial port)
./tools/gps_sim.py rover --dry-run
```

### 4.2 Custom Parameter & Sentence Injection (`inject`)

The `inject` subcommand provides one-shot or sustained injection of custom coordinates, headings, simulated rover distances, UTC timestamps, and raw sentences.

| Argument | Description | Example |
| :--- | :--- | :--- |
| `--grid GRID` | Center on Maidenhead grid (4, 6, or 8 chars) | `--grid CM87wa` |
| `--lat LAT`, `--lon LON` | Set exact decimal geographic coordinates | `--lat 38.3540 --lon -119.4580` |
| `--hdg HDG` | Antenna heading in degrees (0–360) | `--hdg 145.2` |
| `--quality, -q` | RTK quality: `fixed` (white), `float` (yellow), `lost` (red) | `-q fixed` |
| `--duration, -d SEC` | Hold duration in seconds to continuously stream packet (prevents 1.5s staleness timeout) | `-d 10.0` |
| `--time, -t TIME` | Set simulated UTC clock display (`HH:MM` or `HH:MM:SS`) | `-t 21:45` |
| `--rover-origin GRID` | Origin grid for rover distance testing | `--rover-origin CM87wa` |
| `--rover-dist KM` | Simulated rover displacement distance from origin in km | `--rover-dist 14.5` |
| `--repeat, -r N` | Number of packets to send (default: 1) | `-r 5` |
| `--rate HZ` | Streaming rate in Hz for `--duration` or `--repeat` (default: 5.0) | `--rate 5.0` |
| `--altitude ALT` | Altitude in meters (default: 2100.0) | `--altitude 1850.0` |
| `--sats SATS` | Number of tracked satellites (default: 28) | `--sats 32` |
| `--raw RAW` | Direct injection of an arbitrary raw NMEA or `$KSXT` sentence | `--raw '$GNGLL,...'` |

#### Injection Examples

```bash
# 1. Quick coordinate and heading injection (Pure white fixed needle)
./tools/gps_sim.py inject --grid CM87wa --hdg 145.2 --quality fixed

# 2. Hold heading and position on screen for 10 seconds (prevents 1.5s watchdog timeout)
./tools/gps_sim.py inject --grid CM87wb --hdg 90.0 --quality fixed --duration 10

# 3. Simulate rover displacement relative to an origin (evaluates 16 km alert threshold)
./tools/gps_sim.py inject --rover-origin CM87wa --rover-dist 14.5 --hdg 45.0 --duration 15

# 4. Set specific UTC time on the clock display (Row 6)
./tools/gps_sim.py inject --grid CM87wa --time 23:45

# 5. Test degraded RTK reception (Bright yellow float needle)
./tools/gps_sim.py inject --grid CM87wa --hdg 210.0 --quality float --duration 8

# 6. Inject an arbitrary raw NMEA or $KSXT sentence directly
./tools/gps_sim.py inject --raw '$GNGLL,3821.24,N,11927.48,W,183000.00,A*23'
```

---

## 5. Scripting & Automation Examples

### Bash / Shell Quick Query
You can query the device directly from a Linux/macOS shell:

```bash
# Query live status
echo "status" > /dev/ttyACM0 && head -n 20 /dev/ttyACM0

# Zero out calibration
echo "cal zero" > /dev/ttyACM0
```

### Python Script Example
```python
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(1)  # Allow CDC connection to stabilize

# Send status command
ser.write(b"status\n")

# Read response until prompt or timeout
while True:
    line = ser.readline().decode('utf-8', errors='ignore')
    if not line:
        break
    print(line, end='')

ser.close()
```
