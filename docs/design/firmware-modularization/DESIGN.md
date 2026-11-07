# Firmware Architecture & Modularization Specification

**Status:** Proposed  
**Author:** Antigravity / Engineering Team  
**Reviewer:** Gary / Core Firmware Team  
**Target Release:** v0.4-refactor  

---

## 1. Executive Summary & Motivation

The `gps_compass.ino` sketch has grown to over 2,000 lines of mixed concerns: low-level ST7701 RGB panel timing, PCF8574 I2C expansion, CST8XX capacitive touch tracking, NMEA/KSXT sentence parsing, geodesy math, calibration persistence, and host simulator protocols.

This monolithic structure presents four primary technical challenges:
1. **Domain Entanglement (Coordinates vs Grids):** Geodesic calculations (WGS84 distance and bearing for navigation and rover alerts) are mixed with Maidenhead grid projection and ARRL contest scoring rules.
2. **Untestable Code Coupling:** Host-based unit tests (`tests/run_tests.sh`) currently resort to fragile `sed` scripts scraping code snippets from `gps_compass.ino` into temporary headers because pure math is coupled to Arduino hardware headers.
3. **Monolithic Control Flow:** The top-level `loop()` contains an internal `while (1)` loop spanning 250+ lines with deeply nested static state variables, bypassing FreeRTOS cooperative scheduling and watchdog yielding.
4. **Inconsistent Nomenclature:** Identifiers are a mix of truncated C abbreviations (`parselatlon`, `aton`, `rovdist`, `calcLocator`, `printrem`, `remotec`, `s_avg`, `c_avg`) without uniform casing.

This specification details the transition to a modular C++ architecture under `src/`, formalizes the coordinate domain model, establishes consistent coding standards, and outlines a non-breaking phased migration.

---

## 2. Core Architectural Principles

### 2.1 Coordinate Truth vs UI Abstraction
* **Latitude/Longitude (`GeoPoint`) is the physical source of truth:** All physical distance, bearing, and rover transit calculations MUST operate strictly on floating-point latitude and longitude.
* **Maidenhead Grid (`GridSquare`) is purely a human-facing abstraction:** Grid squares exist for human visual display and operator keypad entry.
* **No Grid Truncation for Rover Logic:** The local station location originates as high-precision WGS84 coordinates from the GPS receiver. It must **never** be snapped to the local Maidenhead grid center when evaluating physical distance or rover transit thresholds (e.g., the 16 km rule). Snapping to a 6-character grid center introduces up to $\sim 2.5\text{ km}$ of artificial quantization error.
* **Target Asymmetry:** Target stations dialled into the UI are specified only to grid resolution (4, 6, or 8 characters). When entered, the target grid is mapped to its bounding box center `GeoPoint`. The system tracks this distinction explicitly.

```
+--------------------------------------------------------------------------------+
|                               DATA INGESTION                                   |
|                                                                                |
|   GPS Receiver (KSXT / NMEA)               Operator Touchpad / Rotary Knob     |
|   lat: 38.35412, lon: -119.45821           Grid String: "CM87wb"               |
|               |                                      |                         |
|               v                                      v                         |
|      [ Local GeoPoint ]                     [ Target GridSquare ]              |
|   (High-precision source)                            |                         |
|         |           \                                v                         |
|         |            \                     [ Target GeoPoint ]                 |
|         |             \                    (Grid center coordinate)            |
|         v              \                             |                         |
|  [ Local GridSquare ]   \                           /                          |
|  (UI presentation only)  \                         /                           |
+---------------------------\-----------------------/----------------------------+
                             v                     v
               +-----------------------------------------------+
               |             DISTANCE CALCULATIONS             |
               |                                               |
               | 1. Geodesic / Great-Circle (Navigation):     |
               |    Local GeoPoint <---> Target GeoPoint       |
               |    Model: WGS84 / Haversine (true meters)     |
               |    Used for: Antenna bearing, physical range, |
               |    and Rover <16 km transit alerts.           |
               |                                               |
               | 2. Contest Scoring (ARRL Standard):           |
               |    Local GridSquare <---> Target GridSquare   |
               |    Model: Spherical R=6371.0 km, 1.0 km clamp |
               |    Used for: ARRL contest points calculation. |
               +-----------------------------------------------+
```

### 2.2 Explicit Distance Calculation Taxonomy
The codebase must provide distinct, self-documenting functions so reviewers and contributors cannot conflate contest rules with physical navigation:

| Calculation Type | Function Signature | Earth Model | Purpose |
| :--- | :--- | :--- | :--- |
| **Geodesic Navigation** | `calculateWgs84(const GeoPoint &from, const GeoPoint &to, double &distMeters, double &initialBearingDeg)` | WGS84 Ellipsoid / Haversine ($R=6378137\text{ m}$) | Real-time antenna pointing and true ground distance. |
| **Reverse Bearing** | `calculateReverseBearing(const GeoPoint &from, const GeoPoint &to)` | Great-Circle Azimuth | Azimuth from remote target back toward local station. |
| **ARRL Contest Distance** | `calculateArrlContestDistance(const char *myGrid, const char *targetGrid)` | Spherical ($R=6371.0\text{ km}$) with $1.0\text{ km}$ minimum floor | Contest scoring distance per official ARRL rules. |
| **Rover Distance Check** | `isRoverTransitInsideAlert(const GeoPoint &currentPos, const GeoPoint &originPos, double thresholdKm = 16.0)` | WGS84 Geodesic | Evaluates whether rover has moved beyond the 16 km boundary from activation origin. |

---

## 3. Directory & Module Breakdown

All subsystem implementations move into `src/`. Headers expose clean interfaces, and dependencies on Arduino hardware libraries are isolated.

```text
GNSS_compass/
├── gps_compass.ino                  # Thin application orchestrator (~150 lines)
├── src/
│   ├── types.h                      # Common primitives: GeoPoint, BearingQuality, GpsLinkVerdict
│   ├── config.h                     # Pinouts, layout dimensions, UI colors, timeouts
│   │
│   ├── geo/                         # Pure C++ Geodesy & Grids (100% Host-Testable)
│   │   ├── geo_math.h / .cpp        # Lat/lon geodesy, WGS84, ARRL distance, circular math
│   │   └── maidenhead.h / .cpp      # GeoPoint <-> Maidenhead 4/6/8-char conversions
│   │
│   ├── protocol/                    # Pure C++ Protocol Parsers (100% Host-Testable)
│   │   ├── nmea_parser.h / .cpp     # NMEA 0183 & Unicore KSXT tokenizer & sentence decoders
│   │   └── serial_cli.h / .cpp      # Host USB CLI parser (GPS:*, CAL:*, SIM:*, TEST:*)
│   │
│   ├── gnss/                        # GPS Stream & Hardware
│   │   ├── gps_manager.h / .cpp     # Serial2 UART ingest, staleness watchdog, link state
│   │   └── simulator.h / .cpp       # Simulation mode state, virtual rover injection
│   │
│   ├── system/                      # Persistence & Calibration
│   │   └── calibration.h / .cpp     # Heading offset math & ESP32 Preferences (NVS)
│   │
│   └── ui/                          # Display & Input Subsystems
│       ├── input_manager.h / .cpp   # Rotary encoder ISR & CST8XX touch gesture decoder
│       ├── display_ui.h / .cpp      # ST7701 panel driver, PCF8574 reset, main screen
│       └── cal_modal.h / .cpp       # Calibration modal dialog & button progress rendering
└── tests/
    ├── test_calculations.cpp        # Host unit test suite (directly includes src/geo & src/protocol)
    └── run_tests.sh                 # Clean native g++ compilation script
```

---

## 4. Module Specifications & Responsibilities

### 4.1 Data Primitives (`src/types.h`)
```cpp
#pragma once
#include <cstdint>

struct GeoPoint {
    double latitude;   // Degrees: -90.0 to +90.0 (North positive)
    double longitude;  // Degrees: -180.0 to +180.0 (East positive)
    bool isValid;
};

enum class BearingQuality : uint8_t {
    None = 0,    // Uninitialized / no fix
    Fixed = 1,   // RTK Fixed (highest confidence)
    Float = 2,   // RTK Float (medium confidence)
    Lost = 3     // Stale timeout or lost differential fix
};

enum class GpsLinkVerdict : uint8_t {
    Disconnected = 0,
    NoisyCorrupt = 1,
    Acquiring = 2,
    PositionOnly = 3,
    HeadingFloat = 4,
    HeadingFixed = 5
};
```

### 4.2 Geodesy & Grid Services (`src/geo/`)
* **`geo_math.h`**:
  * `double toRadians(double degrees);`
  * `double toDegrees(double radians);`
  * `void calculateWgs84(const GeoPoint &from, const GeoPoint &to, double &distanceMeters, double &bearingDeg);`
  * `double calculateArrlContestDistance(const char *gridA, const char *gridB);`
  * `float normalizeHeading(float headingDeg);`
  * `float normalizeOffset(float offsetDeg);`
  * `double applyBearingOffset(double rawBearing, float offsetDeg);`
* **`maidenhead.h`**:
  * `bool geoPointToGrid(const GeoPoint &point, char *outGrid, uint8_t length = 8);`
  * `bool gridToGeoPoint(const char *grid, GeoPoint &outPoint);`

### 4.3 Protocol Parsers (`src/protocol/`)
* **`nmea_parser.h`**:
  * Decoupled from hardware serial; accepts `const char *line`.
  * `bool parseKsxtHeading(const char *sentence, float &headingOut, BearingQuality &qualityOut);`
  * `bool parseKsxtTime(const char *sentence, int &hoursOut, int &minutesOut);`
  * `bool parseKsxtLocation(const char *sentence, GeoPoint &pointOut);`
  * `bool parseHdtHeading(const char *sentence, float &headingOut, BearingQuality &qualityOut);`
  * `bool parseThsHeading(const char *sentence, float &headingOut, BearingQuality &qualityOut);`

### 4.4 GNSS & Link Management (`src/gnss/`)
* **`gps_manager.h`**:
  * Owns the `HardwareSerial(2)` peripheral.
  * Ingests streaming bytes and assembles sentences up to 1024 bytes with boundary checks.
  * Tracks packet/byte statistics and sentence timestamps for `GpsLinkVerdict` determination.
  * Enforces the 1.5-second heading staleness watchdog timer.
  * Can be fed by either hardware UART or `simulator.cpp`.

### 4.5 Input & Touch Gesture Decomposition (`src/ui/input_manager.h`)
Replaces the manual coordinate checking in `loop()` with an event-driven model:
```cpp
enum class UserEvent {
    None,
    RotateClockwise,
    RotateCounterClockwise,
    TouchGridCharacter,
    TouchCalibrationPill,
    HoldSaveButtonProgress,
    SaveButtonCommitted,
    SnapOffsetTriggered,
    ResetOffsetTriggered,
    CancelModalTriggered
};
```

---

## 5. Standardized Naming Convention & Identifier Mapping

### 5.1 Naming Rules
* **Types, Structs, Enums:** `PascalCase` (e.g., `GeoPoint`, `BearingQuality`, `GpsManager`)
* **Functions & Methods:** `camelCase` (e.g., `calculateWgs84`, `updateBearings`, `geoPointToGrid`)
* **Variables & Members:** `camelCase` (e.g., `currentPosition`, `targetGrid`, `isRoverAlertActive`)
* **Constants & Macros:** `UPPER_SNAKE_CASE` (e.g., `HEADING_TIMEOUT_MS`, `RICH_YELLOW`)

### 5.2 Legacy Refactoring Mapping Table

| Legacy Name | New Standardized Name | Type / Scope | Description |
| :--- | :--- | :--- | :--- |
| `aton(x)` | *(Inline conversion)* `(x - '0')` | Helper | Eliminated in favor of standard parser math |
| `parselatlon()` | `parseLegacyLatLon()` | `nmea_parser` | Parses compact NMEA degree/minute strings |
| `calcLocator()` | `geoPointToGrid()` | `maidenhead` | Converts `GeoPoint` to Maidenhead grid string |
| `calcLatLon()` | `gridToGeoPoint()` | `maidenhead` | Converts Maidenhead grid string to center `GeoPoint` |
| `calculateARRLDistance()` | `calculateArrlContestDistance()` | `geo_math` | Spherical Earth contest scoring distance with 1 km clamp |
| `calculateWGS84()` | `calculateWgs84()` | `geo_math` | Geodesic distance and forward azimuth |
| `mygrid` | `myGrid` | UI State | Formatted 8-character Maidenhead string for local station |
| `remote` | `targetGrid` | UI State | 6-character target Maidenhead string dialed on UI |
| `mylatitude`, `mylongitude` | `currentPosition` | `GeoPoint` | High-precision GPS latitude & longitude |
| `remlatitude`, `remlongitude` | `targetPosition` | `GeoPoint` | Calculated center coordinates of target grid |
| `rovdist` | `isRoverAlertActive` | `bool` | Rover mode flag indicating <16 km proximity alert active |
| `remotec` | `cursorColumn` | `uint8_t` | Currently selected character index in the target grid editor |
| `s_avg`, `c_avg` | `sinSum`, `cosSum` | `float` | Running circular mean accumulators for bearing smoothing |
| `printrem()` | `renderTargetGrid()` | `display_ui` | Renders the target grid row and cursor caret |
| `printLocalGrid()` | `renderMyGrid()` | `display_ui` | Renders myGrid in top status row |

---

## 6. Simplified Orchestrator & Loop Model

The top-level `gps_compass.ino` is reduced to an initialization routine and a cooperative non-blocking loop without nested infinite `while (1)` blocks:

```cpp
#include "src/config.h"
#include "src/types.h"
#include "src/gnss/gps_manager.h"
#include "src/system/calibration.h"
#include "src/protocol/serial_cli.h"
#include "src/ui/input_manager.h"
#include "src/ui/display_ui.h"

GpsManager    gpsManager;
Calibration   calibration;
SerialCli     serialCli;
InputManager  inputManager;
DisplayUi     displayUi;

void setup() {
    Serial.begin(115200);
    configHardwarePins();
    
    displayUi.init();
    calibration.begin();
    gpsManager.begin(GPS_BAUD);
    inputManager.begin();
    
    displayUi.drawDashboard();
}

void loop() {
    // 1. Ingest GPS UART bytes & check staleness timeouts
    gpsManager.update();

    // 2. Process USB host commands (SIM, CAL, GPS, TEST)
    serialCli.update();

    // 3. Handle physical input (rotary knob & touchscreen)
    inputManager.update();

    // 4. Update display if state changed or 1Hz time elapsed
    displayUi.update();
}
```

---

## 7. Impact on Testing & Continuous Integration

### 7.1 Elimination of `sed` Scraping
Currently, `tests/run_tests.sh` extracts lines from `gps_compass.ino` to build a mock header:
```bash
# CURRENT (Fragile):
sed -n '/void calcLocator/,/^}/p' "$REPO_DIR/gps_compass.ino" >> "$SCRIPT_DIR/extracted_math.h"
```

With the new architecture, `tests/test_calculations.cpp` links directly against `src/geo/` and `src/protocol/`:
```bash
# NEW (Clean & Standard):
g++ -O2 -Wall -Wextra -I.. \
    ../src/geo/geo_math.cpp \
    ../src/geo/maidenhead.cpp \
    ../src/protocol/nmea_parser.cpp \
    test_calculations.cpp -o run_tests
./run_tests
```

### 7.2 Verification with GPS Simulation Suite
The simulator engine (`tools/gps_sim.py`) will run directly against the refactored firmware:
1. `tests/run_tests.sh` confirms mathematical accuracy on the host.
2. `tools/gps_sim.py --port ... --suite rover_transit` validates serial protocol parsing, rover distance tracking, and alert states end-to-end on target hardware.

---

## 8. Phased Implementation Plan

| Phase | Description | Deliverables | Risk / Verification |
| :--- | :--- | :--- | :--- |
| **Phase 1** | Pure Geodesy & Protocol Extraction | `src/types.h`, `src/geo/`, `src/protocol/nmea_parser.*`. Update `tests/`. | **Zero hardware risk.** Verified by passing 15/15 host unit tests. |
| **Phase 2** | Configuration & State Separation | `src/config.h`, `src/system/calibration.*`, `src/gnss/simulator.*`. | Verified by unit tests and CLI calibration tests. |
| **Phase 3** | Serial & GNSS Manager | `src/gnss/gps_manager.*`, `src/protocol/serial_cli.*`. | Verified by `gps_sim.py` interactive injection. |
| **Phase 4** | Input & UI Subsystems | `src/ui/input_manager.*`, `src/ui/display_ui.*`, `src/ui/cal_modal.*`. | Verified by visual self-test and touch interaction. |
| **Phase 5** | Orchestrator Clean-up | Slim `gps_compass.ino` and remove `while (1)` in `loop()`. | Verified by full rover transit test suite. |

