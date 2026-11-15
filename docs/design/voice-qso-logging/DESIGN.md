# Voice-Friendly Contest QSO Logging & Remote Target Setting — Technical Design

* **Feature:** Voice-Dictated Contest QSO Logging & Remote Target Setting via Bluetooth Low Energy & USB Serial
* **Status:** Proposed / Ready for Review
* **Target Hardware:** ESP32-S3 (16 MB Flash, 8 MB OPI PSRAM), Elecrow 2.1" Round Touch Display (480×480)
* **Architecture Base:** Modular C++ Architecture (`src/protocol/serial_cli.h`, `src/system/app_controller.h`)
* **Associated Docs:** [Serial CLI Guide](../../SERIAL.md), [Bluetooth Serial Design](../bluetooth-serial/DESIGN.md), [Rover Mode Design](../rover-transit/DESIGN.md)

---

## 1. Executive Summary & Field Motivation

During amateur microwave and VHF/UHF contests (such as the ARRL 10 GHz and Up Contest, ARRL June/September VHF contests, and regional microwave sprints), operators frequently deploy portable or rover stations to mountain summits, coastal overlooks, and roadside turnouts.

Operating in these environments imposes extreme physical constraints:
* Operators are actively peaking dish azimuth/elevation, listening for faint CW/SSB signals near the noise floor, or working through wind and harsh sunlight.
* Entering log data via laptop keyboards or smartphone touchscreens in high ambient glare while wearing gloves or holding gear is slow, error-prone, and distracting.
* Most microwave contacts take place on a single primary band (**10 GHz**), with occasional coordinated escalations to higher millimeter-wave bands (**24 GHz, 47 GHz, 76 GHz, 122 GHz, 134 GHz, 241 GHz, and Laser**) or lower VHF/UHF bands.

```
                      ┌──────────────────────────────────────────────┐
                      │    Smartphone (Voice Dictation / Gboard)     │
                      │   "log k6mg 10"  or  "target CM88qk"         │
                      └───────────────────────┬──────────────────────┘
                                              │ BLE UART (NUS)
                                              ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ ESP32-S3 GNSS Compass Firmware                                                  │
│                                                                                 │
│   ┌─────────────────────┐       ┌───────────────────────────────────────────┐   │
│   │   LineBuffer / BLE  │  ──>  │      VoiceQsoParser                       │   │
│   └─────────────────────┘       │  - Strips dictation punctuation           │   │
│                                 │  - NATO phonetics ("Kilo 6 Mike Golf...") │   │
│                                 │  - Disambiguates without requiring "GHz"  │   │
│                                 │  - Flexible order (call first / band 1st) │   │
│                                 │  - Freq -> Band auto-mapping (10368->10G) │   │
│                                 │  - Default band logic (10 GHz)            │   │
│                                 └───────────────┬───────────────────────────┘   │
│                                                 │                               │
│                   ┌─────────────────────────────┴─────────────────────────────┐ │
│                   ▼                                                           ▼ │
│   ┌───────────────────────────────┐           ┌─────────────────────────────┐   │
│   │    Target Setting Engine      │           │    Direct-to-Save Engine    │   │
│   │  - Sets Grid in AppController │           │  - Auto-commits complete QSO│   │
│   │  - Updates Display Row 2      │           │  - Echoes text confirmation │   │
│   │  - Immediate calculation      │           │  - Prompts only if partial  │   │
│   └───────────────┬───────────────┘           └──────────────┬──────────────┘   │
│                   │                                          │                  │
│                   ▼                                          ▼                  │
│   ┌───────────────────────────────┐           ┌─────────────────────────────┐   │
│   │ Display UI (Round Screen)     │           │ QsoLogManager (NVS Storage) │   │
│   │ - Target grid & bearing needle│           │ - Persistent QSO ring buffer│   │
│   │ - 10s auto-dismiss pop-up card│           │ - Cabrillo, ADIF, CSV export│   │
│   └───────────────────────────────┘           │ - Delete by number, undo    │   │
│                                               └─────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

The GNSS Compass already continuously computes and maintains the critical situational context required for contest logging:
1. **My Grid Locator:** Real-time 6/8-character Maidenhead grid derived from dual-antenna GNSS RTK/GPS fixes.
2. **His Grid Locator:** Target grid square currently selected and dialed in on the compass display.
3. **UTC Timestamp:** Precise GPS-synchronized date and time (hours, minutes, seconds, epoch).
4. **Geodesics:** Exact geodesic path distance (km) and true bearing (azimuth).

By combining the **Bluetooth Serial Interface (BLE UART)** with the voice dictation built into modern smartphone keyboards (Google Gboard, Apple iOS Dictation), the operator can simply tap the microphone icon in their mobile terminal and speak single natural commands like:
> **"log k6mg"** $\rightarrow$ **Instantly logged to 10 GHz with on-display visual confirmation!**

---

## 2. Voice User Experience & Operational Flow

Field operation requires maximum speed and minimum keystrokes. Rather than forcing multi-step confirmation rituals, the system uses a **direct-to-save** pipeline with immediate feedback and instant undo/delete capabilities.

```mermaid
stateDiagram-v2
    [*] --> Idle: Boot / Ready

    Idle --> DirectSave: "log <call> [band/freq]" (Complete)
    Idle --> DirectSave: "log <call>" (Uses Default 10 GHz)
    Idle --> IncompleteStaging: "log" (Missing Callsign)
    Idle --> SetTarget: "target <gridsquare>"

    state DirectSave {
        [*] --> CommitNVS
        CommitNVS --> TerminalEcho
        TerminalEcho --> TriggerScreenModal
    }
    DirectSave --> Idle: Done (1-shot)

    state IncompleteStaging {
        [*] --> PromptMissing
        PromptMissing --> IncompleteStaging: Partial Input
        PromptMissing --> DirectSave: Callsign Provided
        PromptMissing --> Idle: Hotword "cancel"
    }

    state SetTarget {
        [*] --> UpdateGridEditor
        UpdateGridEditor --> RecalculateGeodesy
        RecalculateGeodesy --> RedrawScreen
    }
    SetTarget --> Idle: Done
```

### 2.1 Fast-Path: Direct-to-Save (Zero Prompting)
When complete arguments are provided, the QSO is **immediately committed to persistent storage** without requiring a second "save" confirmation word:

#### Example 1: Default 10 GHz Band (A Single Spoken Phrase)
* **Operator speaks:** `"log k6mg"`
* **Compass terminal echoes:**
  ```text
  [QSO LOGGED #15] K6MG | 10 GHz (10368.100 MHz) | 21:34z | CM87wa -> CM88qk (118.2 km @ 342 deg)
  > 
  ```
* **Compass 2.1" Display:** Instantly shows the **QSO Pop-Up Card** on the LCD screen (described in Section 3) with a 10-second auto-dismiss timer.
* **Result:** Contact logged in one single phrase!

#### Example 2: Band Specified Without "GHz"
The suffix `"GHz"` is **not required**. Numbers like `10`, `24`, `47`, `76`, etc., are automatically recognized as microwave gigahertz bands:
* **Operator speaks:** `"log k6mg 24"` *(or reversed: `"log 24 k6mg"`)*
* **Compass terminal echoes:**
  ```text
  [QSO LOGGED #16] K6MG | 24 GHz (24192.100 MHz) | 21:35z | CM87wa -> CM88qk (118.2 km @ 342 deg)
  > 
  ```

#### Example 3: Exact Frequency in Digits
* **Operator speaks:** `"log k6mg 10368.1"` *(or `"log 10368.100 k6mg"`)*
* **Compass terminal echoes:**
  ```text
  [QSO LOGGED #17] K6MG | 10 GHz (10368.100 MHz) | 21:36z | CM87wa -> CM88qk (118.2 km @ 342 deg)
  > 
  ```

---

### 2.2 Error Recovery & Deleting Entries

Because contacts are logged immediately, field error correction is simple:

1. **Quick Undo (`log undo`):**
   * If an accidental contact was logged:
     ```text
     > log undo
     [QSO LOG] Removed last entry #17 (K6MG on 10 GHz). Log contains 16 contacts.
     ```
2. **Delete by Number (`log delete <#>` or `log del <#>`):**
   * If a previous entry needs to be removed:
     ```text
     > log delete 3
     [QSO LOG] Deleted QSO #3 (W6OP/R on 10 GHz). Entries re-indexed (15 contacts remain).
     ```

---

### 2.3 Partial Input & Fallback Prompting

If the operator issues an incomplete command (e.g. just `"log"`):
* **Operator speaks:** `"log"`
* **Compass replies:**
  ```text
  [QSO] Callsign? (e.g. 'K6MG' or 'Kilo Six Mike Golf', or 'cancel')
  > 
  ```
* **Operator speaks:** `"k6mg"`
* **Compass immediately completes and saves:**
  ```text
  [QSO LOGGED #18] K6MG | 10 GHz [Default] | 21:38z | CM87wa -> CM88qk (118.2 km @ 342 deg)
  > 
  ```
* **Hotword "cancel":** Saying `"cancel"` at any prompt immediately aborts without logging.

---

### 2.4 Remote Target Setting (`target <gridsquare>`)

Microwave operators regularly coordinate a schedule with a distant station located in a new grid square. Rather than reaching up to turn the rotary dial or tap the touch screen, the operator can update the target grid directly via voice dictation:

* **Operator speaks:** `"target CM88qk"` *(or NATO phonetics: `"target charlie mike 88 quebec kilo"`)*
* **Compass terminal echoes:**
  ```text
  [TARGET] Target set to CM88qk (Bearing: 342.3 deg, Distance: 118.2 km)
  > 
  ```
* **Display Behavior:**
  * Immediately updates Row 2 on the round LCD to `CM88qk`.
  * Recalculates bearing and points the dual-antenna compass needle toward the new station.
  * Updates geodesic distance and ARRL contest distance.
  * No confirmation word required — executes immediately.

---

## 3. On-Display QSO Confirmation Pop-Up Card

To allow the operator to verify logging while looking at the mast/tripod display without glancing down at their phone screen, the firmware displays a temporary **QSO Card Overlay** on the 480×480 round screen:

```
┌─────────────────────────────────────────────────────────┐
│                 480 x 480 Round Display                 │
│                                                         │
│               ┌─────────────────────────┐               │
│               │     *** QSO LOGGED ***  │               │
│               │                         │               │
│               │        K6MG / R         │  (Large Text) │
│               │                         │               │
│               │   10 GHz (10368.100)    │               │
│               │   CM88qk  -  118.2 km   │               │
│               │   Time: 21:34 UTC       │               │
│               │                         │               │
│               │       [  OK  ]          │  (Auto 10s)   │
│               └─────────────────────────┘               │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Visual & Interactive Behavior:
1. **Trigger:** Appears immediately upon any successful `log` save.
2. **Layout:** Centered semi-transparent modal card (similar to `CalModal` and `RoverCard`).
3. **Information Shown:**
   * Big high-contrast callsign.
   * Band and frequency.
   * Target grid square and distance in km.
   * Time in UTC.
4. **Auto-Dismiss:** Automatically closes after **10 seconds** of inactivity, returning to normal compass navigation.
5. **Instant Dismiss:** Tapping the `[ OK ]` button, touching anywhere on the screen, or clicking/turning the rotary knob dismisses the card immediately.

---

## 4. Comprehensive Frequency & Band Mapping

Microwave contesting spans from 10 GHz up into the millimeter and sub-millimeter waves, as well as VHF/UHF coordination bands. The parser maps both plain band numbers (without needing `"GHz"`) and exact frequencies:

### 4.1 Primary Microwave Bands (10 GHz & Up Contest)

| Band Identifier (Voice / CLI) | Frequency Range | Nominal Calling Freq | Cabrillo Code | ADIF Band |
| :--- | :--- | :--- | :--- | :--- |
| **`10`**, `10G`, `10 GHz`, `10368` | 10.000 – 10.500 GHz | `10368.100 MHz` | `10G` | `3cm` |
| **`24`**, `24G`, `24 GHz`, `24192` | 24.000 – 24.250 GHz | `24192.100 MHz` | `24G` | `1.2cm` |
| **`47`**, `47G`, `47 GHz`, `47088` | 47.000 – 47.200 GHz | `47088.100 MHz` | `47G` | `6mm` |
| **`76`**, `76G`, `76 GHz`, `76032`, `77.5` | 75.500 – 81.000 GHz | `76032.100 MHz` | `76G` | `4mm` |
| **`122`**, `122G`, `122 GHz`, `122250` | 122.250 – 123.000 GHz | `122250.100 MHz` | `122G` | `2.5mm` |
| **`134`**, `134G`, `134 GHz`, `134928` | 134.000 – 141.000 GHz | `134928.100 MHz` | `134G` | `2mm` |
| **`241`**, `241G`, `241 GHz`, `241056` | 241.000 – 250.000 GHz | `241056.100 MHz` | `241G` | `1mm` |
| **`light`**, `laser` | > 300 GHz | ~474 THz (Red laser) | `LIGHT` | `submm` |

### 4.2 Sub-10 GHz Bands (VHF/UHF & Microwave Sprints)

| Band Identifier (Voice / CLI) | Nominal Calling Freq | Cabrillo Code | ADIF Band |
| :--- | :--- | :--- | :--- |
| **`5.7`**, `5.7G`, `5760` | `5760.100 MHz` | `5.7G` | `6cm` |
| **`3.4`**, `3.4G`, `3456`, `3400` | `3456.100 MHz` | `3.4G` | `9cm` |
| **`2.3`**, `2.4`, `2304`, `2400` | `2304.100 MHz` | `2.3G` | `13cm` |
| **`1.2`**, `1296` | `1296.100 MHz` | `1.2G` | `23cm` |
| **`902`**, `903` | `902.100 MHz` | `902` | `33cm` |
| **`432`** | `432.100 MHz` | `432` | `70cm` |
| **`222`** | `222.100 MHz` | `222` | `1.25m` |
| **`144`**, `2m` | `144.200 MHz` | `144` | `2m` |
| **`50`**, `6m` | `50.100 MHz` | `50` | `6m` |

---

## 5. Phonetics Lexer & Voice Dictation Normalizer

The `VoiceQsoParser` processes input through a flexible normalization pipeline:

1. **Punctuation Stripping:** Commas, colons, question marks, and trailing periods inserted by smartphone keyboards are stripped, while preserving internal decimal points (`10368.1`, `5.7`, `77.5`).
2. **NATO Phonetic Translation:** Words like `"Kilo Six Mike Golf"` collapse into `K6MG`.
3. **Phonetic Grid Parsing for `target`:** `"charlie mike 88 quebec kilo"` collapses into `CM88qk`.
4. **Spoken Numbers & Homophones:** Words like `"zero"` $\rightarrow$ `0`, `"won"`/`"one"` $\rightarrow$ `1`, `"to"`/`"too"`/`"two"` $\rightarrow$ `2`, `"for"`/`"four"` $\rightarrow$ `4`, `"ate"`/`"eight"` $\rightarrow$ `8`.
5. **Suffixes:** `"stroke rover"`, `"slash rover"`, `"rover"` $\rightarrow$ `/R`; `"portable"` $\rightarrow$ `/P`.
6. **Argument Disambiguation Without "GHz":**
   * Any token containing letters and a digit matching callsign structure is treated as a **callsign**.
   * Any standalone number in `{10, 24, 47, 76, 122, 134, 241}` is treated as a **microwave GHz band**.
   * Any number $\ge 50$ with $\ge 3$ digits (e.g. `144`, `432`, `1296`, `10368`) is treated as a **frequency in MHz**.
   * Any token matching Maidenhead locator structure (2 letters, 2 digits, optional 2 or 4 letters) is recognized as a **grid square**.

---

## 6. Persistent Storage & Capacity (`QsoLogManager`)

QSO entries are stored in the ESP32-S3 non-volatile storage (NVS) using `Preferences`.

### 6.1 Packed Struct (`QsoEntry`)
```cpp
#pragma pack(push, 1)
struct QsoEntry {
    uint32_t timestamp;        // GPS UTC epoch timestamp (seconds)
    char     callsign[12];     // Target callsign (e.g. "K6MG/R\0")
    char     band[8];          // Band identifier (e.g. "10 GHz\0")
    uint32_t freq_khz;         // Frequency in kHz (e.g. 10368100), 0 if band-only
    char     my_grid[9];       // Operator Maidenhead grid at QSO time (e.g. "CM87wa\0")
    char     his_grid[9];      // Target Maidenhead grid at QSO time (e.g. "CM88qk\0")
    float    distance_km;      // Geodesic path distance in km
    float    bearing_deg;      // Geodesic true bearing in degrees
};
#pragma pack(pop)
```

* **Size per Entry:** Exactly **54 bytes**.
* **Capacity:** Default 200 QSOs (configurable up to 1000, ~54 KB in NVS flash).
* **NVS Endurance:** Over 100,000 write cycles; negligible wear in field use.

---

## 7. Command Reference & Contest Export

All commands operate identically over **Bluetooth Low Energy (BLE)** and **USB Serial**:

| Command | Examples / Spoken Variants | Function |
| :--- | :--- | :--- |
| `log <call> [band/freq]` | `"log k6mg 10"`, `"log 24 k6mg"`, `"log k6mg 10368.1"` | Immediately logs QSO to NVS and shows pop-up card |
| `log <call>` | `"log k6mg"`, `"log kilo six mike golf"` | Immediately logs QSO with default band (10 GHz) |
| `log undo` | `"log undo"` | Deletes the most recent QSO entry |
| `log delete <#>` | `"log delete 3"`, `"log del 14"` | Deletes specific QSO by index and re-indexes |
| `log clear` | `"log clear"` | Prompts confirmation to wipe log |
| `target <grid>` | `"target CM88qk"`, `"target charlie mike 88 quebec kilo"` | Immediately sets target grid and updates display |
| `log list` (or `log show`) | `"log list"` | Prints formatted ASCII table of all QSOs |
| `log stats` | `"log stats"` | Displays contest totals (QSOs, DX km, bands, unique grids) |
| `log export cabrillo` | `"log export cabrillo"` | Emits standard ARRL 10 GHz Cabrillo 3.0 log |
| `log export adif` | `"log export adif"` | Emits standard ADIF records for desktop loggers |
| `log export csv` | `"log export csv"` | Emits comma-separated values for spreadsheets |
| `log default band <bnd>`| `"log default band 10 GHz"`, `"log default band 24"` | Configures default band in NVS |

---

## 8. Implementation & Phasing Plan

To maintain modular development, thorough review, and clean unit testing, the implementation is designed into **three distinct phases**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ Chat 1: Foundation — Core Data Model, Target Command & Phonetics Parser     │
│ ─────────────────────────────────────────────────────────────────────────── │
│ 1. src/system/qso_entry.h & src/system/qso_log_manager.*                   │
│    - QsoEntry struct, NVS storage engine, delete-by-number, undo, clear     │
│    - Cabrillo 3.0, ADIF, CSV formatters and contest statistics calculator   │
│ 2. src/protocol/voice_qso_parser.*                                         │
│    - NATO phonetics lexer (Alpha-Zulu, spoken digits, homophones)           │
│    - Band disambiguation without requiring "GHz" (10, 24, 47, etc.)         │
│    - Full frequency-to-band table (10 GHz to 241 GHz, laser, and VHF/UHF)   │
│ 3. "target <gridsquare>" command integration in SerialCli                   │
│    - Phonetic grid decoding and immediate AppController grid update         │
│ 4. Comprehensive C++ Host Unit Tests in tests/test_calculations.cpp        │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ Chat 2: Interactive Voice CLI Loop & Display Pop-Up Card UI                │
│ ─────────────────────────────────────────────────────────────────────────── │
│ 1. src/ui/qso_card.* (or DisplayUi pop-up overlay)                         │
│    - 480x480 round display modal card showing call, band, freq, grid, dist  │
│    - 10-second non-blocking auto-dismiss timer                              │
│    - Touch "OK" button & rotary click dismiss handler                       │
│ 2. src/protocol/serial_cli.* integration                                    │
│    - Direct-to-save one-shot logging logic                                  │
│    - Automatic GPS context capture (My Grid, Target Grid, Time, Geodesy)   │
│    - Pop-up trigger notification to DisplayUi via AppController             │
│    - Partial input fallback prompting and hotword "cancel"                  │
│    - "log delete <#>", "log undo", "log list", "log export" handlers        │
│ 3. End-to-end integration tests & simulated voice logging sessions         │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ Chat 3: Final Review, End-to-End Field Validation & Documentation Updates   │
│ ─────────────────────────────────────────────────────────────────────────── │
│ 1. SERIAL.md guide updates with full voice dictation instructions & macros │
│ 2. Real-time BLE UART loop testing with simulated voice transcript inputs   │
│ 3. Clean PR preparation and final verification build                        │
└─────────────────────────────────────────────────────────────────────────────┘
```
