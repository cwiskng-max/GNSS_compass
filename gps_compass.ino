#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <PCF8574.h>
#include <Adafruit_CST8XX.h>
#include <Preferences.h>
#include <math.h>
#include <ctype.h>

#include "src/types.h"
#include "src/geo/geo_math.h"
#include "src/geo/maidenhead.h"
#include "src/protocol/nmea_parser.h"

// I2C to PCF8574
#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39
#define BKL_PIN 6

#define I2C_TOUCH_ADDR 0x15
#define INCA(x) (((x+1)>'Z')?'A':(x+1))
#define DECA(x) (((x-1)<'A')?'Z':(x-1))
#define INCN(x) (((x+1)>'9')?'0':(x+1))
#define DECN(x) (((x-1)<'0')?'9':(x-1))
#define INCG(x) (x=(x>='0' && x<='9')?INCN(x):INCA(x))
#define DECG(x) (x=(x>='0' && x<='9')?DECN(x):DECA(x))

// --- Color Palette Adjustments ---
// Rich golden yellow (RGB565: R=31, G=53, B=0 = 0xFEA0, ~#FFD400)
// Sunlight-optimized: standard yellow (0xFFE0, R=31, G=63, B=0) washes out to near-white under intense sunlight.
// Reducing green slightly to ~84% creates a rich, warm golden-amber hue clearly differentiated from white,
// while keeping full red and 0 blue to maintain maximum contrast and photopic brightness.
#define RICH_YELLOW          0xFEA0

// --- Display Layout (480 x 480) ---
#define CHAR_W(sz)           (6 * (sz))
#define CHAR_H(sz)           (8 * (sz))

// Row 1: Local Maidenhead Grid
#define LOC_SZ               4
#define LOC_X                144
#define LOC_Y                35

// Row 2: Target Grid
#define REMSIZE              8
#define REMSTARTX            80
#define REMSTARTY            95
#define REMSTOPX             (6 * CHAR_W(REMSIZE) + REMSTARTX)
#define REMSTOPY             (2 * CHAR_H(REMSIZE) + REMSTARTY)

// Target Grid Touch Area (covers character row, caret row, and generous padding)
#define TOUCH_X_MIN          50
#define TOUCH_X_MAX          430
#define TOUCH_Y_MIN          70
#define TOUCH_Y_MAX          235

// Row 3: Antenna Heading & Target Bearing
#define BEARSZ               6
#define BEARX                20
#define BEARY                REMSTOPY
#define TGTBEARX             280

// Row 3 Center: Calibration Status Pill & Touch Target
#define CAL_PILL_X           200
#define CAL_PILL_Y           225
#define CAL_PILL_W           80
#define CAL_PILL_H           44
#define CAL_TOUCH_X_MIN      180
#define CAL_TOUCH_X_MAX      300
#define CAL_TOUCH_Y_MIN      210
#define CAL_TOUCH_Y_MAX      280
#define CAL_MODAL_TIMEOUT_MS 15000

// Set to 1 to show ARRL contest distance (Row 6), 0 to hide
#define ENABLE_ARRL_DIST     1

#if ENABLE_ARRL_DIST
// Row 4: UTC Time
#define TIME_SZ              6
#define TIME_X               132
#define TIME_Y               290

// Row 5: Reverse Bearing & Distance
#define REVBEAR_SZ           4
#define REVBEAR_X            90
#define REVBEAR_Y            352

// Row 6: ARRL Contest Distance
#define ARRL_SZ              3
#define ARRL_X               123
#define ARRL_Y               408
#else
// Row 4: UTC Time
#define TIME_SZ              6
#define TIME_X               132
#define TIME_Y               300

// Row 5: Reverse Bearing & Distance
#define REVBEAR_SZ           4
#define REVBEAR_X            90
#define REVBEAR_Y            372
#endif

// Row 7: Firmware Version Footer
#define FW_VERSION           "v0.3-dev"
#define VER_SZ               2
#define VER_Y                444

// Inactivity Auto-Lock Timeout (milliseconds; set to 0 to disable)
#define EDIT_TIMEOUT_MS      5000

#define aton(x) (x-'0')

 // Define the RX and TX pins for Serial 2
#define RXD2 44
#define TXD2 43

#define GPS_BAUD 115200

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

char remote[7] = "ZA00AA";
char mygrid[9] = "JJ00AA00";
bool hasLocation = false;
bool hasTime = false;
int hours = 0;
int minutes = 0;

// --- Calibration & UI State Variables ---
float calibrationOffset = 0.0f;
enum UiMode {
  UI_MODE_NORMAL = 0,
  UI_MODE_CALIBRATING = 1
};
UiMode currentUiMode = UI_MODE_NORMAL;
float proposedCalibrationOffset = 0.0f;
unsigned long calModalOpenedMillis = 0;

#define ENCODER_CLK 42
#define ENCODER_DT 4

volatile int counter=0;
volatile int encState = 0;
volatile int oldState = -1;
volatile bool hasChanged = false;

PCF8574 pcf8574(0x21);

Adafruit_CST8XX tsPanel = Adafruit_CST8XX();

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  16 /* CS */, 2 /* SCK */, 1 /* SDA */,
  40 /* DE */, 7 /* VSYNC */, 15 /* HSYNC */, 41 /* PCLK */,
  46 /* R0 */, 3 /* R1 */, 8 /* R2 */, 18 /* R3 */, 17 /* R4 */,
  14 /* G0 */, 13 /* G1 */, 12 /* G2 */, 11 /* G3 */, 10 /* G4 */, 9 /* G5 */,
  5 /* B0 */, 45 /* B1 */, 48 /* B2 */, 47 /* B3 */, 21 /* B4 */
);

Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
  bus,
  GFX_NOT_DEFINED,  // RST pin (not used, we reset via PCF8574)
  0,                // rotation
  false,            // IPS
  480, 480,         // width, height
  st7701_type5_init_operations,
  sizeof(st7701_type5_init_operations),
  true,       // BGR
  10, 4, 20,  // hsync front porch, pulse width, back porch
  10, 4, 20   // vsync front porch, pulse width, back porch
);



void IRAM_ATTR encoder_irq() {
  encState = digitalRead(ENCODER_CLK);
  if (encState != oldState) {
    counter = (digitalRead(ENCODER_DT) == encState) ? +1 : -1;
    oldState = encState;
    hasChanged = true;
  }
}

void initEncoder() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoder_irq, CHANGE);
}



void initBacklight() {
  pinMode(BKL_PIN, OUTPUT);
  analogWrite(BKL_PIN, 250);
}

void initPins() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pcf8574.pinMode(P0, OUTPUT);        //tp RST
  pcf8574.pinMode(P2, OUTPUT);        //tp INT
  pcf8574.pinMode(P3, OUTPUT);        //lcd power
  pcf8574.pinMode(P4, OUTPUT);        //lcd reset
  pcf8574.pinMode(P5, INPUT_PULLUP);  //encoder SW

  if (!pcf8574.begin()) {
    Serial.println("Can't init pcf8574");
  }

  // LCD
  pcf8574.digitalWrite(P3, HIGH);
  delay(100);
  pcf8574.digitalWrite(P4, HIGH);
  delay(100);
  pcf8574.digitalWrite(P4, LOW);
  delay(120);
  pcf8574.digitalWrite(P4, HIGH);
  delay(120);

  // Touchpad
  pcf8574.digitalWrite(P0, HIGH);
  delay(100);
  pcf8574.digitalWrite(P0, LOW);
  delay(120);
  pcf8574.digitalWrite(P0, HIGH);
  delay(120);
  pcf8574.digitalWrite(P2, HIGH);
  delay(120);

  if (!tsPanel.begin(&Wire, I2C_TOUCH_ADDR)) {
    Serial.println("No touchscreen found");
  }
  Serial.println("reset done");
}


void initLCD() {
  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE, BLACK);

}

int remotec = 0;
bool isEditing = true;
unsigned long lastEditActivity = 0;

void printLocalGrid() {
  gfx->setTextSize(LOC_SZ);
  gfx->setCursor(LOC_X, LOC_Y);
  gfx->print(mygrid);
}

void printrem() {

  gfx->setTextSize(REMSIZE);

  for (int i = 0; i < 6; i++) {
    if (isEditing && i == remotec) {
      gfx->setTextColor(RICH_YELLOW, BLACK);
    } else {
      gfx->setTextColor(WHITE, BLACK);
    }
    gfx->setCursor(REMSTARTX + i * CHAR_W(REMSIZE), REMSTARTY);
    gfx->print(remote[i]);
  }

  gfx->setTextColor(RICH_YELLOW, BLACK);
  gfx->setCursor(REMSTARTX, REMSTARTY + CHAR_H(REMSIZE));
  gfx->print("      ");
  if (isEditing) {
    gfx->setCursor(REMSTARTX + remotec * CHAR_W(REMSIZE), REMSTARTY + CHAR_H(REMSIZE));
    gfx->print("^");
  }

  gfx->setTextColor(WHITE, BLACK);

}








// --- Bearing Quality & State Machine ---
#define HEADING_TIMEOUT_MS   1500

BearingFilter bearingFilter;

bool hasHeading = false;
BearingQuality currentBearingQuality = BEARING_NONE;
unsigned long lastHeadingUpdate = 0;
bool rovdist = false;
double mylongitude = 0.0, mylatitude = 0.0;
double remlongitude = 0.0, remlatitude = 0.0;
double distance = 0.0, bearing = 0.0, revbearing = 0.0;

// GPS Stream Diagnostic Counters
unsigned long totalGpsBytesReceived = 0;
unsigned long lastGpsByteMillis = 0;
unsigned int gllPacketCount = 0;
unsigned long lastGllMillis = 0;
unsigned int ksxtPacketCount = 0;
unsigned long lastKsxtMillis = 0;
unsigned int hdtPacketCount = 0;
unsigned long lastHdtMillis = 0;

// GPS Simulation Mode State
bool simModeActive = false;
unsigned long lastSimInjectionMillis = 0;
unsigned int simPacketCount = 0;

void loadCalibration() {
  Preferences prefs;
  prefs.begin("compass", true);
  calibrationOffset = prefs.getFloat("cal_offset", 0.0f);
  prefs.end();
  Serial.printf("[NVS] Loaded antenna calibration offset: %+5.1f deg\r\n", calibrationOffset);
}

void saveCalibration(float offset) {
  offset = normalizeOffset(offset);
  if (fabs(offset - calibrationOffset) > 0.01f) {
    calibrationOffset = offset;
    Preferences prefs;
    prefs.begin("compass", false);
    prefs.putFloat("cal_offset", calibrationOffset);
    prefs.end();
    Serial.printf("[NVS] Saved antenna calibration offset: %+5.1f deg\r\n", calibrationOffset);
  }
}

void drawCalibrationPill() {
  if (currentUiMode == UI_MODE_CALIBRATING) return;

  // Clear pill region
  gfx->fillRect(CAL_PILL_X - 2, CAL_PILL_Y - 2, CAL_PILL_W + 4, CAL_PILL_H + 4, BLACK);

  uint16_t borderColor = 0x39E7; // Dark gray border
  uint16_t textColor = 0x7BEF;   // Dim gray text

  if (fabs(calibrationOffset) >= 0.05f) {
    borderColor = 0x07FF;        // Cyan border when active
    textColor = 0x07FF;          // Cyan text when active
  }

  // Draw rounded pill frame
  gfx->drawRoundRect(CAL_PILL_X, CAL_PILL_Y, CAL_PILL_W, CAL_PILL_H, 6, borderColor);

  // Line 1: "CAL"
  gfx->setTextSize(2);
  gfx->setTextColor(textColor, BLACK);
  gfx->setCursor(CAL_PILL_X + 22, CAL_PILL_Y + 4);
  gfx->print("CAL");

  // Line 2: "OFF" or "+4.2"
  char valStr[12];
  if (fabs(calibrationOffset) < 0.05f) {
    strcpy(valStr, "OFF");
  } else {
    sprintf(valStr, "%+4.1f", calibrationOffset);
  }
  int txtW = strlen(valStr) * CHAR_W(2);
  gfx->setCursor(CAL_PILL_X + (CAL_PILL_W - txtW) / 2, CAL_PILL_Y + 24);
  gfx->print(valStr);
  gfx->setTextColor(WHITE, BLACK);
}

void drawProposedOffsetRow() {
  gfx->fillRect(50, 198, 380, 26, 0x10A2);
  gfx->setTextSize(2);
  gfx->setTextColor(RICH_YELLOW, 0x10A2);
  char buf[36];
  sprintf(buf, "Proposed Offset: %+5.1f", proposedCalibrationOffset);
  gfx->setCursor(55, 202);
  gfx->print(buf);
  gfx->setTextColor(WHITE, BLACK);
}

void drawRawHeadingInModal() {
  if (currentUiMode != UI_MODE_CALIBRATING) return;
  gfx->fillRect(55, 146, 370, 22, 0x10A2);
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, 0x10A2);
  gfx->setCursor(55, 148);
  char buf[36];
  if (hasHeading) {
    double rawHdg = bearingFilter.getRawHeading();
    sprintf(buf, "Raw GNSS Hdg   : %5.1f", rawHdg);
  } else {
    sprintf(buf, "Raw GNSS Hdg   : ---.-");
  }
  gfx->print(buf);
  gfx->setTextColor(WHITE, BLACK);
}

void drawSaveButton(bool isHolding, float progress) {
  uint16_t bg = isHolding ? 0x0540 : 0x0320;
  gfx->fillRoundRect(55, 284, 370, 48, 8, bg);
  gfx->drawRoundRect(55, 284, 370, 48, 8, GREEN);

  if (isHolding && progress > 0.0f) {
    int fillW = (int)(366.0f * progress);
    if (fillW > 366) fillW = 366;
    gfx->fillRoundRect(57, 286, fillW, 44, 6, 0x07E0);
  }

  const char* saveText = isHolding ? "HOLDING TO SAVE..." : "HOLD TO SAVE OFFSET";
  int stw = strlen(saveText) * CHAR_W(2);
  gfx->setTextColor(WHITE, (isHolding && progress > 0.5f) ? 0x07E0 : bg);
  gfx->setTextSize(2);
  gfx->setCursor((480 - stw) / 2, 300);
  gfx->print(saveText);
  gfx->setTextColor(WHITE, BLACK);
}

void drawSnapButton(bool highlighted) {
  if (currentUiMode != UI_MODE_CALIBRATING) return;

  uint16_t snapBg, snapBorder, snapFg;
  if (!hasHeading) {
    snapBg = 0x2104;      // Disabled dark gray
    snapBorder = 0x4208;  // Dim outline
    snapFg = 0x7BEF;      // Dim text
  } else if (highlighted) {
    snapBg = CYAN;        // Vibrant electric cyan highlight flash
    snapBorder = WHITE;
    snapFg = BLACK;
  } else {
    snapBg = 0x1B19;      // Vibrant Cobalt / Royal Blue (#1961CD)
    snapBorder = 0x3DFF;  // Electric Dodger Blue / Sky border
    snapFg = WHITE;
  }

  gfx->fillRoundRect(55, 234, 180, 40, 8, snapBg);
  gfx->drawRoundRect(55, 234, 180, 40, 8, snapBorder);
  const char* snapText = "SNAP TARGET";
  int stw = strlen(snapText) * CHAR_W(2);
  gfx->setTextSize(2);
  gfx->setTextColor(snapFg, snapBg);
  gfx->setCursor(55 + (180 - stw) / 2, 246);
  gfx->print(snapText);
  gfx->setTextColor(WHITE, BLACK);
}

void drawResetButton(bool highlighted) {
  if (currentUiMode != UI_MODE_CALIBRATING) return;

  uint16_t rstBg = highlighted ? RED : 0x5000;
  uint16_t rstBorder = highlighted ? WHITE : RED;
  uint16_t rstFg = WHITE;

  gfx->fillRoundRect(245, 234, 180, 40, 8, rstBg);
  gfx->drawRoundRect(245, 234, 180, 40, 8, rstBorder);
  const char* rstText = "RESET (0.0)";
  int rtw = strlen(rstText) * CHAR_W(2);
  gfx->setTextSize(2);
  gfx->setTextColor(rstFg, rstBg);
  gfx->setCursor(245 + (180 - rtw) / 2, 246);
  gfx->print(rstText);
  gfx->setTextColor(WHITE, BLACK);
}

void drawCancelButton(bool highlighted) {
  if (currentUiMode != UI_MODE_CALIBRATING) return;

  uint16_t canBg = highlighted ? WHITE : 0x2945;
  uint16_t canBorder = WHITE;
  uint16_t canFg = highlighted ? BLACK : WHITE;

  gfx->fillRoundRect(140, 342, 200, 36, 8, canBg);
  gfx->drawRoundRect(140, 342, 200, 36, 8, canBorder);
  const char* canText = "CANCEL";
  int ctw = strlen(canText) * CHAR_W(2);
  gfx->setTextSize(2);
  gfx->setTextColor(canFg, canBg);
  gfx->setCursor(140 + (200 - ctw) / 2, 352);
  gfx->print(canText);
  gfx->setTextColor(WHITE, BLACK);
}

void drawCalibrationModal() {
  // Semi-modal dark card in center
  gfx->fillRoundRect(35, 75, 410, 335, 12, 0x10A2); // Dark slate navy
  gfx->drawRoundRect(35, 75, 410, 335, 12, 0x07FF); // Cyan border
  gfx->drawRoundRect(36, 76, 408, 333, 11, 0x07FF); // Double border

  // Title: "ANTENNA CALIBRATION"
  gfx->setTextSize(2);
  gfx->setTextColor(CYAN, 0x10A2);
  const char* title = "ANTENNA CALIBRATION";
  int tw = strlen(title) * CHAR_W(2);
  gfx->setCursor((480 - tw) / 2, 88);
  gfx->print(title);

  // Separator line
  gfx->drawFastHLine(55, 112, 370, 0x39E7);

  // Target Bearing
  char buf[36];
  gfx->setTextColor(WHITE, 0x10A2);
  gfx->setCursor(55, 124);
  sprintf(buf, "Target Bearing : %5.1f", bearing);
  gfx->print(buf);

  // Raw GNSS Heading
  drawRawHeadingInModal();

  // Current Stored Offset
  gfx->setCursor(55, 172);
  sprintf(buf, "Current Offset : %+5.1f", calibrationOffset);
  gfx->print(buf);

  // Proposed Offset (with fine-tuning knob)
  drawProposedOffsetRow();

  // Button 1: [ SNAP TARGET ] (tap only, updates display without saving)
  // X: 55 to 235 (Width 180), Y: 234 to 274 (Height 40)
  drawSnapButton(false);

  // Button 2: [ RESET (0.0) ] (tap only, resets display without saving)
  // X: 245 to 425 (Width 180), Y: 234 to 274 (Height 40)
  drawResetButton(false);

  // Button 3: [ HOLD TO SAVE OFFSET ] (requires 1.5s long-press)
  // X: 55 to 425 (Width 370), Y: 284 to 332 (Height 48)
  drawSaveButton(false, 0.0f);

  // Button 4: [ CANCEL ] (tap is OK)
  // X: 140 to 340 (Width 200), Y: 342 to 378 (Height 36)
  drawCancelButton(false);

  // Instruction footer
  gfx->setTextColor(0x7BEF, 0x10A2);
  const char* hint = "(Turn knob to nudge +-0.1)";
  int htw = strlen(hint) * CHAR_W(2);
  gfx->setCursor((480 - htw) / 2, 386);
  gfx->print(hint);

  gfx->setTextColor(WHITE, BLACK);
}

void redrawMainScreen() {
  gfx->fillScreen(BLACK);
  printLocalGrid();
  printrem();
  updateBearings();

  char tbuf[10];
  sprintf(tbuf, "%02d:%02dZ", hours, minutes);
  gfx->setTextSize(TIME_SZ);
  gfx->setCursor(TIME_X, TIME_Y);
  gfx->setTextColor(WHITE, BLACK);
  gfx->print(hasTime ? tbuf : "--:--Z");

  showFooterText(FW_VERSION, 0x7BEF);
}

void handleCalSerialCommand(const String &cmd) {
  String arg = cmd.substring(3);
  arg.trim();
  if (arg.length() == 0 || arg.equalsIgnoreCase("status") || arg.equalsIgnoreCase("show")) {
    Serial.println();
    Serial.println(F("=== Antenna Calibration Status ==="));
    Serial.printf("  Stored Offset : %+5.1f deg\r\n", calibrationOffset);
    Serial.printf("  Target Bearing: %5.1f deg\r\n", bearing);
    if (hasHeading) {
      double rawHdg = bearingFilter.getRawHeading();
      Serial.printf("  Raw GNSS Hdg  : %5.1f deg\r\n", rawHdg);
      Serial.printf("  Calibrated Hdg: %5.1f deg\r\n", bearingFilter.getCalibratedHeading(calibrationOffset));
    } else {
      Serial.println(F("  Raw GNSS Hdg  : ---.- (No heading fix)"));
    }
    Serial.println(F("=================================="));
    Serial.println();
  } else if (arg.equalsIgnoreCase("zero") || arg.equalsIgnoreCase("reset") || arg.equals("0")) {
    saveCalibration(0.0f);
    updateBearings();
    Serial.println(F("[CAL] Offset reset to 0.0 deg (Cleared)."));
  } else if (arg.equalsIgnoreCase("snap")) {
    if (!hasHeading) {
      Serial.println(F("[CAL ERROR] Cannot snap: No valid GNSS heading fix."));
    } else {
      double rawHdg = bearingFilter.getRawHeading();
      float newOffset = normalizeOffset((float)(bearing - rawHdg));
      saveCalibration(newOffset);
      updateBearings();
      Serial.printf("[CAL] Snapped! New offset: %+5.1f deg\r\n", newOffset);
    }
  } else if (arg.startsWith("set ") || arg.startsWith("SET ")) {
    float val = arg.substring(4).toFloat();
    val = normalizeOffset(val);
    saveCalibration(val);
    updateBearings();
    Serial.printf("[CAL] Offset set to %+5.1f deg\r\n", val);
  } else {
    float val = arg.toFloat();
    val = normalizeOffset(val);
    saveCalibration(val);
    updateBearings();
    Serial.printf("[CAL] Offset set to %+5.1f deg\r\n", val);
  }
}

void drawCurrentBearing() {
  if (currentUiMode == UI_MODE_CALIBRATING) {
    drawRawHeadingInModal();
    return;
  }

  uint16_t color = WHITE;
  if (currentBearingQuality == BEARING_FLOAT) {
    color = RICH_YELLOW;
  } else if (currentBearingQuality == BEARING_LOST) {
    color = RED;
  }

  gfx->setTextSize(BEARSZ);
  gfx->setCursor(BEARX, BEARY);
  gfx->setTextColor(color, BLACK);

  if (!hasHeading) {
    gfx->print("---.-");
  } else {
    char str[10];
    sprintf(str, "%5.1f", bearingFilter.getCalibratedHeading(calibrationOffset));
    gfx->print(str);
  }
  gfx->setTextColor(WHITE, BLACK);
}

void updateBearings() {
  gridToGeoPoint(remote, &remlatitude, &remlongitude);
  calculateWgs84(mylatitude, mylongitude, remlatitude, remlongitude, distance, bearing);
  calculateWgs84(remlatitude, remlongitude, mylatitude, mylongitude, distance, revbearing);

  if (currentUiMode == UI_MODE_CALIBRATING) {
    return;
  }

  char buf[20];
  int rdist = (int)floor(distance / 1000.0);
  sprintf(&buf[0], "%5.1f %4i k  ", revbearing, rdist);
  if ((rdist < 16) && rovdist) {
    gfx->setTextColor(RED, WHITE);
  }
  gfx->setTextSize(REVBEAR_SZ);
  gfx->setCursor(REVBEAR_X, REVBEAR_Y);
  gfx->print(buf);
  gfx->setTextColor(WHITE, BLACK);

  drawCurrentBearing();
  drawCalibrationPill();

  sprintf(&buf[0], "%5.1f", bearing);
  gfx->setTextSize(BEARSZ);
  gfx->setCursor(TGTBEARX, BEARY);
  gfx->print(buf);

#if ENABLE_ARRL_DIST
  char arrl_str[24];
  double arrl_dist = calculateArrlContestDistance(mygrid, remote);
  sprintf(arrl_str, "ARRL %6.1f k  ", arrl_dist);
  gfx->setTextSize(ARRL_SZ);
  gfx->setCursor(ARRL_X, ARRL_Y);
  gfx->print(arrl_str);
#endif
}

void setup() {

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.begin (115200);
  Serial.print("GNSS Compass ");
  Serial.println(FW_VERSION);
  Serial.println(F("Commands: 'status' (GPS stream diagnosis), 'test' (visual demo), 'sim' (simulation)"));

  bearingFilter.reset();

  initPins();
  initBacklight();
  initLCD();
  initEncoder();
  loadCalibration();

  geoPointToGrid(mylatitude, mylongitude, mygrid);
  gfx->setTextSize(LOC_SZ);
  gfx->setCursor(LOC_X, LOC_Y);
  gfx->print(mygrid);

  lastEditActivity = millis();
  printrem();
  updateBearings();

  gfx->setTextSize(TIME_SZ);
  gfx->setCursor(TIME_X, TIME_Y);
  gfx->print("--:--Z");

  // Display firmware version footer
  showFooterText(FW_VERSION, 0x7BEF);
}

// Display centered text in footer row (used for version and test step titles)
void showFooterText(const char *msg, uint16_t color) {
  gfx->fillRect(0, VER_Y, 480, CHAR_H(VER_SZ) + 4, BLACK);
  int w = strlen(msg) * CHAR_W(VER_SZ);
  gfx->setTextSize(VER_SZ);
  gfx->setTextColor(color, BLACK);
  gfx->setCursor((480 - w) / 2, VER_Y);
  gfx->print(msg);
  gfx->setTextColor(WHITE, BLACK);
}

// Display centered rounded banner card across the screen (used for test results and diagnostics)
void showStatusCard(const char *msg, uint16_t bgColor, uint16_t fgColor) {
  int w = strlen(msg) * CHAR_W(2);
  gfx->fillRoundRect(40, 215, 400, 52, 8, bgColor);
  gfx->setTextColor(fgColor, bgColor);
  gfx->setTextSize(2);
  gfx->setCursor((480 - w) / 2, 233);
  gfx->print(msg);
  gfx->setTextColor(WHITE, BLACK);
}

// Full GNSS Receiver & Data Stream diagnostic report
void printGpsStatusReport() {
  unsigned long now = millis();
  GpsLinkVerdict v = evaluateGpsStream(
    now, totalGpsBytesReceived, lastGpsByteMillis,
    gllPacketCount, ksxtPacketCount, hdtPacketCount,
    hasLocation, hasHeading, currentBearingQuality
  );

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("     GNSS Receiver Stream Diagnosis     "));
  Serial.println(F("========================================"));
  if (simModeActive) {
    Serial.println(F("Operation Mode: SIMULATION ACTIVE (Hardware Serial2 suppressed)"));
    Serial.printf("Sim Injections: %u pkts (last %lu ms ago)\r\n", simPacketCount, millis() - lastSimInjectionMillis);
  }

  if (v == GPS_LINK_DISCONNECTED) {
    Serial.println(F("Physical Link:  DISCONNECTED / NO DATA"));
    if (totalGpsBytesReceived == 0) {
      Serial.println(F("Details:        0 bytes received on Serial2 since boot."));
    } else {
      Serial.print(F("Details:        Last byte received "));
      Serial.print((now - lastGpsByteMillis) / 1000.0, 1);
      Serial.println(F(" seconds ago."));
    }
    Serial.println(F("Verdict:        [FAIL] NO GPS DATA STREAM DETECTED"));
    Serial.println(F("Troubleshooting:"));
    Serial.println(F("  1. Verify GX12 3-pin cable is securely connected."));
    Serial.println(F("  2. Check 5V / 12V power to the UM982 receiver enclosure."));
    Serial.println(F("  3. Verify RX pin (GPIO 44) wiring to UM982 TX."));
  } else if (v == GPS_LINK_NOISY_CORRUPT) {
    Serial.println(F("Physical Link:  BYTES RECEIVED, BUT NO VALID NMEA/KSXT"));
    Serial.print(F("Total Bytes:    "));
    Serial.println(totalGpsBytesReceived);
    Serial.println(F("Verdict:        [FAIL] FRAMING / BAUD RATE MISMATCH"));
    Serial.println(F("Troubleshooting:"));
    Serial.println(F("  1. Verify receiver COM1 baud rate is set to 115200 (8N1)."));
    Serial.println(F("  2. Check for loose connection or electrical noise on signal line."));
  } else {
    Serial.println(F("Physical Link:  CONNECTED (Hardware Serial Link Active)"));
    Serial.print(F("Total Bytes:    "));
    Serial.print(totalGpsBytesReceived);
    Serial.print(F(" (last byte "));
    Serial.print(now - lastGpsByteMillis);
    Serial.println(F(" ms ago)"));

    Serial.println();
    Serial.println(F("Message Streams:"));
    Serial.print(F("  - $GNGLL (Position):   "));
    Serial.print(gllPacketCount);
    Serial.print(F(" pkts (last "));
    if (gllPacketCount > 0) {
      Serial.print((now - lastGllMillis) / 1000.0, 1);
      Serial.println(F("s ago)"));
    } else {
      Serial.println(F("none)"));
    }

    Serial.print(F("  - $KSXT (Dual-Ant):    "));
    Serial.print(ksxtPacketCount);
    Serial.print(F(" pkts (last "));
    if (ksxtPacketCount > 0) {
      Serial.print(now - lastKsxtMillis);
      Serial.println(F(" ms ago)"));
    } else {
      Serial.println(F("none)"));
    }

    Serial.print(F("  - $GNHDT/$THS:         "));
    Serial.print(hdtPacketCount);
    Serial.println(F(" pkts"));

    Serial.println(F("\nFix Status:"));
    Serial.print(F("  - Time:                "));
    if (hasTime) {
      char tbuf[16];
      sprintf(tbuf, "%02d:%02d UTC", hours, minutes);
      Serial.println(tbuf);
    } else {
      Serial.println(F("SEARCHING (Syncing...)"));
    }

    Serial.print(F("  - Position:            "));
    if (hasLocation) {
      Serial.print(F("LOCKED (Grid: "));
      Serial.print(mygrid);
      Serial.println(F(")"));
    } else {
      Serial.println(F("SEARCHING (No fix yet)"));
    }

    Serial.print(F("  - Dual-Antenna Heading:"));
    if (hasHeading) {
      if (currentBearingQuality == BEARING_FIXED) {
        Serial.println(F("RTK FIXED (Pure White - High Precision)"));
      } else if (currentBearingQuality == BEARING_FLOAT) {
        Serial.println(F("RTK FLOAT (Bright Yellow - Degraded Sky View)"));
      } else {
        Serial.println(F("LOST / STALE (Bright Red - Tracking Dropped)"));
      }
    } else {
      Serial.println(F("ACQUIRING (Waiting for initial heading lock)"));
    }

    Serial.print(F("\nOverall Verdict:        "));
    if (v == GPS_LINK_OK_FIXED) {
      Serial.println(F("[PASS] FULLY OPERATIONAL (RTK Fixed)"));
    } else if (v == GPS_LINK_HEADING_FLOAT) {
      Serial.println(F("[PASS - WARN] OPERATIONAL (RTK Float - Degraded reception)"));
    } else if (v == GPS_LINK_POSITION_ONLY) {
      Serial.println(F("[WAIT] POSITION LOCKED, WAITING FOR HEADING BASELINE"));
    } else {
      Serial.println(F("[WAIT] RECEIVER TALKING, ACQUIRING SATELLITES"));
    }
  }
  Serial.println(F("========================================\n"));
}

// On-demand visual display self-test demo
void runVisualSelfTest() {
  Serial.println(F("\n=== Starting Visual Display Self-Test Demo ==="));

  // Save current live state
  char saved_remote[7];
  char saved_mygrid[9];
  memcpy(saved_remote, remote, sizeof(saved_remote));
  memcpy(saved_mygrid, mygrid, sizeof(saved_mygrid));
  double saved_mylat = mylatitude, saved_mylon = mylongitude;
  bool saved_hasHeading = hasHeading;
  bool saved_rovdist = rovdist;
  bool saved_isEditing = isEditing;
  BearingQuality saved_quality = currentBearingQuality;
  unsigned long saved_lastHeading = lastHeadingUpdate;
  float saved_calibrationOffset = calibrationOffset;
  UiMode saved_uiMode = currentUiMode;
  isEditing = false;

  // Test 1: Same Grid (ARRL 1.0 km minimum rule)
  Serial.println(F("[Test 1/6] Same Grid (ARRL 1.0 km rule)"));
  showFooterText("TEST 1: SAME GRID", CYAN);
  strcpy(mygrid, "CM87wa00");
  strcpy(remote, "CM87wa");
  mylatitude = 38.354;
  mylongitude = -119.458;
  hasHeading = true;
  currentBearingQuality = BEARING_FIXED;
  bearingFilter.seed(180.0f);
  rovdist = false;
  printLocalGrid();
  printrem();
  updateBearings();
  delay(3000);

  // Test 2: Rover Transit Alert (< 16 km, highlighted RED on WHITE)
  Serial.println(F("[Test 2/6] Rover Transit Alert (< 16 km, highlighted RED on WHITE)"));
  showFooterText("TEST 2: ROVER < 16K", RICH_YELLOW);
  strcpy(remote, "CM87wb");
  currentBearingQuality = BEARING_FIXED;
  rovdist = true;
  printrem();
  updateBearings();
  delay(3000);

  // Test 3: Distance Under 1,000 km (~600 km)
  Serial.println(F("[Test 3/6] Under 1,000 km formatting (~600 km)"));
  showFooterText("TEST 3: DIST < 1000K", CYAN);
  strcpy(remote, "DN13ab");
  currentBearingQuality = BEARING_FIXED;
  rovdist = false;
  printrem();
  updateBearings();
  delay(3000);

  // Test 4: Distance Over 1,000 km (~2,500 km)
  Serial.println(F("[Test 4/6] Over 1,000 km formatting (~2,500 km)"));
  showFooterText("TEST 4: DIST > 1000K", CYAN);
  strcpy(remote, "EL29ja");
  currentBearingQuality = BEARING_FIXED;
  printrem();
  updateBearings();
  delay(3000);

  // Test 5: Extreme Distance (> 10,000 km) and Recovery to 1.0 km
  Serial.println(F("[Test 5/6] Extreme distance (> 10,000 km)"));
  showFooterText("TEST 5: GHOST RECOVERY", CYAN);
  strcpy(remote, "ZA00AA");
  printrem();
  updateBearings();
  delay(2000);

  Serial.println(F("  -> Dropping back to 1.0 km (verifying no ghost characters)"));
  strcpy(remote, "CM87wa");
  printrem();
  updateBearings();
  delay(2000);

  // Test 6: Bearing Color States & Live GPS Receiver Link Diagnostic
  Serial.println(F("[Test 6/6] Bearing Color States & Live GPS Stream Diagnostic"));
  Serial.println(F("  -> Bearing Color: Pure White (RTK Fixed)"));
  showFooterText("TEST 6: BEARING WHITE", WHITE);
  currentBearingQuality = BEARING_FIXED;
  drawCurrentBearing();
  delay(1000);

  Serial.println(F("  -> Bearing Color: Bright Yellow (RTK Float / Degraded)"));
  showFooterText("TEST 6: BEARING YELLOW", RICH_YELLOW);
  currentBearingQuality = BEARING_FLOAT;
  drawCurrentBearing();
  delay(1000);

  Serial.println(F("  -> Bearing Color: Bright Red (Lost / Stale)"));
  showFooterText("TEST 6: BEARING RED", RED);
  currentBearingQuality = BEARING_LOST;
  drawCurrentBearing();
  delay(1000);

  // Restore bearing to fixed before showing live GPS stream ruling
  currentBearingQuality = BEARING_FIXED;
  drawCurrentBearing();
  showFooterText("TEST 6: GPS STREAM", CYAN);
  printGpsStatusReport();

  GpsLinkVerdict liveVerdict = evaluateGpsStream(
    millis(), totalGpsBytesReceived, lastGpsByteMillis,
    gllPacketCount, ksxtPacketCount, hdtPacketCount,
    hasLocation, hasHeading, currentBearingQuality
  );

  if (liveVerdict == GPS_LINK_OK_FIXED) {
    showStatusCard("GPS: RTK FIXED (OK)", GREEN, BLACK);
  } else if (liveVerdict == GPS_LINK_HEADING_FLOAT) {
    showStatusCard("GPS: RTK FLOAT (WARN)", RICH_YELLOW, BLACK);
  } else if (liveVerdict == GPS_LINK_ACQUIRING || liveVerdict == GPS_LINK_POSITION_ONLY) {
    showStatusCard("GPS: ACQUIRING (WAIT)", RICH_YELLOW, BLACK);
  } else {
    showStatusCard("GPS: NO DATA / CABLE", RED, WHITE);
  }
  delay(2500);
  gfx->fillRect(40, 215, 400, 52, BLACK);
  gfx->setTextColor(WHITE, BLACK);

  // Test 7: Antenna Calibration Status & Modal Demo
  Serial.println(F("[Test 7/7] Antenna Calibration Pill & Modal Display Demo"));
  Serial.println(F("  -> Testing Calibration Pill Display (Uncalibrated: OFF)"));
  showFooterText("TEST 7: CAL PILL OFF", CYAN);
  calibrationOffset = 0.0f;
  drawCalibrationPill();
  drawCurrentBearing();
  delay(1500);

  Serial.println(F("  -> Testing Calibration Pill Display (Calibrated: +4.2 deg)"));
  showFooterText("TEST 7: CAL PILL +4.2", GREEN);
  calibrationOffset = 4.2f;
  drawCalibrationPill();
  drawCurrentBearing();
  delay(1500);

  Serial.println(F("  -> Demonstrating Calibration Modal Dialog"));
  showFooterText("TEST 7: CAL MODAL", RICH_YELLOW);
  proposedCalibrationOffset = 4.2f;
  currentUiMode = UI_MODE_CALIBRATING;
  drawCalibrationModal();
  delay(1200);
  drawSnapButton(true);
  delay(300);
  drawSnapButton(false);
  delay(1000);

  // Display PASS/FAIL Result Banner
  bool all_pass = true;
  if (fabs(calculateArrlContestDistance("CM87wa", "CM87wa") - 1.0) > 0.001) all_pass = false;
  double lat_wb, lon_wb, rover_dist, rover_bear;
  gridToGeoPoint("CM87wb", &lat_wb, &lon_wb);
  calculateWgs84(38.354, -119.458, lat_wb, lon_wb, rover_dist, rover_bear);
  if ((int)floor(rover_dist / 1000.0) >= 16) all_pass = false;
  if (fabs(normalizeOffset(5.0f - 355.0f) - 10.0f) > 0.01f) all_pass = false;

  if (all_pass) {
    Serial.println(F("Result: PASS"));
    gfx->fillRoundRect(50, 215, 380, 52, 8, GREEN);
    gfx->setTextColor(BLACK, GREEN);
    gfx->setTextSize(3);
    gfx->setCursor(96, 230);
    gfx->print("SELF-TEST PASSED");
  } else {
    Serial.println(F("Result: FAIL"));
    gfx->fillRoundRect(50, 215, 380, 52, 8, RED);
    gfx->setTextColor(WHITE, RED);
    gfx->setTextSize(3);
    gfx->setCursor(96, 230);
    gfx->print("SELF-TEST FAILED");
  }
  delay(2500);
  gfx->fillRect(50, 215, 380, 52, BLACK);
  gfx->setTextColor(WHITE, BLACK);

  // Restore live state and firmware version footer
  memcpy(remote, saved_remote, sizeof(remote));
  memcpy(mygrid, saved_mygrid, sizeof(mygrid));
  mylatitude = saved_mylat;
  mylongitude = saved_mylon;
  hasHeading = saved_hasHeading;
  currentBearingQuality = saved_quality;
  lastHeadingUpdate = saved_lastHeading;
  rovdist = saved_rovdist;
  isEditing = saved_isEditing;
  calibrationOffset = saved_calibrationOffset;
  currentUiMode = saved_uiMode;
  redrawMainScreen();

  Serial.println(F("=== Visual Self-Test Complete (Live state restored) ===\n"));
}

void processGpsSentence(char *sentence) {
  float new_heading = 0.0f;
  BearingQuality new_quality = BEARING_LOST;

  if (parseHeadingSentence(sentence, &new_heading, &new_quality)) {
    if (strncmp(sentence, "$KSXT", 5) == 0) {
      ksxtPacketCount++;
      lastKsxtMillis = millis();
    } else {
      hdtPacketCount++;
      lastHdtMillis = millis();
    }
    hasHeading = true;
    lastHeadingUpdate = millis();
    currentBearingQuality = new_quality;

    bearingFilter.push(new_heading);

    drawCurrentBearing();
  } else if (strncmp(sentence, "$KSXT", 5) == 0 ||
             strncmp(sentence, "$GNTHS", 6) == 0 || strncmp(sentence, "$GPTHS", 6) == 0 ||
             strncmp(sentence, "$GNHDT", 6) == 0 || strncmp(sentence, "$GPHDT", 6) == 0) {
    if (strncmp(sentence, "$KSXT", 5) == 0) {
      ksxtPacketCount++;
      lastKsxtMillis = millis();
    } else {
      hdtPacketCount++;
      lastHdtMillis = millis();
    }
    // Received heading sentence but no valid heading solution (e.g. baseline lost)
    if (hasHeading && currentBearingQuality != BEARING_LOST) {
      currentBearingQuality = BEARING_LOST;
      drawCurrentBearing();
    }
  }

  // Extract Time and Position from $KSXT (5 Hz)
  if (strncmp(sentence, "$KSXT", 5) == 0) {
    int ksxt_h = 0, ksxt_m = 0;
    if (parseKSXTTime(sentence, &ksxt_h, &ksxt_m)) {
      if (!hasTime || ksxt_h != hours || ksxt_m != minutes) {
        hasTime = true;
        hours = ksxt_h;
        minutes = ksxt_m;
        if (currentUiMode != UI_MODE_CALIBRATING) {
          char tbuf[9];
          sprintf(tbuf, "%02d:%02dZ", hours, minutes);
          gfx->setTextSize(TIME_SZ);
          gfx->setCursor(TIME_X, TIME_Y);
          gfx->setTextColor(WHITE, BLACK);
          gfx->print(tbuf);
        }
      }
    }

    double ksxt_lat = 0.0, ksxt_lon = 0.0;
    if (parseKSXTLocation(sentence, &ksxt_lat, &ksxt_lon)) {
      if (!hasLocation || fabs(ksxt_lat - mylatitude) > 0.00001 || fabs(ksxt_lon - mylongitude) > 0.00001) {
        hasLocation = true;
        mylatitude = ksxt_lat;
        mylongitude = ksxt_lon;
        geoPointToGrid(mylatitude, mylongitude, mygrid);

        if (currentUiMode != UI_MODE_CALIBRATING) {
          gfx->setTextSize(LOC_SZ);
          gfx->setCursor(LOC_X, LOC_Y);
          gfx->setTextColor(WHITE, BLACK);
          gfx->print(mygrid);
        }

        updateBearings();
      }
    }
  } else if (strncmp(sentence, "$GNGLL", 6) == 0) {
    gllPacketCount++;
    lastGllMillis = millis();
    char *ptr;
    char buf[9];

    parseLegacyLatLon(&sentence[7], &mylatitude, &mylongitude, &ptr);
    int gll_h = aton(ptr[0]) * 10.0 + aton(ptr[1]);
    int gll_m = aton(ptr[2]) * 10.0 + aton(ptr[3]);

    char *status_comma = strchr(ptr, ',');
    if ((status_comma != NULL && *(status_comma + 1) == 'A') || ptr[10] == 'A') {
      hasLocation = true;
      geoPointToGrid(mylatitude, mylongitude, mygrid);

      if (currentUiMode != UI_MODE_CALIBRATING) {
        gfx->setTextSize(LOC_SZ);
        gfx->setCursor(LOC_X, LOC_Y);
        gfx->setTextColor(WHITE, BLACK);
        gfx->print(mygrid);
      }

      if (!hasTime || gll_h != hours || gll_m != minutes) {
        hasTime = true;
        hours = gll_h;
        minutes = gll_m;
        if (currentUiMode != UI_MODE_CALIBRATING) {
          sprintf(&buf[0], "%02d:%02dZ", hours, minutes);
          gfx->setTextSize(TIME_SZ);
          gfx->setCursor(TIME_X, TIME_Y);
          gfx->setTextColor(WHITE, BLACK);
          gfx->print(buf);
        }
      }

      updateBearings();
    }
  }
}

void injectSimSentence(const char *sentence) {
  if (!simModeActive) {
    simModeActive = true;
    Serial.println(F("[SIM] Simulation mode activated by sentence injection. Hardware Serial2 suppressed."));
  }
  simPacketCount++;
  lastSimInjectionMillis = millis();
  totalGpsBytesReceived += strlen(sentence);
  lastGpsByteMillis = millis();

  char sbuf[256];
  strncpy(sbuf, sentence, sizeof(sbuf) - 1);
  sbuf[sizeof(sbuf) - 1] = '\0';
  processGpsSentence(sbuf);
}

void handleSimSerialCommand(const String &cmd) {
  String line = cmd;
  line.trim();
  String arg = "";
  if (line.length() > 3) {
    arg = line.substring(3);
    arg.trim();
  }

  if (arg.length() == 0 || arg.equalsIgnoreCase("status")) {
    Serial.println();
    Serial.println(F("=== GPS Simulation Mode Status ==="));
    Serial.printf("  Simulation Active : %s\r\n", simModeActive ? "YES (Hardware Serial2 suppressed)" : "NO (Live Serial2 active)");
    Serial.printf("  Sim Packets Recv  : %u\r\n", simPacketCount);
    if (simPacketCount > 0) {
      Serial.printf("  Last Sim Packet   : %lu ms ago\r\n", millis() - lastSimInjectionMillis);
    }
    Serial.printf("  Current Heading   : %5.1f deg (Quality: %s)\r\n",
                  hasHeading ? bearingFilter.getCalibratedHeading(calibrationOffset) : 0.0,
                  currentBearingQuality == BEARING_FIXED ? "FIXED" : (currentBearingQuality == BEARING_FLOAT ? "FLOAT" : "LOST"));
    Serial.printf("  Current Position  : %9.5f, %9.5f (Grid: %s)\r\n", mylatitude, mylongitude, mygrid);
    Serial.printf("  Rover Mode Active : %s\r\n", rovdist ? "YES (<16k alert tracking)" : "NO (Stationary target tracking)");
    if (rovdist) {
      Serial.printf("  Rover Distance    : %5.1f km (Alert: %s)\r\n", distance / 1000.0, (floor(distance / 1000.0) < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR");
    }
    Serial.println(F("=================================="));
    Serial.println();
    return;
  }

  if (arg.equalsIgnoreCase("on")) {
    simModeActive = true;
    Serial.println(F("[SIM] Simulation mode enabled. Hardware Serial2 input is now SUPPRESSED."));
    return;
  }

  if (arg.equalsIgnoreCase("off")) {
    simModeActive = false;
    Serial.println(F("[SIM] Simulation mode disabled. Hardware Serial2 receiver link RESTORED."));
    return;
  }

  simModeActive = true;

  if (arg.startsWith("hdg") || arg.startsWith("HDG")) {
    String p = arg.substring(3);
    p.trim();
    int sp = p.indexOf(' ');
    float h = 0.0f;
    BearingQuality q = BEARING_FIXED;
    if (sp > 0) {
      h = p.substring(0, sp).toFloat();
      String qstr = p.substring(sp + 1);
      qstr.trim();
      if (qstr.equalsIgnoreCase("float")) q = BEARING_FLOAT;
      else if (qstr.equalsIgnoreCase("lost") || qstr.equalsIgnoreCase("invalid")) q = BEARING_LOST;
      else q = BEARING_FIXED;
    } else {
      h = p.toFloat();
    }
    hasHeading = true;
    lastHeadingUpdate = millis();
    currentBearingQuality = q;
    bearingFilter.seed(h);
    drawCurrentBearing();
    Serial.printf("[SIM] Heading set to %5.1f deg (Quality: %s)\r\n", h, q == BEARING_FIXED ? "FIXED" : (q == BEARING_FLOAT ? "FLOAT" : "LOST"));
  } else if (arg.startsWith("pos") || arg.startsWith("POS")) {
    String p = arg.substring(3);
    p.trim();
    int sp = p.indexOf(' ');
    if (sp > 0) {
      double lat = p.substring(0, sp).toDouble();
      double lon = p.substring(sp + 1).toDouble();
      mylatitude = lat;
      mylongitude = lon;
      hasLocation = true;
      geoPointToGrid(mylatitude, mylongitude, mygrid);
      if (currentUiMode != UI_MODE_CALIBRATING) {
        gfx->setTextSize(LOC_SZ);
        gfx->setCursor(LOC_X, LOC_Y);
        gfx->setTextColor(WHITE, BLACK);
        gfx->print(mygrid);
      }
      updateBearings();
      Serial.printf("[SIM] Position set to %9.5f, %9.5f (Grid: %s)\r\n", lat, lon, mygrid);
    } else {
      Serial.println(F("[SIM ERROR] Usage: sim pos <lat> <lon>"));
    }
  } else if (arg.startsWith("grid") || arg.startsWith("GRID")) {
    String g = arg.substring(4);
    g.trim();
    if (g.length() >= 4 && g.length() <= 8) {
      char gbuf[9];
      strncpy(gbuf, g.c_str(), 8);
      gbuf[8] = '\0';
      double lat = 0.0, lon = 0.0;
      gridToGeoPoint(gbuf, &lat, &lon);
      mylatitude = lat;
      mylongitude = lon;
      hasLocation = true;
      geoPointToGrid(mylatitude, mylongitude, mygrid);
      if (currentUiMode != UI_MODE_CALIBRATING) {
        gfx->setTextSize(LOC_SZ);
        gfx->setCursor(LOC_X, LOC_Y);
        gfx->setTextColor(WHITE, BLACK);
        gfx->print(mygrid);
      }
      updateBearings();
      Serial.printf("[SIM] Position set to Grid %s (%9.5f, %9.5f)\r\n", mygrid, mylatitude, mylongitude);
    } else {
      Serial.println(F("[SIM ERROR] Usage: sim grid <grid4_or_6_or_8>"));
    }
  } else if (arg.startsWith("rover") || arg.startsWith("ROVER")) {
    String p = arg.substring(5);
    p.trim();
    int sp = p.indexOf(' ');
    double orig_lat = mylatitude, orig_lon = mylongitude;
    char orig_grid[7];
    strncpy(orig_grid, mygrid, 6);
    orig_grid[6] = '\0';
    float dist_km = 0.0f;

    if (sp > 0) {
      String og = p.substring(0, sp);
      og.trim();
      dist_km = p.substring(sp + 1).toFloat();
      if (og.length() >= 4) {
        strncpy(orig_grid, og.c_str(), 6);
        orig_grid[6] = '\0';
        gridToGeoPoint(orig_grid, &orig_lat, &orig_lon);
      }
    } else if (p.length() > 0) {
      dist_km = p.toFloat();
    }

    for (int i = 0; i < 6; i++) remote[i] = orig_grid[i];
    remlatitude = orig_lat;
    remlongitude = orig_lon;
    rovdist = true;

    double lat_offset = (dist_km * 1000.0 * cos(toRad(45.0))) / 111132.954;
    double lon_offset = (dist_km * 1000.0 * sin(toRad(45.0))) / (111132.954 * cos(toRad(orig_lat)));
    mylatitude = orig_lat + lat_offset;
    mylongitude = orig_lon + lon_offset;
    hasLocation = true;
    geoPointToGrid(mylatitude, mylongitude, mygrid);

    if (currentUiMode != UI_MODE_CALIBRATING) {
      gfx->setTextSize(LOC_SZ);
      gfx->setCursor(LOC_X, LOC_Y);
      gfx->setTextColor(WHITE, BLACK);
      gfx->print(mygrid);
    }
    printrem();
    updateBearings();

    int rdist = (int)floor(distance / 1000.0);
    Serial.printf("[SIM ROVER] Origin: %s (%.5f, %.5f) | Current: %s (%.5f, %.5f)\r\n",
                  remote, remlatitude, remlongitude, mygrid, mylatitude, mylongitude);
    Serial.printf("[SIM ROVER] Distance: %5.1f km (%d km) | Alert: %s\r\n",
                  distance / 1000.0, rdist, (rdist < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR (>= 16 km)");
  } else if (arg.startsWith("time") || arg.startsWith("TIME")) {
    String t = arg.substring(4);
    t.trim();
    int colon = t.indexOf(':');
    if (colon > 0) {
      int h = t.substring(0, colon).toInt();
      int m = t.substring(colon + 1).toInt();
      if (h >= 0 && h < 24 && m >= 0 && m < 60) {
        hasTime = true;
        hours = h;
        minutes = m;
        if (currentUiMode != UI_MODE_CALIBRATING) {
          char tbuf[9];
          sprintf(tbuf, "%02d:%02dZ", hours, minutes);
          gfx->setTextSize(TIME_SZ);
          gfx->setCursor(TIME_X, TIME_Y);
          gfx->setTextColor(WHITE, BLACK);
          gfx->print(tbuf);
        }
        Serial.printf("[SIM] UTC time set to %02d:%02dZ\r\n", hours, minutes);
      }
    }
  } else {
    Serial.println(F("[SIM ERROR] Unknown sim command. Available:"));
    Serial.println(F("  sim on / sim off"));
    Serial.println(F("  sim status"));
    Serial.println(F("  sim hdg <deg> [fixed|float|lost]"));
    Serial.println(F("  sim pos <lat> <lon>"));
    Serial.println(F("  sim grid <grid>"));
    Serial.println(F("  sim rover [origin_grid] <dist_km>"));
    Serial.println(F("  sim time <hh:mm>"));
  }
}

void handleSerialCommandLine(const String &cmd) {
  String line = cmd;
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("$")) {
    injectSimSentence(line.c_str());
  } else if (line.startsWith("sim") || line.startsWith("SIM")) {
    handleSimSerialCommand(line);
  } else if (line.equalsIgnoreCase("help") || line.equals("?")) {
    Serial.println();
    Serial.println(F("=== GNSS Compass Serial Commands ==="));
    Serial.println(F("  status (or gps)       - Live GNSS receiver stream diagnostic"));
    Serial.println(F("  test (or demo)        - Run visual display self-test demo"));
    Serial.println(F("  cal [snap|zero|<val>] - View or configure antenna calibration"));
    Serial.println(F("  sim [on|off|hdg|pos|grid|rover|status] - GPS simulation commands"));
    Serial.println(F("  $<NMEA/KSXT sentence> - Inject raw GNSS sentence directly"));
    Serial.println(F("  help (or ?)           - Show this help menu"));
    Serial.println(F("===================================="));
    Serial.println();
  } else if (line.equalsIgnoreCase("status") || line.equalsIgnoreCase("gps") || line.equalsIgnoreCase("diag") || line.equalsIgnoreCase("s") || line.equalsIgnoreCase("g")) {
    printGpsStatusReport();
  } else if (line.equalsIgnoreCase("test") || line.equalsIgnoreCase("demo") || line.equalsIgnoreCase("t")) {
    runVisualSelfTest();
  } else if (line.startsWith("cal") || line.startsWith("CAL")) {
    handleCalSerialCommand(line);
  } else {
    Serial.print(F("Unknown command: '"));
    Serial.print(line);
    Serial.println(F("'. Type 'help' for available commands."));
  }
}

void processSerialInput() {
  static char lineBuf[256];
  static size_t lineIdx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineIdx > 0) {
        if (lineBuf[0] != '$') {
          Serial.println(); // Echo newline to terminal for interactive commands
        }
        lineBuf[lineIdx] = '\0';
        handleSerialCommandLine(String(lineBuf));
        lineIdx = 0;
      }
    } else if (c == '\b' || c == 127) { // Backspace or DEL
      if (lineIdx > 0) {
        lineIdx--;
        Serial.print(F("\b \b")); // Visually erase character from terminal screen
      }
    } else if (lineIdx < sizeof(lineBuf) - 1) {
      if (c >= 32 && c <= 126) {
        lineBuf[lineIdx++] = c;
        if (lineBuf[0] != '$') {
          Serial.write(c); // Echo character back to terminal
        }
      }
    }
  }
}

void loop() {

  static char sbuff[1000];
  static int ptr = 0;

  while (1) {

    // Heading staleness watchdog: turns bearing RED after timeout if previously acquired
    if (hasHeading && currentBearingQuality != BEARING_LOST) {
      if (millis() - lastHeadingUpdate >= HEADING_TIMEOUT_MS) {
        currentBearingQuality = BEARING_LOST;
        drawCurrentBearing();
      }
    }

    // Non-blocking serial command processor (handles CR, LF, and CRLF)
    processSerialInput();

    if (hasChanged) {
      hasChanged = false;

      if (currentUiMode == UI_MODE_CALIBRATING) {
        calModalOpenedMillis = millis();
        if (counter > 0) proposedCalibrationOffset += 0.1f;
        else proposedCalibrationOffset -= 0.1f;
        proposedCalibrationOffset = normalizeOffset(proposedCalibrationOffset);
        counter = 0;
        drawProposedOffsetRow();
      } else if (isEditing) {
        lastEditActivity = millis();
        rovdist = false;
        if (counter > 0) INCG(remote[remotec]);
        else DECG(remote[remotec]);

        printrem();
        updateBearings();
      }
    }

    #define LONG_PRESS_MS 1500

    static bool isTouching = false;
    static unsigned long touchStartTime = 0;
    static CST_TS_Point touchStartPoint;
    static bool longPressTriggered = false;
    static bool holdingSaveBtn = false;
    static unsigned long lastTouchReleaseTime = 0;

    if (tsPanel.touched()) {
      CST_TS_Point p = tsPanel.getPoint(0);
      if (p.x != 0 || p.y != 0) {
        if (!isTouching) {
          if (millis() - lastTouchReleaseTime >= 100) {
            isTouching = true;
            touchStartTime = millis();
            touchStartPoint = p;
            longPressTriggered = false;
            holdingSaveBtn = false;

            if (currentUiMode == UI_MODE_CALIBRATING) {
              calModalOpenedMillis = millis(); // Refresh modal timeout
              // Save button: X 55..425, Y 284..332
              if (55 <= p.x && p.x <= 425 && 284 <= p.y && p.y <= 332) {
                holdingSaveBtn = true;
              } else if (55 <= p.x && p.x <= 235 && 230 <= p.y && p.y <= 278) {
                if (hasHeading) drawSnapButton(true);
              } else if (245 <= p.x && p.x <= 425 && 230 <= p.y && p.y <= 278) {
                drawResetButton(true);
              } else if (130 <= p.x && p.x <= 350 && 340 <= p.y && p.y <= 385) {
                drawCancelButton(true);
              }
            }
          }
        } else {
          // Touch is being held
          unsigned long holdDuration = millis() - touchStartTime;

          // Check if finger drifted too far
          if (abs(p.x - touchStartPoint.x) > 35 || abs(p.y - touchStartPoint.y) > 35) {
            if (holdingSaveBtn) {
              holdingSaveBtn = false;
              drawSaveButton(false, 0.0f);
            }
            if (currentUiMode == UI_MODE_CALIBRATING) {
              drawSnapButton(false);
              drawResetButton(false);
              drawCancelButton(false);
            }
          }

          if (holdingSaveBtn && !longPressTriggered) {
            calModalOpenedMillis = millis();
            float prog = (float)holdDuration / (float)LONG_PRESS_MS;
            drawSaveButton(true, prog);

            if (holdDuration >= LONG_PRESS_MS) {
              longPressTriggered = true;
              holdingSaveBtn = false;
              saveCalibration(proposedCalibrationOffset);
              currentUiMode = UI_MODE_NORMAL;
              redrawMainScreen();
            }
          }
        }
      }
    } else {
      // Touch released
      if (isTouching) {
        unsigned long pressDuration = millis() - touchStartTime;
        isTouching = false;
        lastTouchReleaseTime = millis();

        if (holdingSaveBtn) {
          holdingSaveBtn = false;
          drawSaveButton(false, 0.0f);
        } else if (currentUiMode == UI_MODE_CALIBRATING && pressDuration >= LONG_PRESS_MS) {
          drawSnapButton(false);
          drawResetButton(false);
          drawCancelButton(false);
        }

        // Handle short taps if long press did not trigger
        if (!longPressTriggered && pressDuration < LONG_PRESS_MS) {
          CST_TS_Point p = touchStartPoint;

          if (currentUiMode == UI_MODE_CALIBRATING) {
            calModalOpenedMillis = millis();

            // Button 1: [ SNAP TARGET ] (X: 55..235, Y: 230..278) -> Tap updates display only!
            if (55 <= p.x && p.x <= 235 && 230 <= p.y && p.y <= 278) {
              drawSnapButton(true);
              if (hasHeading) {
                double rawHdg = bearingFilter.getRawHeading();
                proposedCalibrationOffset = normalizeOffset((float)(bearing - rawHdg));
                drawProposedOffsetRow();
              }
              delay(120);
              drawSnapButton(false);
            }
            // Button 2: [ RESET (0.0) ] (X: 245..425, Y: 230..278) -> Tap resets display only!
            else if (245 <= p.x && p.x <= 425 && 230 <= p.y && p.y <= 278) {
              drawResetButton(true);
              proposedCalibrationOffset = 0.0f;
              drawProposedOffsetRow();
              delay(120);
              drawResetButton(false);
            }
            // Button 4: [ CANCEL ] (X: 130..350, Y: 340..385) -> Tap cancels immediately!
            else if (130 <= p.x && p.x <= 350 && 340 <= p.y && p.y <= 385) {
              drawCancelButton(true);
              delay(120);
              currentUiMode = UI_MODE_NORMAL;
              redrawMainScreen();
            } else {
              drawSnapButton(false);
              drawResetButton(false);
              drawCancelButton(false);
            }
          } else {
            // Normal Mode Short Taps

            // Tap on CAL Pill (Row 3 center slot) -> Immediate tap opens modal!
            if (CAL_TOUCH_X_MIN <= p.x && p.x <= CAL_TOUCH_X_MAX && CAL_TOUCH_Y_MIN <= p.y && p.y <= CAL_TOUCH_Y_MAX) {
              if (hasHeading) {
                double rawHdg = bearingFilter.getRawHeading();
                proposedCalibrationOffset = normalizeOffset((float)(bearing - rawHdg));
              } else {
                proposedCalibrationOffset = calibrationOffset;
              }
              currentUiMode = UI_MODE_CALIBRATING;
              calModalOpenedMillis = millis();
              isEditing = false;
              drawCalibrationModal();
            }

            // Tap on local grid (Row 1): copy local grid to target for rover distance check
            else if (100 <= p.x && p.x <= 340 && 0 <= p.y && p.y <= 75) {
              for (int ii = 0; ii < 6; ii++) {
                remote[ii] = mygrid[ii];
              }
              remlatitude = mylatitude;
              remlongitude = mylongitude;
              rovdist = true;
              printrem();
              updateBearings();
            }

            // Tap on target grid (Row 2): select character and enter edit mode
            else if (TOUCH_X_MIN <= p.x && p.x <= TOUCH_X_MAX && TOUCH_Y_MIN <= p.y && p.y <= TOUCH_Y_MAX) {
              int touched_char = (p.x - REMSTARTX) / CHAR_W(REMSIZE);
              if (touched_char < 0) touched_char = 0;
              if (touched_char > 5) touched_char = 5;
              remotec = touched_char;
              isEditing = true;
              lastEditActivity = millis();
              printrem();
            }

            // Tap on firmware version footer (Row 7): trigger visual display demo
            else if (100 <= p.x && p.x <= 380 && p.y >= 435) {
              runVisualSelfTest();
            }
          }
        }
      }
    }

    if (currentUiMode == UI_MODE_CALIBRATING) {
      if (millis() - calModalOpenedMillis >= CAL_MODAL_TIMEOUT_MS) {
        currentUiMode = UI_MODE_NORMAL;
        redrawMainScreen();
      }
    }

    if (isEditing && EDIT_TIMEOUT_MS > 0) {
      if (millis() - lastEditActivity >= EDIT_TIMEOUT_MS) {
        isEditing = false;
        printrem();
      }
    }



    if (int num = gpsSerial.available()) {
      if (simModeActive) {
        // Discard physical receiver bytes while simulation mode is active
        while (gpsSerial.available()) {
          gpsSerial.read();
        }
      } else {
        int num1 = gpsSerial.readBytesUntil('\n',&sbuff[ptr],num); // read all the available characters up to the end of line
        ptr += num1;
        totalGpsBytesReceived += num1;
        lastGpsByteMillis = millis();

        if (num != num1) {   // if the line ends before all the available characters were read
          gpsSerial.read();  // consume the '\n'
          totalGpsBytesReceived++;
          sbuff[ptr]=0;      // terminate the string
          Serial.println(sbuff);
          ptr=0;             // set the ptr back to 0 for the next line from the gps

          processGpsSentence(sbuff);
        }
      }
    }
  }
}

