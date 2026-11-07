#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <PCF8574.h>
#include <Adafruit_CST8XX.h>
#include <Preferences.h>
#include <math.h>
#include <ctype.h>

#include "src/config.h"
#include "src/types.h"
#include "src/geo/geo_math.h"
#include "src/geo/maidenhead.h"
#include "src/protocol/nmea_parser.h"
#include "src/protocol/serial_cli.h"
#include "src/system/calibration.h"
#include "src/gnss/simulator.h"
#include "src/gnss/gps_manager.h"

char remote[7] = "ZA00AA";
char mygrid[9] = "JJ00AA00";
bool hasLocation = false;
bool hasTime = false;
int hours = 0;
int minutes = 0;

// System Subsystems
GpsManager gpsManager;
Calibration calibration;
GpsSimulator gpsSimulator;
SerialCli serialCli(gpsManager, calibration, gpsSimulator);

BearingFilter &bearingFilter = gpsManager.getBearingFilter();

// --- UI State Variables ---
enum UiMode {
  UI_MODE_NORMAL = 0,
  UI_MODE_CALIBRATING = 1
};
UiMode currentUiMode = UI_MODE_NORMAL;
unsigned long calModalOpenedMillis = 0;

volatile int counter=0;
volatile int encState = 0;
volatile int oldState = -1;
volatile bool hasChanged = false;

PCF8574 pcf8574(PCF8574_ADDR);

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
bool hasHeading = false;
BearingQuality currentBearingQuality = BEARING_NONE;
unsigned long lastHeadingUpdate = 0;
bool rovdist = false;
double mylongitude = 0.0, mylatitude = 0.0;
double remlongitude = 0.0, remlatitude = 0.0;
double distance = 0.0, bearing = 0.0, revbearing = 0.0;

void loadCalibration() {
  calibration.begin();
}

void saveCalibration(float offset) {
  calibration.setOffset(offset);
}

void drawCalibrationPill() {
  if (currentUiMode == UI_MODE_CALIBRATING) return;

  // Clear pill region
  gfx->fillRect(CAL_PILL_X - 2, CAL_PILL_Y - 2, CAL_PILL_W + 4, CAL_PILL_H + 4, BLACK);

  uint16_t borderColor = DARK_GRAY_BORDER; // Dark gray border
  uint16_t textColor = DIM_GRAY_TEXT;     // Dim gray text

  if (calibration.isCalibrated()) {
    borderColor = CYAN;          // Cyan border when active
    textColor = CYAN;            // Cyan text when active
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
  if (!calibration.isCalibrated()) {
    strcpy(valStr, "OFF");
  } else {
    sprintf(valStr, "%+4.1f", calibration.getOffset());
  }
  int txtW = strlen(valStr) * CHAR_W(2);
  gfx->setCursor(CAL_PILL_X + (CAL_PILL_W - txtW) / 2, CAL_PILL_Y + 24);
  gfx->print(valStr);
  gfx->setTextColor(WHITE, BLACK);
}

void drawProposedOffsetRow() {
  gfx->fillRect(50, 198, 380, 26, MODAL_BG_COLOR);
  gfx->setTextSize(2);
  gfx->setTextColor(RICH_YELLOW, MODAL_BG_COLOR);
  char buf[36];
  sprintf(buf, "Proposed Offset: %+5.1f", calibration.getProposedOffset());
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
  sprintf(buf, "Current Offset : %+5.1f", calibration.getOffset());
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
    sprintf(str, "%5.1f", bearingFilter.getCalibratedHeading(calibration.getOffset()));
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
  Serial.begin(115200);
  Serial.print("GNSS Compass ");
  Serial.println(FW_VERSION);
  Serial.println(F("Commands: 'status' (GPS stream diagnosis), 'test' (visual demo), 'sim' (simulation)"));

  initPins();
  initBacklight();
  initLCD();
  initEncoder();
  loadCalibration();

  gpsManager.setSimulator(&gpsSimulator);
  gpsManager.begin(GPS_BAUD, RXD2, TXD2);

  // Wire GPS Manager event callbacks to UI presentation
  gpsManager.setOnHeadingUpdated([](float /* heading */, BearingQuality quality) {
    hasHeading = gpsManager.hasHeading();
    currentBearingQuality = quality;
    lastHeadingUpdate = gpsManager.getLastHeadingUpdate();
    drawCurrentBearing();
  });

  gpsManager.setOnLocationUpdated([](const GeoPoint &pos) {
    hasLocation = true;
    mylatitude = pos.latitude;
    mylongitude = pos.longitude;
    geoPointToGrid(mylatitude, mylongitude, mygrid);

    if (currentUiMode != UI_MODE_CALIBRATING) {
      gfx->setTextSize(LOC_SZ);
      gfx->setCursor(LOC_X, LOC_Y);
      gfx->setTextColor(WHITE, BLACK);
      gfx->print(mygrid);
    }
    updateBearings();
  });

  gpsManager.setOnTimeUpdated([](int h, int m) {
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
  });

  serialCli.begin();
  serialCli.setRunTestCallback(runVisualSelfTest);
  serialCli.setStateChangedCallback([]() {
    hasHeading = gpsManager.hasHeading();
    currentBearingQuality = gpsManager.getBearingQuality();
    lastHeadingUpdate = gpsManager.getLastHeadingUpdate();
    if (gpsManager.hasLocation()) {
      hasLocation = true;
      mylatitude = gpsManager.getLocation().latitude;
      mylongitude = gpsManager.getLocation().longitude;
      geoPointToGrid(mylatitude, mylongitude, mygrid);
      if (currentUiMode != UI_MODE_CALIBRATING) {
        gfx->setTextSize(LOC_SZ);
        gfx->setCursor(LOC_X, LOC_Y);
        gfx->setTextColor(WHITE, BLACK);
        gfx->print(mygrid);
      }
    }
    if (gpsManager.hasTime()) {
      hasTime = true;
      hours = gpsManager.getHours();
      minutes = gpsManager.getMinutes();
      if (currentUiMode != UI_MODE_CALIBRATING) {
        char tbuf[9];
        sprintf(tbuf, "%02d:%02dZ", hours, minutes);
        gfx->setTextSize(TIME_SZ);
        gfx->setCursor(TIME_X, TIME_Y);
        gfx->setTextColor(WHITE, BLACK);
        gfx->print(tbuf);
      }
    }
    updateBearings();
    drawCurrentBearing();
  });

  serialCli.setRoverSimCallback([](const char *orig_grid, double dist_km) {
    double orig_lat = mylatitude, orig_lon = mylongitude;
    if (orig_grid && strlen(orig_grid) >= 4) {
      gridToGeoPoint(orig_grid, &orig_lat, &orig_lon);
      strncpy(remote, orig_grid, 6);
      remote[6] = '\0';
    }
    for (int i = 0; i < 6; i++) remote[i] = orig_grid[i];
    remlatitude = orig_lat;
    remlongitude = orig_lon;
    rovdist = true;

    GeoPoint originPoint(orig_lat, orig_lon, true);
    GeoPoint simulatedPoint = GpsSimulator::calculateRoverDisplacement(originPoint, dist_km, 45.0);
    gpsManager.setLocation(simulatedPoint);
    printrem();
    updateBearings();
    int rdist = (int)floor(distance / 1000.0);
    Serial.printf("[SIM ROVER] Origin: %s (%.5f, %.5f) | Current: %s (%.5f, %.5f)\r\n",
                  remote, remlatitude, remlongitude, mygrid, mylatitude, mylongitude);
    Serial.printf("[SIM ROVER] Distance: %5.1f km (%d km) | Alert: %s\r\n",
                  distance / 1000.0, rdist, (rdist < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR (>= 16 km)");
  });

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
  serialCli.printGpsStatus();
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
  float savedOffset = calibration.getOffset();
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

  GpsLinkVerdict liveVerdict = gpsManager.evaluateStreamHealth(millis());

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
  calibration.setOffset(0.0f);
  drawCalibrationPill();
  drawCurrentBearing();
  delay(1500);

  Serial.println(F("  -> Testing Calibration Pill Display (Calibrated: +4.2 deg)"));
  showFooterText("TEST 7: CAL PILL +4.2", GREEN);
  calibration.setOffset(4.2f);
  drawCalibrationPill();
  drawCurrentBearing();
  delay(1500);

  Serial.println(F("  -> Demonstrating Calibration Modal Dialog"));
  showFooterText("TEST 7: CAL MODAL", RICH_YELLOW);
  calibration.stageProposedOffset(4.2f);
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
  calibration.setOffset(savedOffset);
  currentUiMode = saved_uiMode;
  redrawMainScreen();

  Serial.println(F("=== Visual Self-Test Complete (Live state restored) ===\n"));
}

void loop() {
  while (1) {
    // 1. Ingest GPS UART bytes & check staleness timeouts
    gpsManager.update();

    // 2. Process USB host commands (SIM, CAL, GPS, TEST)
    serialCli.setTargetBearing(bearing);
    serialCli.setTargetGrid(remote);
    serialCli.setRoverAlertActive(rovdist);
    serialCli.setRoverDistance(distance);
    serialCli.update();

    if (hasChanged) {
      hasChanged = false;

      if (currentUiMode == UI_MODE_CALIBRATING) {
        calModalOpenedMillis = millis();
        if (counter > 0) calibration.adjustProposedOffset(0.1f);
        else calibration.adjustProposedOffset(-0.1f);
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
              calibration.commitProposed();
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
                calibration.snapProposed(bearing, rawHdg);
                drawProposedOffsetRow();
              }
              delay(120);
              drawSnapButton(false);
            }
            // Button 2: [ RESET (0.0) ] (X: 245..425, Y: 230..278) -> Tap resets display only!
            else if (245 <= p.x && p.x <= 425 && 230 <= p.y && p.y <= 278) {
              drawResetButton(true);
              calibration.resetProposed();
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
                calibration.snapProposed(bearing, rawHdg);
              } else {
                calibration.syncProposedFromActive();
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
  }
}

