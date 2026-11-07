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
#include "src/ui/input_manager.h"
#include "src/ui/cal_modal.h"
#include "src/ui/display_ui.h"

char remote[7] = "ZA00AA";
char mygrid[9] = "JJ00AA00";
bool hasLocation = false;
bool hasTime = false;
int hours = 0;
int minutes = 0;

// Subsystems
GpsManager gpsManager;
Calibration calibration;
GpsSimulator gpsSimulator;
SerialCli serialCli(gpsManager, calibration, gpsSimulator);
InputManager inputManager;
DisplayUi displayUi;

BearingFilter &bearingFilter = gpsManager.getBearingFilter();

// UI State Variables
enum UiModeLegacy {
  UI_MODE_NORMAL = 0,
  UI_MODE_CALIBRATING = 1
};
UiModeLegacy currentUiMode = UI_MODE_NORMAL;
unsigned long calModalOpenedMillis = 0;

int remotec = 0;
bool isEditing = true;
unsigned long lastEditActivity = 0;

// Bearing Quality & State Machine
bool hasHeading = false;
BearingQuality currentBearingQuality = BEARING_NONE;
unsigned long lastHeadingUpdate = 0;
bool rovdist = false;
double mylongitude = 0.0, mylatitude = 0.0;
double remlongitude = 0.0, remlatitude = 0.0;
double distance = 0.0, bearing = 0.0, revbearing = 0.0;

// Forward declarations
void updateBearings();
void drawCurrentBearing();
void redrawMainScreen();
void runVisualSelfTest();
void handleUserEvent(const UserEvent &event);

void drawCurrentBearing() {
  if (currentUiMode == UI_MODE_CALIBRATING) {
    displayUi.getCalModal().drawRawHeading(bearingFilter.getRawHeading(), hasHeading);
    return;
  }
  displayUi.renderCurrentBearing(bearingFilter.getCalibratedHeading(calibration.getOffset()),
                                currentBearingQuality, hasHeading);
}

void updateBearings() {
  gridToGeoPoint(remote, &remlatitude, &remlongitude);
  calculateWgs84(mylatitude, mylongitude, remlatitude, remlongitude, distance, bearing);
  calculateWgs84(remlatitude, remlongitude, mylatitude, mylongitude, distance, revbearing);

  if (currentUiMode == UI_MODE_CALIBRATING) {
    return;
  }

  displayUi.renderReverseBearingAndDistance(revbearing, distance, rovdist);
  drawCurrentBearing();
  displayUi.renderCalibrationPill(calibration.getOffset(), calibration.isCalibrated());
  displayUi.renderTargetBearing(bearing);
  displayUi.renderArrlDistance(mygrid, remote);
}

void redrawMainScreen() {
  displayUi.redrawMainScreen(mygrid, remote, remotec, isEditing,
                             bearing, revbearing, distance, rovdist,
                             bearingFilter.getCalibratedHeading(calibration.getOffset()),
                             currentBearingQuality, hasHeading,
                             calibration.getOffset(), calibration.isCalibrated(),
                             hours, minutes, hasTime);
}

void setup() {
  Serial.begin(115200);
  Serial.print("GNSS Compass ");
  Serial.println(FW_VERSION);
  Serial.println(F("Commands: 'status' (GPS stream diagnosis), 'test' (visual demo), 'sim' (simulation)"));

  displayUi.begin();
  inputManager.begin();
  calibration.begin();

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
      displayUi.renderMyGrid(mygrid);
    }
    updateBearings();
  });

  gpsManager.setOnTimeUpdated([](int h, int m) {
    hasTime = true;
    hours = h;
    minutes = m;
    if (currentUiMode != UI_MODE_CALIBRATING) {
      displayUi.renderUtcTime(hours, minutes, hasTime);
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
        displayUi.renderMyGrid(mygrid);
      }
    }
    if (gpsManager.hasTime()) {
      hasTime = true;
      hours = gpsManager.getHours();
      minutes = gpsManager.getMinutes();
      if (currentUiMode != UI_MODE_CALIBRATING) {
        displayUi.renderUtcTime(hours, minutes, hasTime);
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
    displayUi.renderTargetGrid(remote, remotec, isEditing);
    updateBearings();
    int rdist = (int)floor(distance / 1000.0);
    Serial.printf("[SIM ROVER] Origin: %s (%.5f, %.5f) | Current: %s (%.5f, %.5f)\r\n",
                  remote, remlatitude, remlongitude, mygrid, mylatitude, mylongitude);
    Serial.printf("[SIM ROVER] Distance: %5.1f km (%d km) | Alert: %s\r\n",
                  distance / 1000.0, rdist, (rdist < 16) ? "ACTIVE (RED on WHITE)" : "CLEAR (>= 16 km)");
  });

  geoPointToGrid(mylatitude, mylongitude, mygrid);
  displayUi.renderMyGrid(mygrid);

  lastEditActivity = millis();
  displayUi.renderTargetGrid(remote, remotec, isEditing);
  updateBearings();

  displayUi.renderUtcTime(0, 0, false);
  displayUi.showFooterText(FW_VERSION, DIM_GRAY_TEXT);
}

void handleUserEvent(const UserEvent &event) {
  switch (event.type) {
    case UserEventType::RotateClockwise:
      if (currentUiMode == UI_MODE_CALIBRATING) {
        calModalOpenedMillis = millis();
        calibration.adjustProposedOffset(0.1f);
        displayUi.getCalModal().drawProposedOffsetRow(calibration.getProposedOffset());
      } else if (isEditing) {
        lastEditActivity = millis();
        rovdist = false;
        INCG(remote[remotec]);
        displayUi.renderTargetGrid(remote, remotec, isEditing);
        updateBearings();
      }
      break;

    case UserEventType::RotateCounterClockwise:
      if (currentUiMode == UI_MODE_CALIBRATING) {
        calModalOpenedMillis = millis();
        calibration.adjustProposedOffset(-0.1f);
        displayUi.getCalModal().drawProposedOffsetRow(calibration.getProposedOffset());
      } else if (isEditing) {
        lastEditActivity = millis();
        rovdist = false;
        DECG(remote[remotec]);
        displayUi.renderTargetGrid(remote, remotec, isEditing);
        updateBearings();
      }
      break;

    case UserEventType::TouchCalibrationPill:
      if (hasHeading) {
        calibration.snapProposed(bearing, bearingFilter.getRawHeading());
      } else {
        calibration.syncProposedFromActive();
      }
      currentUiMode = UI_MODE_CALIBRATING;
      inputManager.setUiMode(UiMode::Calibrating);
      calModalOpenedMillis = millis();
      isEditing = false;
      displayUi.getCalModal().drawModal(bearing, bearingFilter.getRawHeading(), hasHeading,
                                        calibration.getOffset(), calibration.getProposedOffset());
      break;

    case UserEventType::TouchLocalGrid:
      for (int i = 0; i < 6; i++) remote[i] = mygrid[i];
      remlatitude = mylatitude;
      remlongitude = mylongitude;
      rovdist = true;
      displayUi.renderTargetGrid(remote, remotec, isEditing);
      updateBearings();
      break;

    case UserEventType::TouchGridCharacter:
      remotec = event.gridIndex;
      isEditing = true;
      lastEditActivity = millis();
      displayUi.renderTargetGrid(remote, remotec, isEditing);
      break;

    case UserEventType::TouchVersionFooter:
      runVisualSelfTest();
      break;

    case UserEventType::ModalButtonDown:
      if (event.buttonId == ModalButtonId::Snap && hasHeading) {
        displayUi.getCalModal().drawSnapButton(true, hasHeading);
      } else if (event.buttonId == ModalButtonId::Reset) {
        displayUi.getCalModal().drawResetButton(true);
      } else if (event.buttonId == ModalButtonId::Cancel) {
        displayUi.getCalModal().drawCancelButton(true);
      }
      break;

    case UserEventType::ModalButtonUp:
      displayUi.getCalModal().drawSnapButton(false, hasHeading);
      displayUi.getCalModal().drawResetButton(false);
      displayUi.getCalModal().drawCancelButton(false);
      break;

    case UserEventType::HoldSaveButtonProgress:
      calModalOpenedMillis = millis();
      displayUi.getCalModal().drawSaveButton(event.progress > 0.0f, event.progress);
      break;

    case UserEventType::SaveButtonCommitted:
      calibration.commitProposed();
      currentUiMode = UI_MODE_NORMAL;
      inputManager.setUiMode(UiMode::Normal);
      redrawMainScreen();
      break;

    case UserEventType::SnapOffsetTriggered:
      calModalOpenedMillis = millis();
      displayUi.getCalModal().drawSnapButton(true, hasHeading);
      if (hasHeading) {
        calibration.snapProposed(bearing, bearingFilter.getRawHeading());
        displayUi.getCalModal().drawProposedOffsetRow(calibration.getProposedOffset());
      }
      delay(120);
      displayUi.getCalModal().drawSnapButton(false, hasHeading);
      break;

    case UserEventType::ResetOffsetTriggered:
      calModalOpenedMillis = millis();
      displayUi.getCalModal().drawResetButton(true);
      calibration.resetProposed();
      displayUi.getCalModal().drawProposedOffsetRow(calibration.getProposedOffset());
      delay(120);
      displayUi.getCalModal().drawResetButton(false);
      break;

    case UserEventType::CancelModalTriggered:
      displayUi.getCalModal().drawCancelButton(true);
      delay(120);
      currentUiMode = UI_MODE_NORMAL;
      inputManager.setUiMode(UiMode::Normal);
      redrawMainScreen();
      break;

    default:
      break;
  }
}

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
  UiModeLegacy saved_uiMode = currentUiMode;
  isEditing = false;

  // Test 1: Same Grid (ARRL 1.0 km minimum rule)
  Serial.println(F("[Test 1/6] Same Grid (ARRL 1.0 km rule)"));
  displayUi.showFooterText("TEST 1: SAME GRID", CYAN);
  strcpy(mygrid, "CM87wa00");
  strcpy(remote, "CM87wa");
  gridToGeoPoint("CM87wa", &mylatitude, &mylongitude);
  hasHeading = true;
  currentBearingQuality = BEARING_FIXED;
  bearingFilter.seed(180.0f);
  rovdist = false;
  displayUi.renderMyGrid(mygrid);
  displayUi.renderTargetGrid(remote, remotec, isEditing);
  updateBearings();
  delay(3000);

  // Test 2: Rover Transit Alert (< 16 km, highlighted RED on WHITE)
  Serial.println(F("[Test 2/6] Rover Transit Alert (< 16 km, highlighted RED on WHITE)"));
  displayUi.showFooterText("TEST 2: ROVER < 16K", RICH_YELLOW);
  strcpy(remote, "CM87wb");
  currentBearingQuality = BEARING_FIXED;
  rovdist = true;
  displayUi.renderTargetGrid(remote, remotec, isEditing);
  updateBearings();
  delay(3000);

  // Test 3: Distance Under 1,000 km (~600 km)
  Serial.println(F("[Test 3/6] Under 1,000 km formatting (~600 km)"));
  displayUi.showFooterText("TEST 3: DIST < 1000K", CYAN);
  strcpy(remote, "DN13ab");
  currentBearingQuality = BEARING_FIXED;
  rovdist = false;
  displayUi.renderTargetGrid(remote, remotec, isEditing);
  updateBearings();
  delay(3000);

  // Test 4: Distance Over 1,000 km (~2,500 km)
  Serial.println(F("[Test 4/6] Over 1,000 km formatting (~2,500 km)"));
  displayUi.showFooterText("TEST 4: DIST > 1000K", CYAN);
  strcpy(remote, "EL29ja");
  currentBearingQuality = BEARING_FIXED;
  displayUi.renderTargetGrid(remote, remotec, isEditing);
  updateBearings();
  delay(3000);

  // Test 5: Extreme Distance (> 10,000 km) and Recovery to 1.0 km
  Serial.println(F("[Test 5/6] Extreme distance (> 10,000 km)"));
  displayUi.showFooterText("TEST 5: GHOST RECOVERY", CYAN);
  strcpy(remote, "ZA00AA");
  displayUi.renderTargetGrid(remote, remotec, isEditing);
  updateBearings();
  delay(2000);

  Serial.println(F("  -> Dropping back to 1.0 km (verifying no ghost characters)"));
  strcpy(remote, "CM87wa");
  displayUi.renderTargetGrid(remote, remotec, isEditing);
  updateBearings();
  delay(2000);

  // Test 6: Bearing Color States & Live GPS Receiver Link Diagnostic
  Serial.println(F("[Test 6/6] Bearing Color States & Live GPS Stream Diagnostic"));
  Serial.println(F("  -> Bearing Color: Pure White (RTK Fixed)"));
  displayUi.showFooterText("TEST 6: BEARING WHITE", WHITE);
  currentBearingQuality = BEARING_FIXED;
  drawCurrentBearing();
  delay(1000);

  Serial.println(F("  -> Bearing Color: Bright Yellow (RTK Float / Degraded)"));
  displayUi.showFooterText("TEST 6: BEARING YELLOW", RICH_YELLOW);
  currentBearingQuality = BEARING_FLOAT;
  drawCurrentBearing();
  delay(1000);

  Serial.println(F("  -> Bearing Color: Bright Red (Lost / Stale)"));
  displayUi.showFooterText("TEST 6: BEARING RED", RED);
  currentBearingQuality = BEARING_LOST;
  drawCurrentBearing();
  delay(1000);

  // Restore bearing to fixed before showing live GPS stream ruling
  currentBearingQuality = BEARING_FIXED;
  drawCurrentBearing();
  displayUi.showFooterText("TEST 6: GPS STREAM", CYAN);
  serialCli.printGpsStatus();

  GpsLinkVerdict liveVerdict = gpsManager.evaluateStreamHealth(millis());

  if (liveVerdict == GPS_LINK_OK_FIXED) {
    displayUi.showStatusCard("GPS: RTK FIXED (OK)", GREEN, BLACK);
  } else if (liveVerdict == GPS_LINK_HEADING_FLOAT) {
    displayUi.showStatusCard("GPS: RTK FLOAT (WARN)", RICH_YELLOW, BLACK);
  } else if (liveVerdict == GPS_LINK_ACQUIRING || liveVerdict == GPS_LINK_POSITION_ONLY) {
    displayUi.showStatusCard("GPS: ACQUIRING (WAIT)", RICH_YELLOW, BLACK);
  } else {
    displayUi.showStatusCard("GPS: NO DATA / CABLE", RED, WHITE);
  }
  delay(2500);
#if defined(ARDUINO)
  displayUi.getGfx()->fillRect(40, 215, 400, 52, BLACK);
#endif

  // Test 7: Antenna Calibration Status & Modal Demo
  Serial.println(F("[Test 7/7] Antenna Calibration Pill & Modal Display Demo"));
  Serial.println(F("  -> Testing Calibration Pill Display (Uncalibrated: OFF)"));
  displayUi.showFooterText("TEST 7: CAL PILL OFF", CYAN);
  calibration.setOffset(0.0f);
  displayUi.renderCalibrationPill(calibration.getOffset(), calibration.isCalibrated());
  drawCurrentBearing();
  delay(1500);

  Serial.println(F("  -> Testing Calibration Pill Display (Calibrated: +4.2 deg)"));
  displayUi.showFooterText("TEST 7: CAL PILL +4.2", GREEN);
  calibration.setOffset(4.2f);
  displayUi.renderCalibrationPill(calibration.getOffset(), calibration.isCalibrated());
  drawCurrentBearing();
  delay(1500);

  Serial.println(F("  -> Demonstrating Calibration Modal Dialog"));
  displayUi.showFooterText("TEST 7: CAL MODAL", RICH_YELLOW);
  calibration.stageProposedOffset(4.2f);
  currentUiMode = UI_MODE_CALIBRATING;
  inputManager.setUiMode(UiMode::Calibrating);
  displayUi.getCalModal().drawModal(bearing, bearingFilter.getRawHeading(), hasHeading,
                                    calibration.getOffset(), calibration.getProposedOffset());
  delay(1200);
  displayUi.getCalModal().drawSnapButton(true, hasHeading);
  delay(300);
  displayUi.getCalModal().drawSnapButton(false, hasHeading);
  delay(1000);

  // Display PASS/FAIL Result Banner
  int failedTests[8];
  int failedCount = 0;

  // Test 1: ARRL 1.0 km minimum rule
  if (fabs(calculateArrlContestDistance("CM87wa", "CM87wa") - 1.0) > 0.001) {
    Serial.println(F("FAIL: Test 1 (ARRL 1 km rule)"));
    failedTests[failedCount++] = 1;
  }

  // Test 2: Rover transit alert threshold (< 16 km)
  double lat_wa, lon_wa, lat_wb, lon_wb, rover_dist, rover_bear;
  gridToGeoPoint("CM87wa", &lat_wa, &lon_wa);
  gridToGeoPoint("CM87wb", &lat_wb, &lon_wb);
  calculateWgs84(lat_wa, lon_wa, lat_wb, lon_wb, rover_dist, rover_bear);
  if ((int)floor(rover_dist / 1000.0) >= 16) {
    Serial.println(F("FAIL: Test 2 (Rover transit <16 km threshold)"));
    failedTests[failedCount++] = 2;
  }

  // Test 3: Mid-range distance formatting (< 1,000 km)
  double lat_dn, lon_dn, d_dn, b_dn;
  gridToGeoPoint("DN13ab", &lat_dn, &lon_dn);
  calculateWgs84(lat_wa, lon_wa, lat_dn, lon_dn, d_dn, b_dn);
  int km_dn = (int)floor(d_dn / 1000.0);
  if (km_dn < 100 || km_dn >= 1000) {
    Serial.println(F("FAIL: Test 3 (Mid-range distance <1000 km formatting)"));
    failedTests[failedCount++] = 3;
  }

  // Test 4: Long-range distance formatting (> 1,000 km)
  double lat_el, lon_el, d_el, b_el;
  gridToGeoPoint("EL29ja", &lat_el, &lon_el);
  calculateWgs84(lat_wa, lon_wa, lat_el, lon_el, d_el, b_el);
  int km_el = (int)floor(d_el / 1000.0);
  if (km_el < 1000 || km_el >= 10000) {
    Serial.println(F("FAIL: Test 4 (Long-range distance >1000 km formatting)"));
    failedTests[failedCount++] = 4;
  }

  // Test 5: Extreme distance (> 10,000 km)
  if (calculateArrlContestDistance("CM87wa", "ZA00AA") < 10000.0) {
    Serial.println(F("FAIL: Test 5 (Extreme distance recovery)"));
    failedTests[failedCount++] = 5;
  }

  // Test 7: Heading offset wrap & normalization
  if (fabs(normalizeOffset(5.0f - 355.0f) - 10.0f) > 0.01f ||
      fabs(normalizeOffset(355.0f - 5.0f) - (-10.0f)) > 0.01f) {
    Serial.println(F("FAIL: Test 7 (Antenna offset normalization)"));
    failedTests[failedCount++] = 7;
  }

  if (failedCount == 0) {
    Serial.println(F("Result: PASS"));
    displayUi.showStatusCard("SELF-TEST PASSED", GREEN, BLACK);
    delay(2500);
  } else {
    char failMsg[32];
    DisplayUi::formatSelfTestFailure(failedTests, failedCount, failMsg, sizeof(failMsg));
    Serial.print(F("Result: FAIL -> "));
    Serial.println(failMsg);
    displayUi.showStatusCard(failMsg, RED, WHITE);
    delay(4000);
  }
  delay(2500);
#if defined(ARDUINO)
  displayUi.getGfx()->fillRect(40, 215, 400, 52, BLACK);
#endif

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
  inputManager.setUiMode(saved_uiMode == UI_MODE_CALIBRATING ? UiMode::Calibrating : UiMode::Normal);
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

    // 3. Process Input events (rotary encoder and touch gestures)
    inputManager.update();
    UserEvent event;
    while (inputManager.pollEvent(event)) {
      handleUserEvent(event);
    }

    // Modal timeout check
    if (currentUiMode == UI_MODE_CALIBRATING) {
      if (millis() - calModalOpenedMillis >= CAL_MODAL_TIMEOUT_MS) {
        currentUiMode = UI_MODE_NORMAL;
        inputManager.setUiMode(UiMode::Normal);
        redrawMainScreen();
      }
    }

    // Edit timeout check
    if (isEditing && EDIT_TIMEOUT_MS > 0) {
      if (millis() - lastEditActivity >= EDIT_TIMEOUT_MS) {
        isEditing = false;
        displayUi.renderTargetGrid(remote, remotec, isEditing);
      }
    }
  }
}
