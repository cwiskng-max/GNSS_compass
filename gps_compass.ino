#include <Wire.h>
#include <Preferences.h>
#include <math.h>

#include "src/config.h"
#include "src/types.h"
#include "src/geo/geo_math.h"
#include "src/geo/maidenhead.h"
#include "src/protocol/serial_cli.h"
#include "src/system/calibration.h"
#include "src/system/self_test.h"
#include "src/gnss/simulator.h"
#include "src/gnss/gps_manager.h"
#include "src/ui/input_manager.h"
#include "src/ui/display_ui.h"

// Active Station & Target State
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

// UI & Editing State
UiMode currentUiMode = UiMode::Normal;
unsigned long calModalOpenedMillis = 0;
int remotec = 0;
bool isEditing = true;
unsigned long lastEditActivity = 0;

// Heading & Geodesic Calculation State
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
  if (currentUiMode == UiMode::Calibrating) {
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

  if (currentUiMode == UiMode::Calibrating) {
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

void runVisualSelfTest() {
  SelfTestContext ctx{
    displayUi, gpsManager, calibration, serialCli, inputManager,
    remote, mygrid, mylatitude, mylongitude, remlatitude, remlongitude,
    distance, bearing, revbearing, hasHeading, currentBearingQuality,
    lastHeadingUpdate, rovdist, isEditing, remotec, currentUiMode,
    hours, minutes, hasTime,
    redrawMainScreen, updateBearings, drawCurrentBearing
  };
  SelfTest::run(ctx);
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

    if (currentUiMode != UiMode::Calibrating) {
      displayUi.renderMyGrid(mygrid);
    }
    updateBearings();
  });

  gpsManager.setOnTimeUpdated([](int h, int m) {
    hasTime = true;
    hours = h;
    minutes = m;
    if (currentUiMode != UiMode::Calibrating) {
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
      if (currentUiMode != UiMode::Calibrating) {
        displayUi.renderMyGrid(mygrid);
      }
    }
    if (gpsManager.hasTime()) {
      hasTime = true;
      hours = gpsManager.getHours();
      minutes = gpsManager.getMinutes();
      if (currentUiMode != UiMode::Calibrating) {
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
      if (currentUiMode == UiMode::Calibrating) {
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
      if (currentUiMode == UiMode::Calibrating) {
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
      currentUiMode = UiMode::Calibrating;
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
      currentUiMode = UiMode::Normal;
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
      currentUiMode = UiMode::Normal;
      inputManager.setUiMode(UiMode::Normal);
      redrawMainScreen();
      break;

    default:
      break;
  }
}

void loop() {
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

  // 4. Modal timeout check
  if (currentUiMode == UiMode::Calibrating) {
    if (millis() - calModalOpenedMillis >= CAL_MODAL_TIMEOUT_MS) {
      currentUiMode = UiMode::Normal;
      inputManager.setUiMode(UiMode::Normal);
      redrawMainScreen();
    }
  }

  // 5. Edit timeout check
  if (isEditing && EDIT_TIMEOUT_MS > 0) {
    if (millis() - lastEditActivity >= EDIT_TIMEOUT_MS) {
      isEditing = false;
      displayUi.renderTargetGrid(remote, remotec, isEditing);
    }
  }
}
