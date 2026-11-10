#include "self_test.h"
#include "app_controller.h"
#include "../config.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include "../protocol/nmea_parser.h"
#include "../ui/rover_card.h"
#include "../ui/rover_history_modal.h"
#include "rover_log_manager.h"
#include <cstring>
#include <cmath>
#include <cstdio>

#if defined(ARDUINO)
#include <Arduino.h>

static void selfTestDelay(AppController &app, unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        app.getGpsManager().update(millis());
        delay(10);
    }
}
#endif

int SelfTest::runAssertions(int *failedTests, int maxFails) {
    int failedCount = 0;

    auto recordFailure = [&](int testNum, const char *desc) {
#if defined(ARDUINO)
        Serial.print(F("FAIL: Test "));
        Serial.print(testNum);
        Serial.print(F(" ("));
        Serial.print(desc);
        Serial.println(F(")"));
#else
        std::printf("FAIL: Test %d (%s)\n", testNum, desc);
#endif
        if (failedTests && failedCount < maxFails) {
            failedTests[failedCount] = testNum;
        }
        failedCount++;
    };

    // Test 1: ARRL 1.0 km minimum rule
    if (std::fabs(calculateArrlContestDistance("CM87wa", "CM87wa") - 1.0) > 0.001) {
        recordFailure(1, "ARRL 1 km rule");
    }

    // Test 2: Mid-range distance (< 1,000 km)
    double lat_wa, lon_wa, lat_dn, lon_dn, d_dn, b_dn;
    gridToGeoPoint("CM87wa", &lat_wa, &lon_wa);
    gridToGeoPoint("DN13ab", &lat_dn, &lon_dn);
    calculateWgs84(lat_wa, lon_wa, lat_dn, lon_dn, d_dn, b_dn);
    int km_dn = (int)std::floor(d_dn / 1000.0);
    if (km_dn < 100 || km_dn >= 1000) {
        recordFailure(2, "Mid-range distance <1000 km formatting");
    }

    // Test 3: Long-range distance (> 1,000 km)
    double lat_el, lon_el, d_el, b_el;
    gridToGeoPoint("EL29ja", &lat_el, &lon_el);
    calculateWgs84(lat_wa, lon_wa, lat_el, lon_el, d_el, b_el);
    int km_el = (int)std::floor(d_el / 1000.0);
    if (km_el < 1000 || km_el >= 10000) {
        recordFailure(3, "Long-range distance >1000 km formatting");
    }

    // Test 4: Extreme distance (> 10,000 km)
    if (calculateArrlContestDistance("CM87wa", "ZA00AA") < 10000.0) {
        recordFailure(4, "Extreme distance recovery");
    }

    // Test 5: Antenna heading geometry & circular wrapping
    if (std::fabs(calcCurrentBearing(0.0f, 10.0f) - 270.0f) > 0.01f ||
        std::fabs(calcCurrentBearing(10.0f, 0.0f) - 0.0f) > 0.01f) {
        recordFailure(5, "Antenna heading geometry");
    }

    // Test 6: Deterministic GNSS stream health evaluator
    if (evaluateGpsStream(10000, 0, 0, 0, 0, 0, false, false, BEARING_NONE) != GPS_LINK_DISCONNECTED ||
        evaluateGpsStream(10000, 5000, 9900, 5, 25, 0, true, true, BEARING_FIXED) != GPS_LINK_OK_FIXED ||
        evaluateGpsStream(10000, 5000, 9900, 5, 25, 0, true, true, BEARING_FLOAT) != GPS_LINK_HEADING_FLOAT ||
        evaluateGpsStream(10000, 2500, 9900, 2, 0, 0, true, false, BEARING_NONE) != GPS_LINK_POSITION_ONLY) {
        recordFailure(6, "GNSS stream health evaluator");
    }

    // Test 7: Heading offset wrap & normalization
    if (std::fabs(normalizeOffset(5.0f - 355.0f) - 10.0f) > 0.01f ||
        std::fabs(normalizeOffset(355.0f - 5.0f) - (-10.0f)) > 0.01f) {
        recordFailure(7, "Antenna offset normalization");
    }

    // Test 8: Rover 4-state qualification matrix & card formatting
    char cardBuf[32];
    if (evaluateRoverState(6.4, false, true) != RoverState::InTransitUnder16k ||
        evaluateRoverState(18.2, false, true) != RoverState::QualifiedNewGrid ||
        evaluateRoverState(18.2, true, true) != RoverState::QualifiedRevisitedGrid ||
        evaluateRoverState(18.2, false, false) != RoverState::GpsLostStale ||
        !formatRoverCard(RoverState::InTransitUnder16k, 6.4, cardBuf, sizeof(cardBuf)) ||
        std::strcmp(cardBuf, "TRANSIT    6.4 k") != 0 ||
        !formatRoverCard(RoverState::QualifiedNewGrid, 18.2, cardBuf, sizeof(cardBuf)) ||
        std::strcmp(cardBuf, "QUALIFIED 18.2 k") != 0 ||
        calculateRoverDistanceColor(6.4, false) != COLOR_ROVER_TRANSIT_RED ||
        calculateRoverDistanceColor(18.2, false) != COLOR_ROVER_QUALIFIED_GREEN ||
        calculateRoverDistanceColor(18.2, true) != COLOR_ROVER_REVISIT_BLUE) {
        recordFailure(8, "Rover qualification matrix & card formatting");
    }

    // Test 9: Rover contest location history stack operations (in-memory)
    RoverLogManager localMgr;
    RoverStop s1(1000, 38.35, -119.45, "CM87wa00", 0.0f, true);
    RoverStop s2(2000, 38.45, -119.35, "CM87wb00", 18.2f, true);
    localMgr.appendStop(s1);
    localMgr.appendStop(s2);
    char modalBuf[48];
    RoverHistoryModal::formatTotalStops(localMgr.getStopCount(), modalBuf, sizeof(modalBuf));
    if (localMgr.getStopCount() != 2 ||
        !localMgr.hasVisitedGrid("CM87wa") ||
        localMgr.hasVisitedGrid("DN13ab") ||
        std::strcmp(modalBuf, "Total Logged Stops: 2") != 0 ||
        !localMgr.popLastStop() ||
        localMgr.getStopCount() != 1) {
        recordFailure(9, "Rover history stack operations");
    }

    return failedCount;
}

void SelfTest::run(AppController &app) {
#if defined(ARDUINO)
    Serial.println(F("\n=== Starting Visual Display Self-Test Demo ==="));

    // Suppress live UI callbacks while self-test runs
    app.setSelfTesting(true);

    // 1. Snapshot live operating state
    char saved_targetGrid[7];
    char saved_myGrid[9];
    std::strncpy(saved_targetGrid, app.getTargetGrid(), sizeof(saved_targetGrid) - 1);
    saved_targetGrid[sizeof(saved_targetGrid) - 1] = '\0';
    std::strncpy(saved_myGrid, app.getMyGrid(), sizeof(saved_myGrid) - 1);
    saved_myGrid[sizeof(saved_myGrid) - 1] = '\0';

    GeoPoint saved_currentPos = app.getCurrentPosition();
    GeoPoint saved_targetPos = app.getTargetPosition();
    bool saved_hasHeading = app.hasHeading();
    OperationalMode saved_operationalMode = app.getOperationalMode();
    bool saved_isEditing = app.getGridEditor().isEditing();
    BearingQuality saved_quality = app.getBearingQuality();
    unsigned long saved_lastHeading = app.getLastHeadingUpdate();
    float savedOffset = app.getCalibration().getOffset();
    UiMode saved_uiMode = app.getUiMode();

    app.getGridEditor().setEditing(false);

    // Stage 1: Same Grid (ARRL 1.0 km minimum rule)
    Serial.println(F("[Test 1/9] Same Grid (ARRL 1.0 km rule)"));
    app.getDisplayUi().showFooterText("TEST 1: SAME GRID", CYAN);
    app.setMyGrid("CM87wa00");
    app.setTargetGrid("CM87wa");
    GeoPoint pos1;
    gridToGeoPoint("CM87wa", pos1);
    app.setCurrentPosition(pos1);
    app.setHeadingState(true, BEARING_FIXED);
    app.getGpsManager().getBearingFilter().seed(180.0f);
    app.setOperationalMode(OperationalMode::Bearing);
    app.getDisplayUi().renderMyGrid(app.getMyGrid());
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    selfTestDelay(app, 2500);

    // Stage 2: Mid-range distance (< 1,000 km, ~600 km)
    Serial.println(F("[Test 2/9] Under 1,000 km formatting (~600 km)"));
    app.getDisplayUi().showFooterText("TEST 2: DIST < 1000K", CYAN);
    app.setTargetGrid("DN13ab");
    app.setHeadingState(true, BEARING_FIXED);
    app.setOperationalMode(OperationalMode::Bearing);
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    selfTestDelay(app, 2500);

    // Stage 3: Long-range distance (> 1,000 km, ~2,500 km)
    Serial.println(F("[Test 3/9] Over 1,000 km formatting (~2,500 km)"));
    app.getDisplayUi().showFooterText("TEST 3: DIST > 1000K", CYAN);
    app.setTargetGrid("EL29ja");
    app.setHeadingState(true, BEARING_FIXED);
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    selfTestDelay(app, 2500);

    // Stage 4: Extreme distance (> 10,000 km) and Recovery to 1.0 km
    Serial.println(F("[Test 4/9] Extreme distance (> 10,000 km) & Ghost Recovery"));
    app.getDisplayUi().showFooterText("TEST 4: GHOST RECOVERY", CYAN);
    app.setTargetGrid("ZA00AA");
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    selfTestDelay(app, 1500);

    Serial.println(F("  -> Dropping back to 1.0 km (verifying no ghost characters)"));
    app.setTargetGrid("CM87wa");
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    selfTestDelay(app, 1500);

    // Stage 5: Antenna Heading Quality Color States
    Serial.println(F("[Test 5/9] Antenna Heading Quality Color States"));
    Serial.println(F("  -> Bearing Color: Pure White (RTK Fixed)"));
    app.getDisplayUi().showFooterText("TEST 5: BEARING WHITE", WHITE);
    app.setHeadingState(true, BEARING_FIXED);
    app.drawCurrentBearing();
    selfTestDelay(app, 1000);

    Serial.println(F("  -> Bearing Color: Bright Yellow (RTK Float / Degraded)"));
    app.getDisplayUi().showFooterText("TEST 5: BEARING YELLOW", RICH_YELLOW);
    app.setHeadingState(true, BEARING_FLOAT);
    app.drawCurrentBearing();
    selfTestDelay(app, 1000);

    Serial.println(F("  -> Bearing Color: Bright Red (Lost / Stale)"));
    app.getDisplayUi().showFooterText("TEST 5: BEARING RED", RED);
    app.setHeadingState(true, BEARING_LOST);
    app.drawCurrentBearing();
    selfTestDelay(app, 1000);

    // Restore bearing to fixed
    app.setHeadingState(true, BEARING_FIXED);
    app.drawCurrentBearing();

    // Stage 6: Live GNSS Receiver Stream Diagnostic
    Serial.println(F("[Test 6/9] Live GNSS Receiver Stream Diagnostic"));
    app.getDisplayUi().showFooterText("TEST 6: GPS STREAM", CYAN);

    // Ensure serial port is pumped and up to date
    app.getGpsManager().update(millis());
    app.getSerialCli().printGpsStatus();

    GpsLinkVerdict liveVerdict = app.getGpsManager().evaluateStreamHealth(millis());
    if (liveVerdict == GPS_LINK_OK_FIXED) {
        app.getDisplayUi().showStatusCard("GPS: RTK FIXED (OK)", GREEN, BLACK);
    } else if (liveVerdict == GPS_LINK_HEADING_FLOAT) {
        app.getDisplayUi().showStatusCard("GPS: RTK FLOAT (WARN)", RICH_YELLOW, BLACK);
    } else if (liveVerdict == GPS_LINK_ACQUIRING || liveVerdict == GPS_LINK_POSITION_ONLY) {
        app.getDisplayUi().showStatusCard("GPS: ACQUIRING (WAIT)", RICH_YELLOW, BLACK);
    } else {
        app.getDisplayUi().showStatusCard("GPS: NO DATA / CABLE", RED, WHITE);
    }
    selfTestDelay(app, 2500);
    app.getDisplayUi().getGfx()->fillRect(40, 215, 400, 52, BLACK);

    // Stage 7: Antenna Calibration Status & Modal Demo
    Serial.println(F("[Test 7/9] Antenna Calibration Pill & Modal Display Demo"));
    Serial.println(F("  -> Testing Calibration Pill Display (Uncalibrated: OFF)"));
    app.getDisplayUi().showFooterText("TEST 7: CAL PILL OFF", CYAN);
    app.getCalibration().setOffset(0.0f, false);
    app.getDisplayUi().renderCalibrationPill(app.getCalibration().getOffset(), app.getCalibration().isCalibrated());
    app.drawCurrentBearing();
    selfTestDelay(app, 1200);

    Serial.println(F("  -> Testing Calibration Pill Display (Calibrated: +4.2 deg)"));
    app.getDisplayUi().showFooterText("TEST 7: CAL PILL +4.2", GREEN);
    app.getCalibration().setOffset(4.2f, false);
    app.getDisplayUi().renderCalibrationPill(app.getCalibration().getOffset(), app.getCalibration().isCalibrated());
    app.drawCurrentBearing();
    selfTestDelay(app, 1200);

    Serial.println(F("  -> Demonstrating Calibration Modal Dialog"));
    app.getDisplayUi().showFooterText("TEST 7: CAL MODAL", RICH_YELLOW);
    app.getCalibration().stageProposedOffset(4.2f);
    app.setUiMode(UiMode::Calibrating);
    app.getDisplayUi().getCalModal().drawModal(app.getBearing(), app.getGpsManager().getBearingFilter().getRawHeading(),
                                              app.hasHeading(), app.getCalibration().getOffset(),
                                              app.getCalibration().getProposedOffset());
    selfTestDelay(app, 1000);
    app.getDisplayUi().getCalModal().drawSnapButton(true, app.hasHeading());
    selfTestDelay(app, 300);
    app.getDisplayUi().getCalModal().drawSnapButton(false, app.hasHeading());
    selfTestDelay(app, 1000);

    // Close calibration modal and restore normal screen before Stage 8
    app.setUiMode(UiMode::Normal);
    app.redrawMainScreen();

    // Stage 8: Rover Mode 4-State Qualification Matrix (Row 5 Card)
    Serial.println(F("[Test 8/9] Rover Mode 4-State Qualification Matrix (Row 5 Card)"));
    app.setOperationalMode(OperationalMode::Rover);

    // State A: In Transit Under 16 km (Red on White)
    Serial.println(F("  -> State A: In Transit (< 16.0 km, Red on White)"));
    app.getDisplayUi().showFooterText("TEST 8: ROVER TRANSIT", RED);
    app.getDisplayUi().renderRoverCard(RoverState::InTransitUnder16k, 6.4, false);
    selfTestDelay(app, 1500);

    // State B: Qualified New Grid (>= 16.0 km, Forest Green on White)
    Serial.println(F("  -> State B: Qualified New Grid (>= 16.0 km, Green on White)"));
    app.getDisplayUi().showFooterText("TEST 8: ROVER QUALIFIED", GREEN);
    app.getDisplayUi().renderRoverCard(RoverState::QualifiedNewGrid, 18.2, false);
    selfTestDelay(app, 1500);

    // State C: Qualified Revisited Grid (>= 16.0 km, Royal Blue on White)
    Serial.println(F("  -> State C: Qualified Revisited Grid (>= 16.0 km, Blue on White)"));
    app.getDisplayUi().showFooterText("TEST 8: ROVER REVISIT", COLOR_ROVER_REVISIT_BLUE);
    app.getDisplayUi().renderRoverCard(RoverState::QualifiedRevisitedGrid, 18.2, true);
    selfTestDelay(app, 1500);

    // State D: GPS Lost / Stale (Amber status on White, retaining qualified distance color)
    Serial.println(F("  -> State D: GPS Lost / Stale (Amber status on White)"));
    app.getDisplayUi().showFooterText("TEST 8: ROVER GPS LOST", RICH_YELLOW);
    app.getDisplayUi().renderRoverCard(RoverState::GpsLostStale, 18.2, false);
    selfTestDelay(app, 1500);

    app.getDisplayUi().clearRoverCard();

    // Stage 9: Rover Contest Location History & 1-Touch Undo Modal
    Serial.println(F("[Test 9/9] Rover Contest Location History & 1-Touch Undo Modal"));
    app.getDisplayUi().showFooterText("TEST 9: ROVER MODAL", RICH_YELLOW);
    app.setUiMode(UiMode::RoverHistory);

    RoverStop mockStop(1716999000, 38.35412, -119.45821, "CM87wb22", 18.2f, true);
    app.getDisplayUi().getRoverHistoryModal().drawModal(&mockStop, 3, 3);
    selfTestDelay(app, 1200);

    // Demonstrate discard button long-press progress animation
    app.getDisplayUi().getRoverHistoryModal().drawDiscardButton(true, 0.33f);
    selfTestDelay(app, 400);
    app.getDisplayUi().getRoverHistoryModal().drawDiscardButton(true, 0.66f);
    selfTestDelay(app, 400);
    app.getDisplayUi().getRoverHistoryModal().drawDiscardButton(true, 1.0f);
    selfTestDelay(app, 400);
    app.getDisplayUi().getRoverHistoryModal().drawDiscardButton(false, 0.0f);
    selfTestDelay(app, 800);

    // Restore UI mode before result banner
    app.setUiMode(UiMode::Normal);
    app.redrawMainScreen();

    // Display PASS/FAIL Result Banner
    int failedTests[16];
    int failedCount = runAssertions(failedTests, sizeof(failedTests) / sizeof(failedTests[0]));

    if (failedCount == 0) {
        Serial.println(F("Result: PASS"));
        app.getDisplayUi().showStatusCard("SELF-TEST PASSED", GREEN, BLACK);
        selfTestDelay(app, 2500);
    } else {
        char failMsg[32];
        DisplayUi::formatSelfTestFailure(failedTests, failedCount, failMsg, sizeof(failMsg));
        Serial.print(F("Result: FAIL -> "));
        Serial.println(failMsg);
        app.getDisplayUi().showStatusCard(failMsg, RED, WHITE);
        selfTestDelay(app, 4000);
    }
    app.getDisplayUi().getGfx()->fillRect(40, 215, 400, 52, BLACK);

    // Restore live state and re-enable UI callbacks
    app.setSelfTesting(false);
    app.setTargetGrid(saved_targetGrid);
    app.setMyGrid(saved_myGrid);
    app.setCurrentPosition(saved_currentPos);
    app.setTargetPosition(saved_targetPos);
    app.setHeadingState(saved_hasHeading, saved_quality, saved_lastHeading);
    app.setOperationalMode(saved_operationalMode);
    app.getGridEditor().setEditing(saved_isEditing);
    app.getCalibration().setOffset(savedOffset, false);
    app.setUiMode(saved_uiMode);
    app.updateNavigation();
    app.redrawMainScreen();

    Serial.println(F("=== Visual Self-Test Complete (Live state restored) ===\n"));
#else
    (void)app;
#endif
}
