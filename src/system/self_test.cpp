#include "self_test.h"
#include "app_controller.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include <cstring>
#include <cmath>
#include <cstdio>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

void SelfTest::run(AppController &app) {
#if defined(ARDUINO)
    Serial.println(F("\n=== Starting Visual Display Self-Test Demo ==="));

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
    bool saved_isRoverAlertActive = app.isRoverAlertActive();
    bool saved_isEditing = app.getGridEditor().isEditing();
    BearingQuality saved_quality = app.getBearingQuality();
    unsigned long saved_lastHeading = app.getLastHeadingUpdate();
    float savedOffset = app.getCalibration().getOffset();
    UiMode saved_uiMode = app.getUiMode();

    app.getGridEditor().setEditing(false);

    // Test 1: Same Grid (ARRL 1.0 km minimum rule)
    Serial.println(F("[Test 1/7] Same Grid (ARRL 1.0 km rule)"));
    app.getDisplayUi().showFooterText("TEST 1: SAME GRID", CYAN);
    app.setMyGrid("CM87wa00");
    app.setTargetGrid("CM87wa");
    GeoPoint pos1;
    gridToGeoPoint("CM87wa", pos1);
    app.setCurrentPosition(pos1);
    app.setHeadingState(true, BEARING_FIXED);
    app.getGpsManager().getBearingFilter().seed(180.0f);
    app.setRoverAlertActive(false);
    app.getDisplayUi().renderMyGrid(app.getMyGrid());
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    delay(3000);

    // Test 2: Rover Transit Alert (< 16 km, highlighted RED on WHITE)
    Serial.println(F("[Test 2/7] Rover Transit Alert (< 16 km, highlighted RED on WHITE)"));
    app.getDisplayUi().showFooterText("TEST 2: ROVER < 16K", RICH_YELLOW);
    app.setTargetGrid("CM87wb");
    app.setHeadingState(true, BEARING_FIXED);
    app.setRoverAlertActive(true);
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    delay(3000);

    // Test 3: Distance Under 1,000 km (~600 km)
    Serial.println(F("[Test 3/7] Under 1,000 km formatting (~600 km)"));
    app.getDisplayUi().showFooterText("TEST 3: DIST < 1000K", CYAN);
    app.setTargetGrid("DN13ab");
    app.setHeadingState(true, BEARING_FIXED);
    app.setRoverAlertActive(false);
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    delay(3000);

    // Test 4: Distance Over 1,000 km (~2,500 km)
    Serial.println(F("[Test 4/7] Over 1,000 km formatting (~2,500 km)"));
    app.getDisplayUi().showFooterText("TEST 4: DIST > 1000K", CYAN);
    app.setTargetGrid("EL29ja");
    app.setHeadingState(true, BEARING_FIXED);
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    delay(3000);

    // Test 5: Extreme Distance (> 10,000 km) and Recovery to 1.0 km
    Serial.println(F("[Test 5/7] Extreme distance (> 10,000 km)"));
    app.getDisplayUi().showFooterText("TEST 5: GHOST RECOVERY", CYAN);
    app.setTargetGrid("ZA00AA");
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    delay(2000);

    Serial.println(F("  -> Dropping back to 1.0 km (verifying no ghost characters)"));
    app.setTargetGrid("CM87wa");
    app.getDisplayUi().renderTargetGrid(app.getTargetGrid(), app.getGridEditor().getCursorColumn(), app.getGridEditor().isEditing());
    app.updateNavigation();
    delay(2000);

    // Test 6: Bearing Color States & Live GPS Receiver Link Diagnostic
    Serial.println(F("[Test 6/7] Bearing Color States & Live GPS Stream Diagnostic"));
    Serial.println(F("  -> Bearing Color: Pure White (RTK Fixed)"));
    app.getDisplayUi().showFooterText("TEST 6: BEARING WHITE", WHITE);
    app.setHeadingState(true, BEARING_FIXED);
    app.drawCurrentBearing();
    delay(1000);

    Serial.println(F("  -> Bearing Color: Bright Yellow (RTK Float / Degraded)"));
    app.getDisplayUi().showFooterText("TEST 6: BEARING YELLOW", RICH_YELLOW);
    app.setHeadingState(true, BEARING_FLOAT);
    app.drawCurrentBearing();
    delay(1000);

    Serial.println(F("  -> Bearing Color: Bright Red (Lost / Stale)"));
    app.getDisplayUi().showFooterText("TEST 6: BEARING RED", RED);
    app.setHeadingState(true, BEARING_LOST);
    app.drawCurrentBearing();
    delay(1000);

    // Restore bearing to fixed before showing live GPS stream ruling
    app.setHeadingState(true, BEARING_FIXED);
    app.drawCurrentBearing();
    app.getDisplayUi().showFooterText("TEST 6: GPS STREAM", CYAN);
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
    delay(2500);
    app.getDisplayUi().getGfx()->fillRect(40, 215, 400, 52, BLACK);

    // Test 7: Antenna Calibration Status & Modal Demo
    Serial.println(F("[Test 7/7] Antenna Calibration Pill & Modal Display Demo"));
    Serial.println(F("  -> Testing Calibration Pill Display (Uncalibrated: OFF)"));
    app.getDisplayUi().showFooterText("TEST 7: CAL PILL OFF", CYAN);
    app.getCalibration().setOffset(0.0f, false);
    app.getDisplayUi().renderCalibrationPill(app.getCalibration().getOffset(), app.getCalibration().isCalibrated());
    app.drawCurrentBearing();
    delay(1500);

    Serial.println(F("  -> Testing Calibration Pill Display (Calibrated: +4.2 deg)"));
    app.getDisplayUi().showFooterText("TEST 7: CAL PILL +4.2", GREEN);
    app.getCalibration().setOffset(4.2f, false);
    app.getDisplayUi().renderCalibrationPill(app.getCalibration().getOffset(), app.getCalibration().isCalibrated());
    app.drawCurrentBearing();
    delay(1500);

    Serial.println(F("  -> Demonstrating Calibration Modal Dialog"));
    app.getDisplayUi().showFooterText("TEST 7: CAL MODAL", RICH_YELLOW);
    app.getCalibration().stageProposedOffset(4.2f);
    app.setUiMode(UiMode::Calibrating);
    app.getDisplayUi().getCalModal().drawModal(app.getBearing(), app.getGpsManager().getBearingFilter().getRawHeading(),
                                          app.hasHeading(), app.getCalibration().getOffset(),
                                          app.getCalibration().getProposedOffset());
    delay(1200);
    app.getDisplayUi().getCalModal().drawSnapButton(true, app.hasHeading());
    delay(300);
    app.getDisplayUi().getCalModal().drawSnapButton(false, app.hasHeading());
    delay(1000);

    // Display PASS/FAIL Result Banner
    int failedTests[8];
    int failedCount = 0;

    // Test 1: ARRL 1.0 km minimum rule
    if (std::fabs(calculateArrlContestDistance("CM87wa", "CM87wa") - 1.0) > 0.001) {
        Serial.println(F("FAIL: Test 1 (ARRL 1 km rule)"));
        failedTests[failedCount++] = 1;
    }

    // Test 2: Rover transit alert threshold (< 16 km)
    double lat_wa, lon_wa, lat_wb, lon_wb, rover_dist, rover_bear;
    gridToGeoPoint("CM87wa", &lat_wa, &lon_wa);
    gridToGeoPoint("CM87wb", &lat_wb, &lon_wb);
    calculateWgs84(lat_wa, lon_wa, lat_wb, lon_wb, rover_dist, rover_bear);
    if ((int)std::floor(rover_dist / 1000.0) >= 16) {
        Serial.println(F("FAIL: Test 2 (Rover transit <16 km threshold)"));
        failedTests[failedCount++] = 2;
    }

    // Test 3: Mid-range distance formatting (< 1,000 km)
    double lat_dn, lon_dn, d_dn, b_dn;
    gridToGeoPoint("DN13ab", &lat_dn, &lon_dn);
    calculateWgs84(lat_wa, lon_wa, lat_dn, lon_dn, d_dn, b_dn);
    int km_dn = (int)std::floor(d_dn / 1000.0);
    if (km_dn < 100 || km_dn >= 1000) {
        Serial.println(F("FAIL: Test 3 (Mid-range distance <1000 km formatting)"));
        failedTests[failedCount++] = 3;
    }

    // Test 4: Long-range distance formatting (> 1,000 km)
    double lat_el, lon_el, d_el, b_el;
    gridToGeoPoint("EL29ja", &lat_el, &lon_el);
    calculateWgs84(lat_wa, lon_wa, lat_el, lon_el, d_el, b_el);
    int km_el = (int)std::floor(d_el / 1000.0);
    if (km_el < 1000 || km_el >= 10000) {
        Serial.println(F("FAIL: Test 4 (Long-range distance >1000 km formatting)"));
        failedTests[failedCount++] = 4;
    }

    // Test 5: Extreme distance (> 10,000 km)
    if (calculateArrlContestDistance("CM87wa", "ZA00AA") < 10000.0) {
        Serial.println(F("FAIL: Test 5 (Extreme distance recovery)"));
        failedTests[failedCount++] = 5;
    }

    // Test 6: Antenna baseline offset calculation
    if (std::fabs(calcCurrentBearing(0.0f, 10.0f) - 270.0f) > 0.01f ||
        std::fabs(calcCurrentBearing(10.0f, 0.0f) - 0.0f) > 0.01f) {
        Serial.println(F("FAIL: Test 6 (Antenna baseline offset calculation)"));
        failedTests[failedCount++] = 6;
    }

    // Test 7: Heading offset wrap & normalization
    if (std::fabs(normalizeOffset(5.0f - 355.0f) - 10.0f) > 0.01f ||
        std::fabs(normalizeOffset(355.0f - 5.0f) - (-10.0f)) > 0.01f) {
        Serial.println(F("FAIL: Test 7 (Antenna offset normalization)"));
        failedTests[failedCount++] = 7;
    }

    if (failedCount == 0) {
        Serial.println(F("Result: PASS"));
        app.getDisplayUi().showStatusCard("SELF-TEST PASSED", GREEN, BLACK);
        delay(2500);
    } else {
        char failMsg[32];
        DisplayUi::formatSelfTestFailure(failedTests, failedCount, failMsg, sizeof(failMsg));
        Serial.print(F("Result: FAIL -> "));
        Serial.println(failMsg);
        app.getDisplayUi().showStatusCard(failMsg, RED, WHITE);
        delay(4000);
    }
    app.getDisplayUi().getGfx()->fillRect(40, 215, 400, 52, BLACK);

    // Restore live state and firmware version footer
    app.setTargetGrid(saved_targetGrid);
    app.setMyGrid(saved_myGrid);
    app.setCurrentPosition(saved_currentPos);
    app.setTargetPosition(saved_targetPos);
    app.setHeadingState(saved_hasHeading, saved_quality, saved_lastHeading);
    app.setRoverAlertActive(saved_isRoverAlertActive);
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
