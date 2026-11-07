#include "self_test.h"
#include "../geo/geo_math.h"
#include "../geo/maidenhead.h"
#include <cstring>
#include <cmath>
#include <cstdio>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

void SelfTest::run(SelfTestContext &ctx) {
#if defined(ARDUINO)
    Serial.println(F("\n=== Starting Visual Display Self-Test Demo ==="));

    // 1. Snapshot live operating state
    char saved_targetGrid[7];
    char saved_myGrid[9];
    memcpy(saved_targetGrid, ctx.targetGrid, sizeof(saved_targetGrid));
    memcpy(saved_myGrid, ctx.myGrid, sizeof(saved_myGrid));
    GeoPoint saved_currentPos = ctx.currentPosition;
    GeoPoint saved_targetPos = ctx.targetPosition;
    bool saved_hasHeading = ctx.hasHeading;
    bool saved_isRoverAlertActive = ctx.isRoverAlertActive;
    bool saved_isEditing = ctx.isEditing;
    BearingQuality saved_quality = ctx.currentBearingQuality;
    unsigned long saved_lastHeading = ctx.lastHeadingUpdate;
    float savedOffset = ctx.calibration.getOffset();
    UiMode saved_uiMode = ctx.currentUiMode;
    ctx.isEditing = false;

    // Test 1: Same Grid (ARRL 1.0 km minimum rule)
    Serial.println(F("[Test 1/7] Same Grid (ARRL 1.0 km rule)"));
    ctx.displayUi.showFooterText("TEST 1: SAME GRID", CYAN);
    strcpy(ctx.myGrid, "CM87wa00");
    strcpy(ctx.targetGrid, "CM87wa");
    gridToGeoPoint("CM87wa", ctx.currentPosition);
    ctx.hasHeading = true;
    ctx.currentBearingQuality = BEARING_FIXED;
    ctx.gpsManager.getBearingFilter().seed(180.0f);
    ctx.isRoverAlertActive = false;
    ctx.displayUi.renderMyGrid(ctx.myGrid);
    ctx.displayUi.renderTargetGrid(ctx.targetGrid, ctx.cursorColumn, ctx.isEditing);
    if (ctx.updateBearings) ctx.updateBearings();
    delay(3000);

    // Test 2: Rover Transit Alert (< 16 km, highlighted RED on WHITE)
    Serial.println(F("[Test 2/7] Rover Transit Alert (< 16 km, highlighted RED on WHITE)"));
    ctx.displayUi.showFooterText("TEST 2: ROVER < 16K", RICH_YELLOW);
    strcpy(ctx.targetGrid, "CM87wb");
    ctx.currentBearingQuality = BEARING_FIXED;
    ctx.isRoverAlertActive = true;
    ctx.displayUi.renderTargetGrid(ctx.targetGrid, ctx.cursorColumn, ctx.isEditing);
    if (ctx.updateBearings) ctx.updateBearings();
    delay(3000);

    // Test 3: Distance Under 1,000 km (~600 km)
    Serial.println(F("[Test 3/7] Under 1,000 km formatting (~600 km)"));
    ctx.displayUi.showFooterText("TEST 3: DIST < 1000K", CYAN);
    strcpy(ctx.targetGrid, "DN13ab");
    ctx.currentBearingQuality = BEARING_FIXED;
    ctx.isRoverAlertActive = false;
    ctx.displayUi.renderTargetGrid(ctx.targetGrid, ctx.cursorColumn, ctx.isEditing);
    if (ctx.updateBearings) ctx.updateBearings();
    delay(3000);

    // Test 4: Distance Over 1,000 km (~2,500 km)
    Serial.println(F("[Test 4/7] Over 1,000 km formatting (~2,500 km)"));
    ctx.displayUi.showFooterText("TEST 4: DIST > 1000K", CYAN);
    strcpy(ctx.targetGrid, "EL29ja");
    ctx.currentBearingQuality = BEARING_FIXED;
    ctx.displayUi.renderTargetGrid(ctx.targetGrid, ctx.cursorColumn, ctx.isEditing);
    if (ctx.updateBearings) ctx.updateBearings();
    delay(3000);

    // Test 5: Extreme Distance (> 10,000 km) and Recovery to 1.0 km
    Serial.println(F("[Test 5/7] Extreme distance (> 10,000 km)"));
    ctx.displayUi.showFooterText("TEST 5: GHOST RECOVERY", CYAN);
    strcpy(ctx.targetGrid, "ZA00AA");
    ctx.displayUi.renderTargetGrid(ctx.targetGrid, ctx.cursorColumn, ctx.isEditing);
    if (ctx.updateBearings) ctx.updateBearings();
    delay(2000);

    Serial.println(F("  -> Dropping back to 1.0 km (verifying no ghost characters)"));
    strcpy(ctx.targetGrid, "CM87wa");
    ctx.displayUi.renderTargetGrid(ctx.targetGrid, ctx.cursorColumn, ctx.isEditing);
    if (ctx.updateBearings) ctx.updateBearings();
    delay(2000);

    // Test 6: Bearing Color States & Live GPS Receiver Link Diagnostic
    Serial.println(F("[Test 6/7] Bearing Color States & Live GPS Stream Diagnostic"));
    Serial.println(F("  -> Bearing Color: Pure White (RTK Fixed)"));
    ctx.displayUi.showFooterText("TEST 6: BEARING WHITE", WHITE);
    ctx.currentBearingQuality = BEARING_FIXED;
    if (ctx.drawCurrentBearing) ctx.drawCurrentBearing();
    delay(1000);

    Serial.println(F("  -> Bearing Color: Bright Yellow (RTK Float / Degraded)"));
    ctx.displayUi.showFooterText("TEST 6: BEARING YELLOW", RICH_YELLOW);
    ctx.currentBearingQuality = BEARING_FLOAT;
    if (ctx.drawCurrentBearing) ctx.drawCurrentBearing();
    delay(1000);

    Serial.println(F("  -> Bearing Color: Bright Red (Lost / Stale)"));
    ctx.displayUi.showFooterText("TEST 6: BEARING RED", RED);
    ctx.currentBearingQuality = BEARING_LOST;
    if (ctx.drawCurrentBearing) ctx.drawCurrentBearing();
    delay(1000);

    // Restore bearing to fixed before showing live GPS stream ruling
    ctx.currentBearingQuality = BEARING_FIXED;
    if (ctx.drawCurrentBearing) ctx.drawCurrentBearing();
    ctx.displayUi.showFooterText("TEST 6: GPS STREAM", CYAN);
    ctx.serialCli.printGpsStatus();

    GpsLinkVerdict liveVerdict = ctx.gpsManager.evaluateStreamHealth(millis());
    if (liveVerdict == GPS_LINK_OK_FIXED) {
        ctx.displayUi.showStatusCard("GPS: RTK FIXED (OK)", GREEN, BLACK);
    } else if (liveVerdict == GPS_LINK_HEADING_FLOAT) {
        ctx.displayUi.showStatusCard("GPS: RTK FLOAT (WARN)", RICH_YELLOW, BLACK);
    } else if (liveVerdict == GPS_LINK_ACQUIRING || liveVerdict == GPS_LINK_POSITION_ONLY) {
        ctx.displayUi.showStatusCard("GPS: ACQUIRING (WAIT)", RICH_YELLOW, BLACK);
    } else {
        ctx.displayUi.showStatusCard("GPS: NO DATA / CABLE", RED, WHITE);
    }
    delay(2500);
    ctx.displayUi.getGfx()->fillRect(40, 215, 400, 52, BLACK);

    // Test 7: Antenna Calibration Status & Modal Demo
    Serial.println(F("[Test 7/7] Antenna Calibration Pill & Modal Display Demo"));
    Serial.println(F("  -> Testing Calibration Pill Display (Uncalibrated: OFF)"));
    ctx.displayUi.showFooterText("TEST 7: CAL PILL OFF", CYAN);
    ctx.calibration.setOffset(0.0f, false);
    ctx.displayUi.renderCalibrationPill(ctx.calibration.getOffset(), ctx.calibration.isCalibrated());
    if (ctx.drawCurrentBearing) ctx.drawCurrentBearing();
    delay(1500);

    Serial.println(F("  -> Testing Calibration Pill Display (Calibrated: +4.2 deg)"));
    ctx.displayUi.showFooterText("TEST 7: CAL PILL +4.2", GREEN);
    ctx.calibration.setOffset(4.2f, false);
    ctx.displayUi.renderCalibrationPill(ctx.calibration.getOffset(), ctx.calibration.isCalibrated());
    if (ctx.drawCurrentBearing) ctx.drawCurrentBearing();
    delay(1500);

    Serial.println(F("  -> Demonstrating Calibration Modal Dialog"));
    ctx.displayUi.showFooterText("TEST 7: CAL MODAL", RICH_YELLOW);
    ctx.calibration.stageProposedOffset(4.2f);
    ctx.currentUiMode = UiMode::Calibrating;
    ctx.inputManager.setUiMode(UiMode::Calibrating);
    ctx.displayUi.getCalModal().drawModal(ctx.bearing, ctx.gpsManager.getBearingFilter().getRawHeading(),
                                          ctx.hasHeading, ctx.calibration.getOffset(),
                                          ctx.calibration.getProposedOffset());
    delay(1200);
    ctx.displayUi.getCalModal().drawSnapButton(true, ctx.hasHeading);
    delay(300);
    ctx.displayUi.getCalModal().drawSnapButton(false, ctx.hasHeading);
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

    // Test 6: Antenna baseline offset calculation
    if (fabs(calcCurrentBearing(0.0f, 10.0f) - 270.0f) > 0.01f ||
        fabs(calcCurrentBearing(10.0f, 0.0f) - 0.0f) > 0.01f) {
        Serial.println(F("FAIL: Test 6 (Antenna baseline offset calculation)"));
        failedTests[failedCount++] = 6;
    }

    // Test 7: Heading offset wrap & normalization
    if (fabs(normalizeOffset(5.0f - 355.0f) - 10.0f) > 0.01f ||
        fabs(normalizeOffset(355.0f - 5.0f) - (-10.0f)) > 0.01f) {
        Serial.println(F("FAIL: Test 7 (Antenna offset normalization)"));
        failedTests[failedCount++] = 7;
    }

    if (failedCount == 0) {
        Serial.println(F("Result: PASS"));
        ctx.displayUi.showStatusCard("SELF-TEST PASSED", GREEN, BLACK);
        delay(2500);
    } else {
        char failMsg[32];
        DisplayUi::formatSelfTestFailure(failedTests, failedCount, failMsg, sizeof(failMsg));
        Serial.print(F("Result: FAIL -> "));
        Serial.println(failMsg);
        ctx.displayUi.showStatusCard(failMsg, RED, WHITE);
        delay(4000);
    }
    ctx.displayUi.getGfx()->fillRect(40, 215, 400, 52, BLACK);

    // Restore live state and firmware version footer
    memcpy(ctx.targetGrid, saved_targetGrid, sizeof(saved_targetGrid));
    memcpy(ctx.myGrid, saved_myGrid, sizeof(saved_myGrid));
    ctx.currentPosition = saved_currentPos;
    ctx.targetPosition = saved_targetPos;
    ctx.hasHeading = saved_hasHeading;
    ctx.currentBearingQuality = saved_quality;
    ctx.lastHeadingUpdate = saved_lastHeading;
    ctx.isRoverAlertActive = saved_isRoverAlertActive;
    ctx.isEditing = saved_isEditing;
    ctx.calibration.setOffset(savedOffset, false);
    ctx.currentUiMode = saved_uiMode;
    ctx.inputManager.setUiMode(saved_uiMode);
    if (ctx.updateBearings) ctx.updateBearings();
    if (ctx.redrawMainScreen) ctx.redrawMainScreen();

    Serial.println(F("=== Visual Self-Test Complete (Live state restored) ===\n"));
#else
    (void)ctx;
#endif
}

