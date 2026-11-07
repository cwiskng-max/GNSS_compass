#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
#include "../src/types.h"
#include "../src/config.h"
#include "../src/geo/geo_math.h"
#include "../src/geo/maidenhead.h"
#include "../src/protocol/nmea_parser.h"
#include "../src/system/calibration.h"
#include "../src/gnss/simulator.h"
#include "../src/gnss/gps_manager.h"
#include "../src/protocol/serial_cli.h"
#include "../src/ui/input_manager.h"
#include "../src/ui/cal_modal.h"
#include "../src/ui/display_ui.h"

int tests_run = 0;
int tests_passed = 0;

#define TEST(name) \
    std::cout << "[TEST] " << name << "... "; \
    tests_run++;

#define PASS() \
    std::cout << "PASS" << std::endl; \
    tests_passed++;

#define FAIL(msg) \
    do { \
        std::cerr << "FAIL: " << msg << " (line " << __LINE__ << ")" << std::endl; \
        exit(1); \
    } while(0)

#define ASSERT_TRUE(cond) if (!(cond)) { FAIL(#cond); }
#define ASSERT_NEAR(a, b, eps) if (std::abs((a) - (b)) > (eps)) { FAIL(#a " not near " #b); }
#define ASSERT_STR_EQ(a, b) if (std::strcmp((a), (b)) != 0) { FAIL(std::string(a) + " != " + std::string(b)); }

void test_trig_helpers() {
    TEST("toRad and toDeg conversions");
    ASSERT_NEAR(toRad(180.0), M_PI, 1e-9);
    ASSERT_NEAR(toDeg(M_PI), 180.0, 1e-9);
    ASSERT_NEAR(toDeg(toRad(45.0)), 45.0, 1e-9);
    ASSERT_NEAR(toDeg(toRad(360.0)), 360.0, 1e-9);
    PASS();
}

void test_grid_to_geopoint() {
    TEST("gridToGeoPoint center coordinates");
    double lat = 0.0, lon = 0.0;

    // JJ00AA: Origin grid center (Equator and Prime Meridian)
    ASSERT_TRUE(gridToGeoPoint("JJ00AA", &lat, &lon));
    ASSERT_NEAR(lat, 1.25 / 60.0, 1e-6); // +0.020833 deg
    ASSERT_NEAR(lon, 2.5 / 60.0, 1e-6);  // +0.041667 deg

    // 4-character grid: EN40 defaults to center of 2 deg x 1 deg square
    ASSERT_TRUE(gridToGeoPoint("EN40", &lat, &lon));
    ASSERT_NEAR(lat, 40.5, 1e-5);
    ASSERT_NEAR(lon, -91.0, 1e-5);

    // 8-character grid: EN40ab56 preserves extended precision
    ASSERT_TRUE(gridToGeoPoint("EN40ab56", &lat, &lon));
    ASSERT_NEAR(lat, 40.06875, 1e-5);
    ASSERT_NEAR(lon, -91.954167, 1e-5);

    // Coordinates should stay within valid geographic bounds
    ASSERT_TRUE(lat >= -90.0 && lat <= 90.0);
    PASS();
}

void test_arrl_1km_rule() {
    TEST("ARRL 1 km minimum distance rule");
    // Same 6-character grid must return exactly 1.0 km
    ASSERT_NEAR(calculateArrlContestDistance("CM87wa", "CM87wa"), 1.0, 1e-9);
    ASSERT_NEAR(calculateArrlContestDistance("FN31AA", "FN31AA"), 1.0, 1e-9);
    ASSERT_NEAR(calculateArrlContestDistance("JJ00AA", "JJ00AA"), 1.0, 1e-9);
    ASSERT_NEAR(calculateArrlContestDistance("ZA00AA", "ZA00AA"), 1.0, 1e-9);

    // Adjacent subsquares (2.5' lat step ~ 4.63 km) must exceed 1.0 km
    double adj = calculateArrlContestDistance("CM87AA", "CM87AB");
    ASSERT_TRUE(adj > 1.0);
    ASSERT_NEAR(adj, 4.63, 0.1);
    PASS();
}

void test_arrl_symmetry_and_known_paths() {
    TEST("ARRL distance symmetry and known distance ranges");
    // Symmetry: distance(A, B) == distance(B, A)
    double d1 = calculateArrlContestDistance("CM87wa", "DM04ms");
    double d2 = calculateArrlContestDistance("DM04ms", "CM87wa");
    ASSERT_NEAR(d1, d2, 1e-9);

    // CM87wa to DM04ms center-to-center is ~379.37 km
    ASSERT_NEAR(d1, 379.37, 0.5);

    // Boot default distance: JJ00AA to ZA00AA is > 10,000 km
    double boot_dist = calculateArrlContestDistance("JJ00AA", "ZA00AA");
    ASSERT_TRUE(boot_dist > 10000.0);
    PASS();
}

void test_wgs84_distance_and_bearing() {
    TEST("calculateWgs84 distance and cardinal bearings");
    double dist = 0.0, bearing = 0.0;

    // Same coordinates
    calculateWgs84(37.5, -122.0, 37.5, -122.0, dist, bearing);
    ASSERT_NEAR(dist, 0.0, 1e-6);

    // True North: lat increases, lon constant -> bearing 0 deg
    calculateWgs84(0.0, 0.0, 1.0, 0.0, dist, bearing);
    ASSERT_NEAR(bearing, 0.0, 1e-4);
    ASSERT_NEAR(dist, 111319.5, 1000.0);

    // True East: lat constant, lon increases -> bearing 90 deg
    calculateWgs84(0.0, 0.0, 0.0, 1.0, dist, bearing);
    ASSERT_NEAR(bearing, 90.0, 1e-4);

    // True South: lat decreases, lon constant -> bearing 180 deg
    calculateWgs84(1.0, 0.0, 0.0, 0.0, dist, bearing);
    ASSERT_NEAR(bearing, 180.0, 1e-4);

    // True West: lat constant, lon decreases -> bearing 270 deg
    calculateWgs84(0.0, 1.0, 0.0, 0.0, dist, bearing);
    ASSERT_NEAR(bearing, 270.0, 1e-4);
    PASS();
}

void test_locator_round_trip() {
    TEST("geoPointToGrid and gridToGeoPoint round-trip consistency");
    const char *grids[] = {"CM87AA", "FN31AA", "JJ00AA", "IO91AA"};
    for (const char *expected : grids) {
        double lat = 0.0, lon = 0.0;
        ASSERT_TRUE(gridToGeoPoint(expected, &lat, &lon));

        char result[9];
        ASSERT_TRUE(geoPointToGrid(lat, lon, result, 8));
        ASSERT_TRUE(std::strncmp(result, expected, 6) == 0);
    }

    // 8-character full precision round-trip
    double lat8 = 0.0, lon8 = 0.0;
    ASSERT_TRUE(gridToGeoPoint("EN40ab56", &lat8, &lon8));
    char result8[9];
    ASSERT_TRUE(geoPointToGrid(lat8, lon8, result8, 8));
    ASSERT_TRUE(std::strcmp(result8, "EN40AB56") == 0);

    PASS();
}

void test_arrl_string_formatting_and_buffer_safety() {
    TEST("ARRL string formatting, buffer bounds, and ghost character erasure");
    char arrl_str[24];
    double test_distances[] = {1.0, 15.4, 99.9, 999.9, 1000.0, 4520.3, 10008.1, 20015.0};

    for (double d : test_distances) {
        std::memset(arrl_str, 0, sizeof(arrl_str));
        int written = std::snprintf(arrl_str, sizeof(arrl_str), "ARRL %6.1f k  ", d);

        // Must never exceed buffer size of 24
        ASSERT_TRUE(written > 0 && written < (int)sizeof(arrl_str));
        ASSERT_TRUE(std::strlen(arrl_str) < sizeof(arrl_str));

        // Must always start with "ARRL "
        ASSERT_TRUE(std::strncmp(arrl_str, "ARRL ", 5) == 0);

        // Must always end with trailing spaces to erase ghost characters
        ASSERT_TRUE(arrl_str[written - 1] == ' ');
        ASSERT_TRUE(arrl_str[written - 2] == ' ');
        ASSERT_TRUE(arrl_str[written - 3] == 'k');
    }

    // Verify trailing space alignment for < 1000 km
    std::snprintf(arrl_str, sizeof(arrl_str), "ARRL %6.1f k  ", 1.0);
    ASSERT_STR_EQ(arrl_str, "ARRL    1.0 k  ");

    // Verify alignment for 999.9 km
    std::snprintf(arrl_str, sizeof(arrl_str), "ARRL %6.1f k  ", 999.9);
    ASSERT_STR_EQ(arrl_str, "ARRL  999.9 k  ");

    // Verify alignment for >= 1000 km
    std::snprintf(arrl_str, sizeof(arrl_str), "ARRL %6.1f k  ", 1000.0);
    ASSERT_STR_EQ(arrl_str, "ARRL 1000.0 k  ");
    PASS();
}

void test_rover_transit_threshold() {
    TEST("Rover 16 km transit threshold logic");
    // Under 16 km (< 16000 meters)
    double dist_under = 15999.0;
    int rdist_under = (int)std::floor(dist_under / 1000.0);
    ASSERT_TRUE(rdist_under < 16);

    // Exactly 16 km (16000 meters)
    double dist_exact = 16000.0;
    int rdist_exact = (int)std::floor(dist_exact / 1000.0);
    ASSERT_TRUE(rdist_exact >= 16);

    // Over 16 km
    double dist_over = 16001.0;
    int rdist_over = (int)std::floor(dist_over / 1000.0);
    ASSERT_TRUE(rdist_over >= 16);
    PASS();
}

void test_get_nth_field() {
    TEST("getNthField comma-separated field extraction");
    const char *csv = "$KSXT,20261024,116.23,40.07,68.3,299.22,-67.03,190.28,0.022,0.0,1,3,46,28*27";
    const char *f0 = getNthField(csv, 0);
    ASSERT_TRUE(f0 != NULL && std::strncmp(f0, "$KSXT", 5) == 0);

    const char *f1 = getNthField(csv, 1);
    ASSERT_TRUE(f1 != NULL && std::strncmp(f1, "20261024", 8) == 0);

    const char *f5 = getNthField(csv, 5);
    ASSERT_TRUE(f5 != NULL && std::strncmp(f5, "299.22", 6) == 0);

    const char *f11 = getNthField(csv, 11);
    ASSERT_TRUE(f11 != NULL && f11[0] == '3');

    // Out of bounds field
    const char *f99 = getNthField(csv, 99);
    ASSERT_TRUE(f99 == NULL);

    // Empty field
    const char *empty_csv = "$GNHDT,,T*1E";
    const char *ef1 = getNthField(empty_csv, 1);
    ASSERT_TRUE(ef1 != NULL && ef1[0] == ',');
    PASS();
}

void test_ksxt_parsing() {
    TEST("parseKSXT heading and quality extraction (Fixed, Float, Invalid)");
    float heading = 0.0f;
    BearingQuality quality = BEARING_NONE;

    // Status 3: RTK Fixed (Unicore native fixed)
    const char *ksxt_fixed3 = "$KSXT,20261024.00,116.23,40.07,68.3,299.22,-67.03,190.28,0.022,0.0,1,3,46,28*27";
    ASSERT_TRUE(parseHeadingSentence(ksxt_fixed3, &heading, &quality));
    ASSERT_NEAR(heading, 299.22, 1e-2);
    ASSERT_TRUE(quality == BEARING_FIXED);

    // Status 4: RTK Fixed (NMEA standard enum)
    const char *ksxt_fixed4 = "$KSXT,20261024.00,116.23,40.07,68.3,120.50,-10.00,190.28,0.022,0.0,1,4,46,28*27";
    ASSERT_TRUE(parseHeadingSentence(ksxt_fixed4, &heading, &quality));
    ASSERT_NEAR(heading, 120.50, 1e-2);
    ASSERT_TRUE(quality == BEARING_FIXED);

    // Status 2: RTK Float (Unicore native float)
    const char *ksxt_float2 = "$KSXT,20261024.00,116.23,40.07,68.3,145.75,-10.00,190.28,0.022,0.0,1,2,30,18*27";
    ASSERT_TRUE(parseHeadingSentence(ksxt_float2, &heading, &quality));
    ASSERT_NEAR(heading, 145.75, 1e-2);
    ASSERT_TRUE(quality == BEARING_FLOAT);

    // Status 5: RTK Float (NMEA standard enum)
    const char *ksxt_float5 = "$KSXT,20261024.00,116.23,40.07,68.3,145.75,-10.00,190.28,0.022,0.0,1,5,30,18*27";
    ASSERT_TRUE(parseHeadingSentence(ksxt_float5, &heading, &quality));
    ASSERT_NEAR(heading, 145.75, 1e-2);
    ASSERT_TRUE(quality == BEARING_FLOAT);

    // Status 1: Single Point Positioning (carrier-phase baseline not resolved -> must be rejected)
    const char *ksxt_single = "$KSXT,20261024.00,116.23,40.07,68.3,145.75,-10.00,190.28,0.022,0.0,1,1,30,18*27";
    ASSERT_TRUE(!parseHeadingSentence(ksxt_single, &heading, &quality));
    ASSERT_TRUE(quality == BEARING_LOST);

    // Status 0: Invalid solution
    const char *ksxt_inv = "$KSXT,20261024.00,116.23,40.07,68.3,145.75,-10.00,190.28,0.022,0.0,1,0,10,5*27";
    ASSERT_TRUE(!parseHeadingSentence(ksxt_inv, &heading, &quality));
    ASSERT_TRUE(quality == BEARING_LOST);

    // Empty heading field: Baseline lost
    const char *ksxt_empty = "$KSXT,20261024.00,116.23,40.07,68.3,,-10.00,190.28,0.022,0.0,1,0,10,5*27";
    ASSERT_TRUE(!parseHeadingSentence(ksxt_empty, &heading, &quality));
    ASSERT_TRUE(quality == BEARING_LOST);
    PASS();
}

void test_ksxt_time_and_location() {
    TEST("parseKSXTTime and parseKSXTLocation extraction");
    int h = 0, m = 0;

    // 1. Full 14-digit UTC timestamp (yyyymmddhhmmss.ss)
    const char *ksxt_full = "$KSXT,20261024185730.00,116.23662400,40.07897925,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    ASSERT_TRUE(parseKSXTTime(ksxt_full, &h, &m));
    ASSERT_TRUE(h == 18);
    ASSERT_TRUE(m == 57);

    // 2. Short 6-digit UTC timestamp (hhmmss.ss)
    const char *ksxt_short_time = "$KSXT,091545.00,116.23662400,40.07897925,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    ASSERT_TRUE(parseKSXTTime(ksxt_short_time, &h, &m));
    ASSERT_TRUE(h == 9);
    ASSERT_TRUE(m == 15);

    // 3. Date only (8-digit yyyymmdd) - must return false (no time)
    const char *ksxt_date_only = "$KSXT,20261024.00,116.23662400,40.07897925,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    ASSERT_TRUE(!parseKSXTTime(ksxt_date_only, &h, &m));

    // 4. Empty UTC field
    const char *ksxt_empty_time = "$KSXT,,116.23662400,40.07897925,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    ASSERT_TRUE(!parseKSXTTime(ksxt_empty_time, &h, &m));

    // 5. Valid Location extraction (Pos qual == 1, Single, full 8-decimal double precision)
    double lat = 0.0, lon = 0.0;
    ASSERT_TRUE(parseKSXTLocation(ksxt_full, &lat, &lon));
    ASSERT_NEAR(lat, 40.07897925, 1e-8);
    ASSERT_NEAR(lon, 116.23662400, 1e-8);

    // 6. Western / Southern hemisphere coordinates
    const char *ksxt_west = "$KSXT,20261024185730.00,-122.12345678,37.45678901,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    ASSERT_TRUE(parseKSXTLocation(ksxt_west, &lat, &lon));
    ASSERT_NEAR(lat, 37.45678901, 1e-8);
    ASSERT_NEAR(lon, -122.12345678, 1e-8);

    // 7. Invalid Location (Pos qual == 0)
    const char *ksxt_no_pos = "$KSXT,20261024185730.00,116.23662400,40.07897925,68.3830,299.22,-67.03,190.28,0.022,,0,0,0,0*27";
    ASSERT_TRUE(!parseKSXTLocation(ksxt_no_pos, &lat, &lon));

    // 8. Empty Lat/Lon fields
    const char *ksxt_empty_loc = "$KSXT,20261024185730.00,,,68.3830,299.22,-67.03,190.28,0.022,,1,0,0,0*27";
    ASSERT_TRUE(!parseKSXTLocation(ksxt_empty_loc, &lat, &lon));

    PASS();
}

void test_legacy_hdt_ths_parsing() {
    TEST("Legacy GNHDT and GNTHS parsing and lost-signal detection");
    float heading = 0.0f;
    BearingQuality quality = BEARING_NONE;

    // Valid GNHDT
    const char *hdt_valid = "$GNHDT,180.50,T*1C";
    ASSERT_TRUE(parseHeadingSentence(hdt_valid, &heading, &quality));
    ASSERT_NEAR(heading, 180.50, 1e-2);
    ASSERT_TRUE(quality == BEARING_FIXED);

    // Valid GPHDT prefix
    const char *gphdt_valid = "$GPHDT,45.20,T*2B";
    ASSERT_TRUE(parseHeadingSentence(gphdt_valid, &heading, &quality));
    ASSERT_NEAR(heading, 45.20, 1e-2);
    ASSERT_TRUE(quality == BEARING_FIXED);

    // Empty GNHDT (no differential lock)
    const char *hdt_empty = "$GNHDT,,T*1E";
    ASSERT_TRUE(!parseHeadingSentence(hdt_empty, &heading, &quality));
    ASSERT_TRUE(quality == BEARING_LOST);

    // Valid GNTHS with mode 'A'
    const char *ths_valid = "$GNTHS,270.80,A*1F";
    ASSERT_TRUE(parseHeadingSentence(ths_valid, &heading, &quality));
    ASSERT_NEAR(heading, 270.80, 1e-2);
    ASSERT_TRUE(quality == BEARING_FIXED);

    // Invalid GNTHS with mode 'V'
    const char *ths_invalid = "$GNTHS,270.80,V*2A";
    ASSERT_TRUE(!parseHeadingSentence(ths_invalid, &heading, &quality));
    ASSERT_TRUE(quality == BEARING_LOST);

    // Empty GNTHS with mode 'V'
    const char *ths_empty_v = "$GNTHS,,V*35";
    ASSERT_TRUE(!parseHeadingSentence(ths_empty_v, &heading, &quality));
    ASSERT_TRUE(quality == BEARING_LOST);
    PASS();
}

void test_current_bearing_calculation() {
    TEST("calcCurrentBearing 270 deg offset and circular wrap-around");
    // Baseline pointing North (0 deg): (0 + 270) % 360 = 270 deg
    float s0 = std::sin(toRad(0.0)) * BSZ;
    float c0 = std::cos(toRad(0.0)) * BSZ;
    ASSERT_NEAR(calcCurrentBearing(s0, c0), 270.0, 1e-3);

    // Baseline pointing East (90 deg): (90 + 270) % 360 = 0 deg
    float s90 = std::sin(toRad(90.0)) * BSZ;
    float c90 = std::cos(toRad(90.0)) * BSZ;
    ASSERT_NEAR(calcCurrentBearing(s90, c90), 0.0, 1e-3);

    // Baseline pointing South (180 deg): (180 + 270) % 360 = 90 deg
    float s180 = std::sin(toRad(180.0)) * BSZ;
    float c180 = std::cos(toRad(180.0)) * BSZ;
    ASSERT_NEAR(calcCurrentBearing(s180, c180), 90.0, 1e-3);

    // Baseline pointing West (270 deg): (270 + 270) % 360 = 180 deg
    float s270 = std::sin(toRad(270.0)) * BSZ;
    float c270 = std::cos(toRad(270.0)) * BSZ;
    ASSERT_NEAR(calcCurrentBearing(s270, c270), 180.0, 1e-3);
    PASS();
}

void test_antenna_calibration() {
    TEST("Antenna offset normalization, bearing offset application, and circular wrapping");

    // 1. normalizeOffset
    ASSERT_NEAR(normalizeOffset(0.0f), 0.0f, 1e-4);
    ASSERT_NEAR(normalizeOffset(4.2f), 4.2f, 1e-4);
    ASSERT_NEAR(normalizeOffset(-12.5f), -12.5f, 1e-4);
    ASSERT_NEAR(normalizeOffset(180.0f), 180.0f, 1e-4);
    ASSERT_NEAR(normalizeOffset(-180.0f), -180.0f, 1e-4);
    // Wrap around over 180 deg
    ASSERT_NEAR(normalizeOffset(185.0f), -175.0f, 1e-4);
    ASSERT_NEAR(normalizeOffset(-185.0f), 175.0f, 1e-4);
    // Wrap around from compass subtraction: Target = 5 deg, Raw = 355 deg -> delta = -350 deg -> normalized +10 deg
    ASSERT_NEAR(normalizeOffset(5.0f - 355.0f), 10.0f, 1e-4);
    // Target = 355 deg, Raw = 5 deg -> delta = +350 deg -> normalized -10 deg
    ASSERT_NEAR(normalizeOffset(355.0f - 5.0f), -10.0f, 1e-4);

    // 2. applyBearingOffset
    ASSERT_NEAR(applyBearingOffset(120.3, 4.2f), 124.5, 1e-4);
    ASSERT_NEAR(applyBearingOffset(358.0, 4.0f), 2.0, 1e-4);
    ASSERT_NEAR(applyBearingOffset(2.0, -5.0f), 357.0, 1e-4);
    ASSERT_NEAR(applyBearingOffset(0.0, 0.0f), 0.0, 1e-4);

    // 3. calcCurrentBearing with calibration offset
    float s0 = std::sin(toRad(0.0)) * BSZ;
    float c0 = std::cos(toRad(0.0)) * BSZ;
    // Raw is 270.0 deg. With +10.0 offset -> 280.0 deg
    ASSERT_NEAR(calcCurrentBearing(s0, c0, 10.0f), 280.0, 1e-3);
    // Raw 270.0 deg with -10.0 offset -> 260.0 deg
    ASSERT_NEAR(calcCurrentBearing(s0, c0, -10.0f), 260.0, 1e-3);

    float s90 = std::sin(toRad(90.0)) * BSZ;
    float c90 = std::cos(toRad(90.0)) * BSZ;
    // Raw is 0.0 deg. With -5.0 offset -> 355.0 deg
    ASSERT_NEAR(calcCurrentBearing(s90, c90, -5.0f), 355.0, 1e-3);
    // Default offset parameter should equal 0.0 offset
    ASSERT_NEAR(calcCurrentBearing(s90, c90), 0.0, 1e-3);
    PASS();
}

void test_bearing_filter_class() {
    TEST("BearingFilter vector smoothing, seed, and 360 wrap-around");
    BearingFilter filter;
    ASSERT_TRUE(!filter.hasSamples());
    ASSERT_TRUE(filter.sampleCount() == 0);

    // Baseline pointing North (0 deg): (0 + 270) % 360 = 270 deg
    filter.seed(0.0f);
    ASSERT_TRUE(filter.hasSamples());
    ASSERT_TRUE(filter.sampleCount() == 5);
    ASSERT_NEAR(filter.getRawHeading(), 270.0, 1e-3);

    // With +10 deg calibration offset -> 280 deg
    ASSERT_NEAR(filter.getCalibratedHeading(10.0f), 280.0, 1e-3);
    // With -10 deg calibration offset -> 260 deg
    ASSERT_NEAR(filter.getCalibratedHeading(-10.0f), 260.0, 1e-3);

    // Baseline pointing East (90 deg): (90 + 270) % 360 = 0 deg
    filter.seed(90.0f);
    ASSERT_NEAR(filter.getRawHeading(), 0.0, 1e-3);

    // Push samples across 0/360 wrap-around: alternate 359 deg and 1 deg
    // True circular average of 359 and 1 is 0 deg (or 360 deg)
    // Raw heading = (0 + 270) = 270 deg
    filter.reset();
    filter.push(359.0f);
    filter.push(1.0f);
    filter.push(359.0f);
    filter.push(1.0f);
    filter.push(0.0f);
    ASSERT_TRUE(filter.sampleCount() == 5);
    ASSERT_NEAR(filter.getRawHeading(), 270.0, 0.5);

    PASS();
}

void test_gps_stream_evaluation() {
    TEST("evaluateGpsStream link verdicts (Disconnected, Corrupt, Acquiring, Float, Fixed)");

    // 1. Zero bytes received (unplugged since boot)
    GpsLinkVerdict v1 = evaluateGpsStream(10000, 0, 0, 0, 0, 0, false, false, BEARING_NONE);
    ASSERT_TRUE(v1 == GPS_LINK_DISCONNECTED);

    // 2. Stale bytes (> 3000 ms elapsed since last byte)
    GpsLinkVerdict v2 = evaluateGpsStream(10000, 500, 6000, 10, 50, 0, true, true, BEARING_FIXED);
    ASSERT_TRUE(v2 == GPS_LINK_DISCONNECTED);

    // 3. Raw bytes streaming, but no recognized NMEA/KSXT sentences (corrupt / wrong baud)
    GpsLinkVerdict v3 = evaluateGpsStream(10000, 1500, 9900, 0, 0, 0, false, false, BEARING_NONE);
    ASSERT_TRUE(v3 == GPS_LINK_NOISY_CORRUPT);

    // 4. Packets arriving, but searching for satellites (no location, no heading)
    GpsLinkVerdict v4 = evaluateGpsStream(10000, 2000, 9900, 1, 5, 0, false, false, BEARING_NONE);
    ASSERT_TRUE(v4 == GPS_LINK_ACQUIRING);

    // 5. Position locked via GLL, but no dual-antenna heading lock yet
    GpsLinkVerdict v5 = evaluateGpsStream(10000, 2500, 9900, 2, 0, 0, true, false, BEARING_NONE);
    ASSERT_TRUE(v5 == GPS_LINK_POSITION_ONLY);

    // 6. Dual-antenna heading locked in RTK Float mode (degraded / yellow)
    GpsLinkVerdict v6 = evaluateGpsStream(10000, 5000, 9900, 5, 25, 0, true, true, BEARING_FLOAT);
    ASSERT_TRUE(v6 == GPS_LINK_HEADING_FLOAT);

    // 7. Full dual-antenna lock in RTK Fixed mode (high-precision / white)
    GpsLinkVerdict v7 = evaluateGpsStream(10000, 5000, 9900, 5, 25, 0, true, true, BEARING_FIXED);
    ASSERT_TRUE(v7 == GPS_LINK_OK_FIXED);
    PASS();
}

void test_geopoint_and_modern_domain() {
    TEST("GeoPoint domain model, grid projection, and rover threshold checks");
    GeoPoint origin(38.354, -119.458, true);
    GeoPoint target;

    // Grid center to GeoPoint
    ASSERT_TRUE(gridToGeoPoint("CM87wb", target));
    ASSERT_TRUE(target.isValid);
    ASSERT_NEAR(target.latitude, 37.0625, 1e-4);
    ASSERT_NEAR(target.longitude, -122.125, 1e-4);

    // GeoPoint back to Grid
    char grid6[7];
    ASSERT_TRUE(geoPointToGrid(origin, grid6, 6));
    ASSERT_STR_EQ(grid6, "DM08GI");

    // WGS84 with GeoPoint structs
    double distM = 0.0, bearingDeg = 0.0;
    calculateWgs84(origin, target, distM, bearingDeg);
    ASSERT_TRUE(distM > 270000.0 && distM < 280000.0); // ~275 km

    // Reverse bearing
    double revBearing = calculateReverseBearing(origin, target);
    ASSERT_NEAR(revBearing, 57.72, 0.05);

    // Rover transit alert: station within 15 km is inside alert; station at 17 km is outside
    GeoPoint nearby(38.360, -119.460, true);
    ASSERT_TRUE(isRoverTransitInsideAlert(nearby, origin, 16.0));

    GeoPoint farAway(38.550, -119.460, true); // ~21 km North
    ASSERT_TRUE(!isRoverTransitInsideAlert(farAway, origin, 16.0));

    // KSXT location parser into GeoPoint
    const char *ksxt_sample = "$KSXT,20261024185730.00,-119.45821000,38.35412000,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    GeoPoint parsedGps;
    ASSERT_TRUE(parseKsxtLocation(ksxt_sample, parsedGps));
    ASSERT_TRUE(parsedGps.isValid);
    ASSERT_NEAR(parsedGps.latitude, 38.35412000, 1e-6);
    ASSERT_NEAR(parsedGps.longitude, -119.45821000, 1e-6);

    PASS();
}

void test_config_constants() {
    TEST("src/config.h constants and macro dimensions");
    ASSERT_TRUE(SCREEN_WIDTH == 480);
    ASSERT_TRUE(SCREEN_HEIGHT == 480);
    ASSERT_TRUE(CHAR_W(2) == 12);
    ASSERT_TRUE(CHAR_H(2) == 16);
    ASSERT_TRUE(RICH_YELLOW == 0xFEA0);
    ASSERT_TRUE(HEADING_TIMEOUT_MS == 1500);
    ASSERT_TRUE(EDIT_TIMEOUT_MS == 5000);
    ASSERT_TRUE(CAL_MODAL_TIMEOUT_MS == 15000);
    ASSERT_NEAR(ROVER_ALERT_THRESHOLD_KM, 16.0, 1e-6);

    // Test grid character rolling macros
    char c = 'A';
    ASSERT_TRUE(INCA(c) == 'B');
    ASSERT_TRUE(DECA(c) == 'Z');
    c = 'Z';
    ASSERT_TRUE(INCA(c) == 'A');

    char n = '0';
    ASSERT_TRUE(INCN(n) == '1');
    ASSERT_TRUE(DECN(n) == '9');
    n = '9';
    ASSERT_TRUE(INCN(n) == '0');

    PASS();
}

void test_calibration_class() {
    TEST("Calibration class offset staging and persistence");
    Calibration::resetHostStorage();

    Calibration cal;
    cal.begin();
    ASSERT_NEAR(cal.getOffset(), 0.0f, 1e-4);
    ASSERT_TRUE(!cal.isCalibrated());

    // Setting offset and wrap normalization
    cal.setOffset(4.2f);
    ASSERT_NEAR(cal.getOffset(), 4.2f, 1e-4);
    ASSERT_TRUE(cal.isCalibrated());

    // Wrap-around normalization > 180 and < -180
    cal.setOffset(190.0f);
    ASSERT_NEAR(cal.getOffset(), -170.0f, 1e-4);

    cal.setOffset(-200.0f);
    ASSERT_NEAR(cal.getOffset(), 160.0f, 1e-4);

    // Snapping: target = 10.0, raw = 355.0 -> delta = +15.0
    cal.snap(10.0, 355.0);
    ASSERT_NEAR(cal.getOffset(), 15.0f, 1e-4);

    // Staging and proposed offset manipulation
    cal.reset();
    ASSERT_NEAR(cal.getOffset(), 0.0f, 1e-4);
    ASSERT_TRUE(!cal.isCalibrated());

    cal.stageProposedOffset(5.0f);
    ASSERT_NEAR(cal.getProposedOffset(), 5.0f, 1e-4);
    ASSERT_NEAR(cal.getOffset(), 0.0f, 1e-4); // Active unchanged until commit

    cal.adjustProposedOffset(0.1f);
    ASSERT_NEAR(cal.getProposedOffset(), 5.1f, 1e-4);

    cal.adjustProposedOffset(-0.2f);
    ASSERT_NEAR(cal.getProposedOffset(), 4.9f, 1e-4);

    cal.snapProposed(45.0, 40.0); // +5.0
    ASSERT_NEAR(cal.getProposedOffset(), 5.0f, 1e-4);

    cal.commitProposed();
    ASSERT_NEAR(cal.getOffset(), 5.0f, 1e-4);

    // Persistence verification: new instance loads persisted value
    Calibration cal2;
    cal2.begin();
    ASSERT_NEAR(cal2.getOffset(), 5.0f, 1e-4);
    ASSERT_TRUE(cal2.isCalibrated());

    // Bearing application
    ASSERT_NEAR(cal2.apply(358.0), 3.0, 1e-4);

    // In-memory update without persistence (used during Self-Test demo to avoid flash wear)
    cal2.setOffset(4.2f, false);
    ASSERT_NEAR(cal2.getOffset(), 4.2f, 1e-4);
    Calibration cal3;
    cal3.begin();
    ASSERT_NEAR(cal3.getOffset(), 5.0f, 1e-4); // NVS storage preserved at 5.0f!

    PASS();
}

void test_gps_simulator() {
    TEST("GpsSimulator synthetic sentences and virtual rover displacement");
    GpsSimulator sim;
    ASSERT_TRUE(!sim.isActive());
    ASSERT_TRUE(sim.getPacketCount() == 0);

    // Checksum verification
    uint8_t csum = GpsSimulator::computeChecksum("$GNHDT,123.4,T");
    ASSERT_TRUE(csum != 0);

    // Sentence injection and callback
    static bool sentenceReceived = false;
    static std::string lastSentence;
    sim.injectSentence("$GNHDT,045.0,T*26", [](const char *s) {
        sentenceReceived = true;
        lastSentence = s;
    }, 1000);

    ASSERT_TRUE(sentenceReceived);
    ASSERT_STR_EQ(lastSentence.c_str(), "$GNHDT,045.0,T*26");
    ASSERT_TRUE(sim.isActive());
    ASSERT_TRUE(sim.getPacketCount() == 1);
    ASSERT_TRUE(sim.getLastInjectionMillis() == 1000);

    // Synthetic KSXT generation & roundtrip parsing
    char ksxtBuf[256];
    ASSERT_TRUE(GpsSimulator::generateKsxt(ksxtBuf, sizeof(ksxtBuf),
                                          "20261106180000.00", 38.35412, -119.45821,
                                          125.5, BEARING_FIXED));
    float hOut = 0.0f;
    BearingQuality qOut = BEARING_LOST;
    ASSERT_TRUE(parseKsxtHeading(ksxtBuf, hOut, qOut));
    ASSERT_NEAR(hOut, 125.5, 0.01);
    ASSERT_TRUE(qOut == BEARING_FIXED);

    GeoPoint ptOut;
    ASSERT_TRUE(parseKsxtLocation(ksxtBuf, ptOut));
    ASSERT_NEAR(ptOut.latitude, 38.35412, 1e-5);
    ASSERT_NEAR(ptOut.longitude, -119.45821, 1e-5);

    // Synthetic HDT generation & roundtrip parsing
    char hdtBuf[64];
    ASSERT_TRUE(GpsSimulator::generateHdt(hdtBuf, sizeof(hdtBuf), 270.5));
    ASSERT_TRUE(parseHDT(hdtBuf, &hOut, &qOut));
    ASSERT_NEAR(hOut, 270.5, 0.01);

    // Virtual rover displacement math
    GeoPoint origin(38.35412, -119.45821, true); // CM87wa center
    GeoPoint displaced10km = GpsSimulator::calculateRoverDisplacement(origin, 10.0, 45.0);
    double dist10m = 0.0, brg10 = 0.0;
    calculateWgs84(origin, displaced10km, dist10m, brg10);
    ASSERT_NEAR(dist10m / 1000.0, 10.0, 0.2); // within 200m over 10km planar approximation
    ASSERT_NEAR(brg10, 45.0, 1.0);
    ASSERT_TRUE(isRoverTransitInsideAlert(displaced10km, origin, 16.0));

    GeoPoint displaced20km = GpsSimulator::calculateRoverDisplacement(origin, 20.0, 45.0);
    double dist20m = 0.0, brg20 = 0.0;
    calculateWgs84(origin, displaced20km, dist20m, brg20);
    ASSERT_NEAR(dist20m / 1000.0, 20.0, 0.4);
    ASSERT_TRUE(!isRoverTransitInsideAlert(displaced20km, origin, 16.0));

    PASS();
}

void test_gps_manager() {
    TEST("GpsManager streaming sentence assembly, watchdog, and health evaluation");
    GpsManager gm;
    ASSERT_TRUE(!gm.hasHeading());
    ASSERT_TRUE(!gm.hasLocation());
    ASSERT_TRUE(!gm.hasTime());
    ASSERT_TRUE(gm.getTotalBytesReceived() == 0);

    // Ingest synthetic KSXT sentence byte by byte via feedByte
    char ksxtBuf[256];
    GpsSimulator::generateKsxt(ksxtBuf, sizeof(ksxtBuf),
                              "20261106180000.00", 38.35412, -119.45821,
                              123.4, BEARING_FIXED);

    size_t len = strlen(ksxtBuf);
    for (size_t i = 0; i < len; ++i) {
        gm.feedByte((uint8_t)ksxtBuf[i], 1000);
    }

    ASSERT_TRUE(gm.hasHeading());
    ASSERT_NEAR(gm.getHeading(), std::fmod(123.4 + 270.0, 360.0), 0.1);
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_FIXED);
    ASSERT_TRUE(gm.getLastHeadingUpdate() == 1000);

    ASSERT_TRUE(gm.hasLocation());
    ASSERT_NEAR(gm.getLocation().latitude, 38.35412, 1e-5);
    ASSERT_NEAR(gm.getLocation().longitude, -119.45821, 1e-5);

    ASSERT_TRUE(gm.hasTime());
    ASSERT_TRUE(gm.getHours() == 18);
    ASSERT_TRUE(gm.getMinutes() == 0);

    // Watchdog check before timeout
    gm.checkStaleness(1500); // 500 ms elapsed
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_FIXED);

    // Watchdog check after timeout (HEADING_TIMEOUT_MS = 1500)
    gm.checkStaleness(2501); // 1501 ms elapsed -> BEARING_LOST
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_LOST);

    // Feed fresh heading recovery
    gm.feedSentence(ksxtBuf, 3000);
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_FIXED);
    ASSERT_TRUE(gm.getLastHeadingUpdate() == 3000);

    // Stream health evaluation
    GpsLinkVerdict v = gm.evaluateStreamHealth(3000);
    ASSERT_TRUE(v == GPS_LINK_OK_FIXED);

    PASS();
}

void test_serial_cli() {
    TEST("SerialCli host USB command execution, reports, and dispatching");
    GpsManager gm;
    Calibration cal;
    GpsSimulator sim;
    SerialCli cli(gm, cal, sim);

    static std::string cliOutput;
    cliOutput.clear();
    cli.setOutputHandler([](const char *str) {
        cliOutput += str;
    });

    // Test help command
    cli.executeCommand("help");
    ASSERT_TRUE(cliOutput.find("GNSS Compass Serial Commands") != std::string::npos);
    cliOutput.clear();

    // Test status command
    cli.executeCommand("status");
    ASSERT_TRUE(cliOutput.find("GNSS Receiver Stream Diagnosis") != std::string::npos);
    cliOutput.clear();

    // Test sim commands
    ASSERT_TRUE(!sim.isActive());
    cli.executeCommand("sim on");
    ASSERT_TRUE(sim.isActive());
    ASSERT_TRUE(cliOutput.find("Simulation mode enabled") != std::string::npos);
    cliOutput.clear();

    cli.executeCommand("sim hdg 180.5 float");
    ASSERT_TRUE(gm.hasHeading());
    ASSERT_NEAR(gm.getHeading(), std::fmod(180.5 + 270.0, 360.0), 0.1);
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_FLOAT);
    ASSERT_TRUE(cliOutput.find("Heading set to 180.5") != std::string::npos);
    cliOutput.clear();

    cli.executeCommand("sim pos 38.354 -119.458");
    ASSERT_TRUE(gm.hasLocation());
    ASSERT_NEAR(gm.getLocation().latitude, 38.354, 1e-4);
    ASSERT_NEAR(gm.getLocation().longitude, -119.458, 1e-4);
    cliOutput.clear();

    cli.executeCommand("sim time 14:25");
    ASSERT_TRUE(gm.hasTime());
    ASSERT_TRUE(gm.getHours() == 14);
    ASSERT_TRUE(gm.getMinutes() == 25);
    cliOutput.clear();

    // Test cal commands: raw antenna heading is 90.5 deg (180.5 baseline + 270 deg orthogonal offset)
    cli.setTargetBearing(95.5);
    cli.executeCommand("cal set 2.0");
    ASSERT_NEAR(cal.getOffset(), 2.0f, 1e-4);
    cliOutput.clear();

    // Snap test: raw antenna heading is 90.5, target bearing is 95.5 -> snapped offset = +5.0
    cli.executeCommand("cal snap");
    ASSERT_NEAR(cal.getOffset(), 5.0f, 1e-4);
    cliOutput.clear();

    cli.executeCommand("cal zero");
    ASSERT_NEAR(cal.getOffset(), 0.0f, 1e-4);
    cliOutput.clear();

    // Raw sentence injection through CLI: 090.0 baseline
    char ksxtBuf[256];
    GpsSimulator::generateKsxt(ksxtBuf, sizeof(ksxtBuf),
                              "20261106193000.00", 38.35412, -119.45821,
                              090.0, BEARING_FIXED);
    cli.executeCommand(ksxtBuf);
    ASSERT_TRUE(gm.hasHeading());
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_FIXED);
    ASSERT_TRUE(sim.isActive());

    cli.executeCommand("sim off");
    ASSERT_TRUE(!sim.isActive());

    PASS();
}

void test_input_manager_hit_testing() {
    TEST("InputManager touch hit-testing and grid character translation");

    // Local Grid Hit Area (100..340, 0..75)
    ASSERT_TRUE(InputManager::isLocalGridTouched(100, 0));
    ASSERT_TRUE(InputManager::isLocalGridTouched(340, 75));
    ASSERT_TRUE(InputManager::isLocalGridTouched(220, 35));
    ASSERT_TRUE(!InputManager::isLocalGridTouched(99, 35));
    ASSERT_TRUE(!InputManager::isLocalGridTouched(341, 35));
    ASSERT_TRUE(!InputManager::isLocalGridTouched(220, 76));

    // Target Grid Hit Area (50..430, 70..235)
    ASSERT_TRUE(InputManager::isTargetGridTouched(50, 70));
    ASSERT_TRUE(InputManager::isTargetGridTouched(430, 235));
    ASSERT_TRUE(InputManager::isTargetGridTouched(200, 150));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(49, 150));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(431, 150));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(200, 69));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(200, 236));

    // Target Grid Character Column Mapping (REMSTARTX=80, CHAR_W(REMSIZE)=48)
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(80) == 0);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(127) == 0);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(128) == 1);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(80 + 5 * 48) == 5);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(30) == 0);   // Underflow clamp
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(450) == 5);  // Overflow clamp

    // Calibration Pill Hit Area (180..300, 210..280)
    ASSERT_TRUE(InputManager::isCalPillTouched(180, 210));
    ASSERT_TRUE(InputManager::isCalPillTouched(300, 280));
    ASSERT_TRUE(InputManager::isCalPillTouched(240, 245));
    ASSERT_TRUE(!InputManager::isCalPillTouched(179, 245));
    ASSERT_TRUE(!InputManager::isCalPillTouched(301, 245));
    ASSERT_TRUE(!InputManager::isCalPillTouched(240, 209));
    ASSERT_TRUE(!InputManager::isCalPillTouched(240, 281));

    // Footer Hit Area (100..380, >= 435)
    ASSERT_TRUE(InputManager::isFooterTouched(100, 435));
    ASSERT_TRUE(InputManager::isFooterTouched(380, 479));
    ASSERT_TRUE(InputManager::isFooterTouched(240, 450));
    ASSERT_TRUE(!InputManager::isFooterTouched(99, 450));
    ASSERT_TRUE(!InputManager::isFooterTouched(381, 450));
    ASSERT_TRUE(!InputManager::isFooterTouched(240, 434));

    // Modal Buttons Hit Areas
    ASSERT_TRUE(InputManager::getModalButtonFromTouch(100, 250) == ModalButtonId::Snap);
    ASSERT_TRUE(InputManager::getModalButtonFromTouch(300, 250) == ModalButtonId::Reset);
    ASSERT_TRUE(InputManager::getModalButtonFromTouch(200, 300) == ModalButtonId::Save);
    ASSERT_TRUE(InputManager::getModalButtonFromTouch(200, 360) == ModalButtonId::Cancel);
    ASSERT_TRUE(InputManager::getModalButtonFromTouch(10, 10) == ModalButtonId::None);

    PASS();
}

void test_input_manager_gestures() {
    TEST("InputManager gesture state machine, encoder events, and long-press hold");

    InputManager im;

    // 1. Rotary Encoder Events
    im.injectEncoderStep(+1);
    UserEvent evt;
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::RotateClockwise);

    im.injectEncoderStep(-1);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::RotateCounterClockwise);

    // 2. Normal Mode Short Taps
    // Tap on Cal Pill
    im.injectTouch(true, 240, 245, 1000);
    im.injectTouch(false, 0, 0, 1150);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::TouchCalibrationPill);

    // Tap on Local Grid
    im.injectTouch(true, 200, 35, 1300);
    im.injectTouch(false, 0, 0, 1420);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::TouchLocalGrid);

    // Tap on Target Grid col 3
    im.injectTouch(true, 80 + 3 * 48 + 10, 120, 1600);
    im.injectTouch(false, 0, 0, 1750);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::TouchGridCharacter);
    ASSERT_TRUE(evt.gridIndex == 3);

    // Tap on Footer
    im.injectTouch(true, 240, 450, 1900);
    im.injectTouch(false, 0, 0, 2020);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::TouchVersionFooter);

    // 3. Calibration Mode Interactions
    im.setUiMode(UiMode::Calibrating);

    // Tap on Snap
    im.injectTouch(true, 100, 250, 2200);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::ModalButtonDown);
    ASSERT_TRUE(evt.buttonId == ModalButtonId::Snap);
    im.injectTouch(false, 0, 0, 2300);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::SnapOffsetTriggered);

    // Tap on Reset
    im.injectTouch(true, 300, 250, 2500);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::ModalButtonDown);
    ASSERT_TRUE(evt.buttonId == ModalButtonId::Reset);
    im.injectTouch(false, 0, 0, 2600);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::ResetOffsetTriggered);

    // Tap on Cancel
    im.injectTouch(true, 200, 360, 2800);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::ModalButtonDown);
    ASSERT_TRUE(evt.buttonId == ModalButtonId::Cancel);
    im.injectTouch(false, 0, 0, 2900);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::CancelModalTriggered);

    // 4. Long Press Hold on Save Button
    im.injectTouch(true, 200, 300, 4000);
    im.injectTouch(true, 200, 300, 4750); // 750ms -> 50%
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::HoldSaveButtonProgress);
    ASSERT_NEAR(evt.progress, 0.5f, 0.05f);

    im.injectTouch(true, 200, 300, 5500); // 1500ms -> Commit!
    bool committed = false;
    while (im.pollEvent(evt)) {
        if (evt.type == UserEventType::SaveButtonCommitted) committed = true;
    }
    ASSERT_TRUE(committed);

    // 5. Finger Drift Cancellation (>35px)
    im.injectTouch(false, 0, 0, 5600);
    im.injectTouch(true, 200, 300, 6000);
    im.injectTouch(true, 200, 345, 6300); // 45px drift
    bool canceled = false;
    while (im.pollEvent(evt)) {
        if (evt.type == UserEventType::HoldSaveButtonProgress && evt.progress == 0.0f) canceled = true;
    }
    ASSERT_TRUE(canceled);

    PASS();
}

void test_ui_formatters_and_modal_geometry() {
    TEST("CalModal and DisplayUi string formatters and layout presentation");

    char buf[64];

    // CalModal Formatters
    CalModal::formatTargetBearing(245.8, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Target Bearing : 245.8");

    CalModal::formatRawHeading(12.3, true, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Raw GNSS Hdg   :  12.3");

    CalModal::formatRawHeading(0.0, false, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Raw GNSS Hdg   : ---.-");

    CalModal::formatCurrentOffset(4.2f, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Current Offset :  +4.2");

    CalModal::formatCurrentOffset(-10.5f, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Current Offset : -10.5");

    CalModal::formatProposedOffset(0.0f, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Proposed Offset:  +0.0");

    // DisplayUi Formatters
    DisplayUi::formatTimeString(9, 3, true, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "09:03Z");

    DisplayUi::formatTimeString(0, 0, false, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "--:--Z");

    DisplayUi::formatReverseBearingAndDistance(65.4, 15400.0, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, " 65.4   15 k  ");

    DisplayUi::formatArrlDistance(1.0, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "ARRL    1.0 k  ");

    DisplayUi::formatBearingString(180.0, true, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "180.0");

    DisplayUi::formatBearingString(0.0, false, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "---.-");

    DisplayUi::formatCalibrationPillText(0.0f, false, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "OFF");

    DisplayUi::formatCalibrationPillText(2.5f, true, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "+2.5");

    PASS();
}

void test_self_test_failure_formatting() {
    TEST("DisplayUi self-test failure message formatting and buffer safety");

    char buf[32];

    // Zero failures -> Pass message
    DisplayUi::formatSelfTestFailure(nullptr, 0, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "SELF-TEST PASSED");

    // Single failure -> "FAILED: TEST #3"
    int singleFail[] = {3};
    DisplayUi::formatSelfTestFailure(singleFail, 1, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "FAILED: TEST #3");

    // Two failures -> "FAILED: #1, #2"
    int twoFails[] = {1, 2};
    DisplayUi::formatSelfTestFailure(twoFails, 2, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "FAILED: #1, #2");

    // Three failures -> "FAILED: #1, #2, #7"
    int threeFails[] = {1, 2, 7};
    DisplayUi::formatSelfTestFailure(threeFails, 3, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "FAILED: #1, #2, #7");

    // Four failures -> "FAILED: #1, #2, #3, #4"
    int fourFails[] = {1, 2, 3, 4};
    DisplayUi::formatSelfTestFailure(fourFails, 4, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "FAILED: #1, #2, #3, #4");
    ASSERT_TRUE(std::strlen(buf) < sizeof(buf));

    // Buffer bounds safety with small buffer (16 bytes)
    char smallBuf[16];
    DisplayUi::formatSelfTestFailure(fourFails, 4, smallBuf, sizeof(smallBuf));
    ASSERT_TRUE(std::strlen(smallBuf) < sizeof(smallBuf));

    PASS();
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  GNSS Compass Unit Test Suite" << std::endl;
    std::cout << "========================================" << std::endl;

    test_trig_helpers();
    test_grid_to_geopoint();
    test_arrl_1km_rule();
    test_arrl_symmetry_and_known_paths();
    test_wgs84_distance_and_bearing();
    test_locator_round_trip();
    test_arrl_string_formatting_and_buffer_safety();
    test_rover_transit_threshold();
    test_get_nth_field();
    test_ksxt_parsing();
    test_ksxt_time_and_location();
    test_legacy_hdt_ths_parsing();
    test_current_bearing_calculation();
    test_bearing_filter_class();
    test_antenna_calibration();
    test_gps_stream_evaluation();
    test_geopoint_and_modern_domain();
    test_config_constants();
    test_calibration_class();
    test_gps_simulator();
    test_gps_manager();
    test_serial_cli();
    test_input_manager_hit_testing();
    test_input_manager_gestures();
    test_ui_formatters_and_modal_geometry();
    test_self_test_failure_formatting();

    std::cout << "========================================" << std::endl;
    std::cout << "  All " << tests_passed << " / " << tests_run << " tests PASSED successfully!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
