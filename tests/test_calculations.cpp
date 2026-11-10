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
#include "../src/ui/rover_card.h"
#include "../src/ui/grid_editor.h"
#include "../src/system/board.h"
#include "../src/system/app_controller.h"
#include "../src/system/rover_log_manager.h"
#include "../src/protocol/ble_serial_manager.h"
#include "../src/system/self_test.h"

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

    // Partial/odd-length grids must parse safely without buffer overrun
    GeoPoint ptSafe;
    ASSERT_TRUE(gridToGeoPoint("EN40a", ptSafe));
    ASSERT_NEAR(ptSafe.latitude, 40.5, 1e-5);
    ASSERT_TRUE(gridToGeoPoint("EN40ab5", ptSafe));
    ASSERT_NEAR(ptSafe.latitude, 40.0625, 1e-4);

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

    // Antipodal point bounds check (ensures a is clamped and no NaN occurs)
    calculateWgs84(0.0, 0.0, 0.0, 180.0, dist, bearing);
    ASSERT_TRUE(!std::isnan(dist));
    ASSERT_TRUE(!std::isnan(bearing));
    ASSERT_NEAR(dist, 20037508.34, 1.0); // Half Earth circumference (pi * 6378137 m)
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

    // Extreme pole and antimeridian bounds safety
    char poleBuf[9];
    ASSERT_TRUE(geoPointToGrid(90.0, 0.0, poleBuf, 8));
    ASSERT_TRUE(poleBuf[1] == 'R'); // Field clamped to 'R' (never illegal 'S')
    ASSERT_TRUE(geoPointToGrid(-90.0, 0.0, poleBuf, 8));
    ASSERT_TRUE(poleBuf[1] == 'A');

    char antiBuf[9];
    ASSERT_TRUE(geoPointToGrid(0.0, 180.0, antiBuf, 8));
    ASSERT_TRUE(antiBuf[0] >= 'A' && antiBuf[0] <= 'R');
    ASSERT_TRUE(geoPointToGrid(0.0, -180.0, antiBuf, 8));
    ASSERT_TRUE(antiBuf[0] >= 'A' && antiBuf[0] <= 'R');

    // Out-of-bounds coordinates clamped safely
    char clampBuf[9];
    ASSERT_TRUE(geoPointToGrid(95.0, 200.0, clampBuf, 8));
    ASSERT_TRUE(clampBuf[0] <= 'R' && clampBuf[1] <= 'R');
    ASSERT_TRUE(geoPointToGrid(-95.0, -200.0, clampBuf, 8));
    ASSERT_TRUE(clampBuf[0] >= 'A' && clampBuf[1] >= 'A');

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

    // Full Date & Time and Epoch Extraction
    int yr = 0, mo = 0, dy = 0, hr = 0, mn = 0, sc = 0;
    ASSERT_TRUE(parseKsxtDateTime(ksxt_full, yr, mo, dy, hr, mn, sc));
    ASSERT_TRUE(yr == 2026 && mo == 10 && dy == 24 && hr == 18 && mn == 57 && sc == 30);
    uint32_t epoch = 0;
    ASSERT_TRUE(parseKsxtEpochTime(ksxt_full, epoch));
    ASSERT_TRUE(epoch == 1792868250);

    // 2. Short 6-digit UTC timestamp (hhmmss.ss)
    const char *ksxt_short_time = "$KSXT,091545.00,116.23662400,40.07897925,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    ASSERT_TRUE(parseKSXTTime(ksxt_short_time, &h, &m));
    ASSERT_TRUE(h == 9);
    ASSERT_TRUE(m == 15);
    ASSERT_TRUE(!parseKsxtEpochTime(ksxt_short_time, epoch)); // Short time lacks calendar date

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

    // Valid GNHDT via parseHdtHeading wrapper & scoped enum
    float hdtHdg = 0.0f;
    BearingQuality hdtQual = BearingQuality::None;
    ASSERT_TRUE(parseHdtHeading(hdt_valid, hdtHdg, hdtQual));
    ASSERT_NEAR(hdtHdg, 180.50, 1e-2);
    ASSERT_TRUE(hdtQual == BearingQuality::Fixed);

    // Valid GNTHS via parseThsHeading wrapper
    float thsHdg = 0.0f;
    BearingQuality thsQual = BearingQuality::None;
    const char *ths_wrapper_test = "$GNTHS,312.4,A*18";
    ASSERT_TRUE(parseThsHeading(ths_wrapper_test, thsHdg, thsQual));
    ASSERT_NEAR(thsHdg, 312.4, 1e-2);
    ASSERT_TRUE(thsQual == BearingQuality::Fixed);

    // KSXT configured the old way without quality fields (compact sentence)
    const char *ksxt_no_quality = "$KSXT,20261024185730.00,-122.084000,37.421998,15.2,142.50*2A";
    float noqHdg = 0.0f;
    BearingQuality noqQual = BearingQuality::None;
    ASSERT_TRUE(parseHeadingSentence(ksxt_no_quality, &noqHdg, &noqQual));
    ASSERT_NEAR(noqHdg, 142.50, 1e-2);
    ASSERT_TRUE(noqQual == BearingQuality::Fixed);

    double noqLat = 0.0, noqLon = 0.0;
    ASSERT_TRUE(parseKSXTLocation(ksxt_no_quality, &noqLat, &noqLon));
    ASSERT_NEAR(noqLat, 37.421998, 1e-6);
    ASSERT_NEAR(noqLon, -122.084000, 1e-6);

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

    // 0. normalizeHeading
    ASSERT_NEAR(normalizeHeading(370.5f), 10.5f, 1e-4);
    ASSERT_NEAR(normalizeHeading(-10.0f), 350.0f, 1e-4);
    ASSERT_NEAR(normalizeHeading(720.0f), 0.0f, 1e-4);
    ASSERT_NEAR(normalizeHeading(0.0f), 0.0f, 1e-4);
    ASSERT_NEAR(normalizeHeading(359.9f), 359.9f, 1e-4);

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
    ASSERT_TRUE(DEFAULT_MAX_ROVER_STOPS == 100);
    ASSERT_TRUE(MIN_ROVER_STOPS == 10);
    ASSERT_TRUE(ABS_MAX_ROVER_STOPS == 500);
    ASSERT_TRUE(ROVER_CARD_SZ == 3);
    ASSERT_TRUE(ROVER_CARD_W == 320);
    ASSERT_TRUE(ROVER_CARD_H == 40);
    ASSERT_TRUE(ROVER_CARD_X == 80);
    ASSERT_TRUE(ROVER_CARD_TEXT_X == 96);
    ASSERT_TRUE(COLOR_ROVER_TRANSIT_RED == 0xF800);
    ASSERT_TRUE(COLOR_ROVER_QUALIFIED_GREEN == 0x03E0);
    ASSERT_TRUE(COLOR_ROVER_REVISIT_BLUE == 0x011C);
    ASSERT_TRUE(COLOR_ROVER_NOGPS_AMBER == 0xDC00);
    ASSERT_TRUE(GPS_TX_PROBE_TIMEOUT_MS == 1000);
    ASSERT_TRUE(GPS_TX_PROBE_MAX_RETRIES == 2);
    ASSERT_TRUE(GPS_CONFIG_STEP_TIMEOUT_MS == 1000);
    ASSERT_TRUE(GPS_CONFIG_VERIFY_TIMEOUT_MS == 1500);

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

    // Test ASCII to numeric digit conversion
    ASSERT_TRUE(charToDigit('0') == 0);
    ASSERT_TRUE(charToDigit('5') == 5);
    ASSERT_TRUE(charToDigit('9') == 9);
    ASSERT_TRUE(charToDigit('X') == 0);
    ASSERT_TRUE(aton('7') == 7);

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

    // GLL sentence ingestion with standard South and West indicators
    const char *gllStandard = "$GNGLL,3821.2472,S,11927.4926,W,193000.00,A*3C";
    gm.feedSentence(gllStandard, 3100);
    ASSERT_TRUE(gm.hasLocation());
    ASSERT_NEAR(gm.getLocation().latitude, -38.35412, 1e-4);
    ASSERT_NEAR(gm.getLocation().longitude, -119.45821, 1e-4);
    ASSERT_TRUE(gm.getHours() == 19);
    ASSERT_TRUE(gm.getMinutes() == 30);

    // Malformed/truncated GLL sentences must be rejected safely without out-of-bounds reads
    gm.feedSentence("$GNGLL,38*12", 3200);
    gm.feedSentence("$GNGLL,,,,,", 3300);
    gm.feedSentence("$GNGLL", 3400);

    // Watchdog staleness across 32-bit millis() rollover
    gm.setHeading(90.0f, BEARING_FIXED, 0xFFFFFFF0UL);
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_FIXED);
    gm.checkStaleness(1500UL); // 1500 - 0xFFFFFFF0 = 1516 ms >= 1500 ms -> BEARING_LOST
    ASSERT_TRUE(gm.getBearingQuality() == BEARING_LOST);

    // Bidirectional TX command transmission
    std::string sentData;
    gm.setTxHandler([](const uint8_t *data, size_t len, void *ctx) {
        auto *s = static_cast<std::string*>(ctx);
        if (s) s->append(reinterpret_cast<const char*>(data), len);
    }, &sentData);
    ASSERT_TRUE(gm.sendCommand("$PCFG,SAVE"));
    ASSERT_STR_EQ(sentData.c_str(), "$PCFG,SAVE\r\n");

    // Malformed & truncated NMEA boundary safety
    gm.feedSentence("$GNGLL");
    gm.feedSentence("$GPGLL");
    gm.feedSentence("$GNGLL,");

    PASS();
}

void test_gps_tx_discovery() {
    TEST("GNSS TX discovery probe state machine, retries, and response parsing");
    GpsManager gm;

    // 1. Initial State
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Unknown);
    ASSERT_TRUE(!gm.isSendAvailable());
    ASSERT_TRUE(gm.getProbeRetries() == 0);
    ASSERT_TRUE(gm.getLastProbeTime() == 0);
    ASSERT_STR_EQ(gm.getTxStateString(), "UNKNOWN");

    std::string sentData;
    gm.setTxHandler([](const uint8_t *data, size_t len, void *ctx) {
        auto *s = static_cast<std::string*>(ctx);
        if (s) s->append(reinterpret_cast<const char*>(data), len);
    }, &sentData);

    // 2. Start Probe
    gm.probeTx(1000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(!gm.isSendAvailable());
    ASSERT_TRUE(gm.getLastProbeTime() == 1000);
    ASSERT_TRUE(gm.getProbeRetries() == 0);
    ASSERT_STR_EQ(gm.getTxStateString(), "PROBING");
    ASSERT_STR_EQ(sentData.c_str(), "version\r\n");

    // Normal NMEA or non-response streams must not trigger probe acknowledgment
    gm.feedSentence("$GNGLL,3821.2472,N,11927.4926,W,193000.00,A*3C", 1020);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(!gm.isSendAvailable());

    // 3. Receive <OK acknowledgment -> Available
    gm.feedSentence("<OK", 1050);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Available);
    ASSERT_TRUE(gm.isSendAvailable());
    ASSERT_STR_EQ(gm.getTxStateString(), "AVAILABLE");

    // 4. Manual re-probe from Available
    sentData.clear();
    gm.probeTx(2000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(!gm.isSendAvailable());
    ASSERT_TRUE(gm.getLastProbeTime() == 2000);
    ASSERT_STR_EQ(sentData.c_str(), "version\r\n");

    // Acknowledge with #VERSIONA log
    gm.feedSentence("#VERSIONA,91,GPS,FINE,2320,150123000,0,0,18,0;UM982,R4.10Build11252*9543ab12", 2100);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Available);
    ASSERT_TRUE(gm.isSendAvailable());

    // 5. Manual re-probe and $command response
    sentData.clear();
    gm.probeTx(3000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    gm.feedSentence("$command,version,response: OK*24", 3100);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Available);
    ASSERT_TRUE(gm.isSendAvailable());

    // 6. Byte-by-byte ingestion of acknowledgment (< OK with whitespace)
    sentData.clear();
    gm.probeTx(4000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    const char *replyWithSpace = "< OK\r\n";
    for (size_t i = 0; replyWithSpace[i]; ++i) {
        gm.feedByte((uint8_t)replyWithSpace[i], 4050);
    }
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Available);
    ASSERT_TRUE(gm.isSendAvailable());

    // 7. Timeout and Retry Sequence to Unavailable
    sentData.clear();
    gm.probeTx(5000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(gm.getProbeRetries() == 0);
    ASSERT_STR_EQ(sentData.c_str(), "version\r\n");

    // Update before timeout (500 ms elapsed)
    gm.update(5500);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(gm.getProbeRetries() == 0);

    // Timeout: Retry 1 at t = 6000 (1000 ms elapsed)
    gm.update(6000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(gm.getProbeRetries() == 1);
    ASSERT_TRUE(gm.getLastProbeTime() == 6000);
    ASSERT_STR_EQ(sentData.c_str(), "version\r\nversion\r\n");

    // Before second timeout (500 ms elapsed)
    gm.update(6500);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(gm.getProbeRetries() == 1);

    // Timeout: Retry 2 at t = 7000 (1000 ms elapsed)
    gm.update(7000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(gm.getProbeRetries() == 2);
    ASSERT_TRUE(gm.getLastProbeTime() == 7000);
    ASSERT_STR_EQ(sentData.c_str(), "version\r\nversion\r\nversion\r\n");

    // Before expiration (500 ms elapsed)
    gm.update(7500);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(gm.getProbeRetries() == 2);

    // Timeout: Max retries (2) exceeded at t = 8000 -> Unavailable
    gm.update(8000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Unavailable);
    ASSERT_TRUE(!gm.isSendAvailable());
    ASSERT_STR_EQ(gm.getTxStateString(), "UNAVAILABLE");
    // Ensure no additional probe was sent on transition to Unavailable
    ASSERT_STR_EQ(sentData.c_str(), "version\r\nversion\r\nversion\r\n");

    // Subsequent updates when Unavailable should not transmit
    gm.update(9000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Unavailable);
    ASSERT_STR_EQ(sentData.c_str(), "version\r\nversion\r\nversion\r\n");

    // 8. Manual re-probe from Unavailable
    sentData.clear();
    gm.probeTx(10000);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    ASSERT_TRUE(gm.getProbeRetries() == 0);
    ASSERT_TRUE(gm.getLastProbeTime() == 10000);
    ASSERT_STR_EQ(sentData.c_str(), "version\r\n");

    // 9. Response callback registration
    std::string lastResponse;
    gm.setOnResponse([](const char *resp, void *ctx) {
        auto *s = static_cast<std::string*>(ctx);
        if (s) *s = resp;
    }, &lastResponse);

    gm.feedSentence("<OK", 10050);
    ASSERT_STR_EQ(lastResponse.c_str(), "<OK");
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Available);

    gm.feedSentence("#MODE,ROVER,UAV", 10100);
    ASSERT_STR_EQ(lastResponse.c_str(), "#MODE,ROVER,UAV");

    // 10. Millis rollover handling
    gm.probeTx(0xFFFFFFF0UL);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);
    gm.update(1000UL); // 1016 ms elapsed across 0xFFFFFFFF rollover -> retry 1
    ASSERT_TRUE(gm.getProbeRetries() == 1);
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Probing);

    PASS();
}

void test_gps_config_pipeline() {
    TEST("GpsConfigPipeline sequential commands, safeguards, and stream verification");
    GpsManager gm;

    // Helper lambda to make TX available
    auto makeTxAvailable = [&gm]() {
        gm.probeTx(100);
        gm.feedSentence("<OK", 150);
        ASSERT_TRUE(gm.isSendAvailable());
    };

    // 1. TX unavailable rejection guard
    ASSERT_TRUE(gm.getTxState() == GpsTxState::Unknown);
    ASSERT_TRUE(!gm.startConfiguration(0.0f, 2.5f, true, 1000));
    ASSERT_TRUE(gm.getConfigPipeline().isFailed());
    ASSERT_TRUE(gm.getConfigPipeline().getFailureReason() == ConfigFailureReason::TxNotAvailable);
    ASSERT_STR_EQ(gm.getConfigPipeline().getFailureReasonString(), "TX_NOT_AVAILABLE");

    // Make TX available
    makeTxAvailable();

    std::string sentData;
    gm.setTxHandler([](const uint8_t *data, size_t len, void *ctx) {
        auto *s = static_cast<std::string*>(ctx);
        if (s) s->append(reinterpret_cast<const char*>(data), len);
    }, &sentData);

    // 2. Full successful configuration sequence without baseline constraint
    sentData.clear();
    ASSERT_TRUE(gm.startConfiguration(0.0f, 2.5f, true, 1000));
    auto &pipe = gm.getConfigPipeline();
    ASSERT_TRUE(pipe.isRunning());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::ModeRoverUav);
    ASSERT_STR_EQ(sentData.c_str(), "MODE ROVER UAV\r\n");

    // Acknowledge step 1 -> Step 2 CONFIG SIGNALGROUP 3 6
    sentData.clear();
    gm.feedSentence("<OK", 1050);
    ASSERT_TRUE(pipe.isRunning());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::ConfigSignalGroup);
    ASSERT_STR_EQ(sentData.c_str(), "CONFIG SIGNALGROUP 3 6\r\n");

    // Acknowledge step 2 -> Step 3 UNLOG COM1
    sentData.clear();
    gm.feedSentence("<OK", 1100);
    ASSERT_TRUE(pipe.isRunning());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::UnlogCom1);
    ASSERT_STR_EQ(sentData.c_str(), "UNLOG COM1\r\n");

    // Acknowledge step 3 -> Step 4 KSXT com1 0.2
    sentData.clear();
    gm.feedSentence("<OK", 1150);
    ASSERT_TRUE(pipe.isRunning());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::KsxtOutput);
    ASSERT_STR_EQ(sentData.c_str(), "KSXT com1 0.2\r\n");

    // Acknowledge step 4 -> Step 5 SAVECONFIG (baseline steps skipped)
    sentData.clear();
    gm.feedSentence("<OK", 1200);
    ASSERT_TRUE(pipe.isRunning());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::SaveConfig);
    ASSERT_STR_EQ(sentData.c_str(), "SAVECONFIG\r\n");

    // Acknowledge step 5 -> Step 6 VerifyStream
    sentData.clear();
    gm.feedSentence("<OK", 1250);
    ASSERT_TRUE(pipe.isRunning());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::VerifyStream);
    ASSERT_TRUE(pipe.isWaitingForStream());
    ASSERT_TRUE(sentData.empty()); // No command sent while waiting for stream

    // Feed a valid $KSXT sentence to complete stream verification
    char ksxtBuf[256];
    GpsSimulator::generateKsxt(ksxtBuf, sizeof(ksxtBuf),
                              "20261109120000.00", 37.7749, -122.4194,
                              90.0, BEARING_FIXED);
    gm.feedSentence(ksxtBuf, 1300);
    ASSERT_TRUE(pipe.isSuccess());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::Done);
    ASSERT_STR_EQ(pipe.getStateString(), "SUCCESS");

    // 3. Full configuration sequence WITH baseline length constraint (e.g. 101.6 cm, tol 2.5 cm)
    pipe.reset();
    ASSERT_TRUE(pipe.isIdle());
    sentData.clear();
    ASSERT_TRUE(gm.startConfiguration(101.6f, 2.5f, false, 2000)); // verifyStream = false
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::ModeRoverUav);
    ASSERT_STR_EQ(sentData.c_str(), "MODE ROVER UAV\r\n");

    sentData.clear();
    gm.feedSentence("<OK", 2050); // MODE -> SIGNALGROUP
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::ConfigSignalGroup);
    ASSERT_STR_EQ(sentData.c_str(), "CONFIG SIGNALGROUP 3 6\r\n");

    sentData.clear();
    gm.feedSentence("<OK", 2100); // SIGNALGROUP -> UNLOG COM1
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::UnlogCom1);
    ASSERT_STR_EQ(sentData.c_str(), "UNLOG COM1\r\n");

    sentData.clear();
    gm.feedSentence("<OK", 2150); // UNLOG -> KSXT
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::KsxtOutput);
    ASSERT_STR_EQ(sentData.c_str(), "KSXT com1 0.2\r\n");

    sentData.clear();
    gm.feedSentence("<OK", 2200); // KSXT -> CONFIG HEADING FIXLENGTH
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::HeadingFixLength);
    ASSERT_STR_EQ(sentData.c_str(), "CONFIG HEADING FIXLENGTH\r\n");

    sentData.clear();
    gm.feedSentence("<OK", 2250); // FIXLENGTH -> CONFIG HEADING LENGTH 101.6 2.5
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::HeadingLength);
    ASSERT_STR_EQ(sentData.c_str(), "CONFIG HEADING LENGTH 101.6 2.5\r\n");

    sentData.clear();
    gm.feedSentence("<OK", 2300); // LENGTH -> SAVECONFIG
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::SaveConfig);
    ASSERT_STR_EQ(sentData.c_str(), "SAVECONFIG\r\n");

    // Acknowledge SAVECONFIG -> with verifyStream = false, finishes immediately!
    sentData.clear();
    gm.feedSentence("<OK", 2350);
    ASSERT_TRUE(pipe.isSuccess());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::Done);

    // 4. Flash Write Safeguard: Abort on Command Rejection (SAVECONFIG must NEVER be sent)
    pipe.reset();
    sentData.clear();
    ASSERT_TRUE(gm.startConfiguration(0.0f, 2.5f, false, 3000));
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::ModeRoverUav);
    gm.feedSentence("<OK", 3050); // Advance to SIGNALGROUP
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::ConfigSignalGroup);

    // Receiver rejects command with <ERROR
    gm.feedSentence("<ERROR: Command syntax invalid", 3100);
    ASSERT_TRUE(pipe.isFailed());
    ASSERT_TRUE(pipe.getFailureReason() == ConfigFailureReason::CommandRejected);
    ASSERT_TRUE(pipe.getFailedStep() == ConfigStep::ConfigSignalGroup);
    ASSERT_STR_EQ(pipe.getFailureReasonString(), "COMMAND_REJECTED");

    // Critical assertion: SAVECONFIG must NOT appear in sentData!
    ASSERT_TRUE(sentData.find("SAVECONFIG") == std::string::npos);

    // 5. Flash Write Safeguard: Abort on Step Timeout (SAVECONFIG must NEVER be sent)
    pipe.reset();
    sentData.clear();
    ASSERT_TRUE(gm.startConfiguration(0.0f, 2.5f, false, 4000));
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::ModeRoverUav);
    gm.feedSentence("<OK", 4050); // Advance to SIGNALGROUP
    gm.feedSentence("<OK", 4100); // Advance to UNLOG COM1
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::UnlogCom1);

    // Time elapses: 500 ms (before timeout) -> still running
    gm.update(4600);
    ASSERT_TRUE(pipe.isRunning());
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::UnlogCom1);

    // Time elapses: 1000 ms elapsed -> Timeout!
    gm.update(5100);
    ASSERT_TRUE(pipe.isFailed());
    ASSERT_TRUE(pipe.getFailureReason() == ConfigFailureReason::StepTimeout);
    ASSERT_TRUE(pipe.getFailedStep() == ConfigStep::UnlogCom1);
    ASSERT_STR_EQ(pipe.getFailureReasonString(), "STEP_TIMEOUT");

    // Critical assertion: SAVECONFIG must NOT appear in sentData!
    ASSERT_TRUE(sentData.find("SAVECONFIG") == std::string::npos);

    // 6. Stream Verification Timeout
    pipe.reset();
    sentData.clear();
    ASSERT_TRUE(gm.startConfiguration(0.0f, 2.5f, true, 6000));
    gm.feedSentence("<OK", 6050); // MODE
    gm.feedSentence("<OK", 6100); // SIGNALGROUP
    gm.feedSentence("<OK", 6150); // UNLOG
    gm.feedSentence("<OK", 6200); // KSXT
    gm.feedSentence("<OK", 6250); // SAVECONFIG
    ASSERT_TRUE(pipe.getCurrentStep() == ConfigStep::VerifyStream);

    // Advance time: 1000 ms (< 1500 ms verify timeout) -> still waiting
    gm.update(7250);
    ASSERT_TRUE(pipe.isWaitingForStream());

    // Advance time: 1500 ms elapsed -> Stream verification timeout!
    gm.update(7751);
    ASSERT_TRUE(pipe.isFailed());
    ASSERT_TRUE(pipe.getFailureReason() == ConfigFailureReason::StreamVerifyTimeout);
    ASSERT_STR_EQ(pipe.getFailureReasonString(), "STREAM_VERIFY_TIMEOUT");

    // 7. Pipeline Progress Callback and String Helpers
    struct ProgressTracker {
        std::string msg;
        ConfigStep step = ConfigStep::None;
    } tracker;

    pipe.reset();
    pipe.setOnProgress([](ConfigStep step, ConfigPipelineState, const char *msg, void *ctx) {
        auto *t = static_cast<ProgressTracker*>(ctx);
        if (t) {
            if (msg) t->msg = msg;
            t->step = step;
        }
    }, &tracker);

    ASSERT_TRUE(gm.startConfiguration(0.0f, 2.5f, false, 8000));
    ASSERT_TRUE(tracker.step == ConfigStep::ModeRoverUav);
    ASSERT_TRUE(!tracker.msg.empty());

    // String helpers coverage
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::ModeRoverUav), "MODE ROVER UAV");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::ConfigSignalGroup), "CONFIG SIGNALGROUP 3 6");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::UnlogCom1), "UNLOG COM1");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::KsxtOutput), "KSXT com1 0.2");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::HeadingFixLength), "CONFIG HEADING FIXLENGTH");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::HeadingLength), "CONFIG HEADING LENGTH");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::SaveConfig), "SAVECONFIG");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::VerifyStream), "VERIFY STREAM");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::Done), "DONE");
    ASSERT_STR_EQ(GpsConfigPipeline::stepToString(ConfigStep::None), "NONE");

    pipe.setOnProgress(nullptr, nullptr);

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
    cli.setOutputHandler([](const char *str, void *ctx) {
        auto *out = static_cast<std::string*>(ctx);
        if (out) *out += str;
    }, &cliOutput);

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

    // Target Grid Hit Area (50..430, 70..200)
    ASSERT_TRUE(InputManager::isTargetGridTouched(50, 70));
    ASSERT_TRUE(InputManager::isTargetGridTouched(430, 200));
    ASSERT_TRUE(InputManager::isTargetGridTouched(200, 150));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(49, 150));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(431, 150));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(200, 69));
    ASSERT_TRUE(!InputManager::isTargetGridTouched(200, 201));

    // Target Grid Character Column Mapping (REMSTARTX=80, CHAR_W(REMSIZE)=48)
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(80) == 0);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(127) == 0);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(128) == 1);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(80 + 5 * 48) == 5);
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(30) == 0);   // Underflow clamp
    ASSERT_TRUE(InputManager::getTargetGridCharFromTouch(450) == 5);  // Overflow clamp

    // Calibration Pill Hit Area (180..300, 205..280)
    ASSERT_TRUE(InputManager::isCalPillTouched(180, 205));
    ASSERT_TRUE(InputManager::isCalPillTouched(300, 280));
    ASSERT_TRUE(InputManager::isCalPillTouched(240, 245));
    ASSERT_TRUE(!InputManager::isCalPillTouched(179, 245));
    ASSERT_TRUE(!InputManager::isCalPillTouched(301, 245));
    ASSERT_TRUE(!InputManager::isCalPillTouched(240, 204));
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

    im.injectEncoderButton(true);
    ASSERT_TRUE(im.pollEvent(evt));
    ASSERT_TRUE(evt.type == UserEventType::EncoderButtonPressed);

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

void test_grid_editor() {
    TEST("GridEditor character rolling, cursor, and inactivity timeout");
    GridEditor ge;
    ASSERT_STR_EQ(ge.getGrid(), "ZA00AA");
    ASSERT_TRUE(ge.isEditing());
    ASSERT_TRUE(ge.getCursorColumn() == 0);

    // Roll character up ('Z' -> 'A')
    ge.rollChar(+1, 1000);
    ASSERT_STR_EQ(ge.getGrid(), "AA00AA");

    // Roll character down ('A' -> 'Z')
    ge.rollChar(-1, 1100);
    ASSERT_STR_EQ(ge.getGrid(), "ZA00AA");

    // Select character column 2 ('0')
    ge.selectCharacter(2, 1200);
    ASSERT_TRUE(ge.getCursorColumn() == 2);
    ge.rollChar(+1, 1300); // '0' -> '1'
    ASSERT_STR_EQ(ge.getGrid(), "ZA10AA");

    // Clamping cursor column
    ge.setCursorColumn(10);
    ASSERT_TRUE(ge.getCursorColumn() == 5);
    ge.setCursorColumn(-5);
    ASSERT_TRUE(ge.getCursorColumn() == 0);

    // Inactivity timeout: 5000 ms
    ASSERT_TRUE(!ge.checkTimeout(1300 + 4999));
    ASSERT_TRUE(ge.isEditing());
    ASSERT_TRUE(ge.checkTimeout(1300 + 5001));
    ASSERT_TRUE(!ge.isEditing());

    PASS();
}

void test_app_controller() {
    TEST("AppController state coordination, geodesy sync, and event handling");
    Board board;
    DisplayUi displayUi;
    InputManager inputManager;
    GpsManager gpsManager;
    Calibration calibration;
    GpsSimulator gpsSimulator;
    SerialCli serialCli(gpsManager, calibration, gpsSimulator);

    AppController app(board, displayUi, inputManager, gpsManager, calibration, gpsSimulator, serialCli);
    app.begin();

    // Initial state
    ASSERT_STR_EQ(app.getMyGrid(), "JJ00AA00");
    ASSERT_STR_EQ(app.getTargetGrid(), "ZA00AA");
    ASSERT_TRUE(app.getUiMode() == UiMode::Normal);

    // Heading state setter & getter verification
    app.setHeadingState(true, BEARING_FIXED, 12345UL);
    ASSERT_TRUE(app.hasHeading());
    ASSERT_TRUE(app.getBearingQuality() == BEARING_FIXED);
    ASSERT_TRUE(app.getLastHeadingUpdate() == 12345UL);

    // GPS Location update through GPS Manager
    gpsManager.setLocation(38.35412, -119.45821, 1000);
    ASSERT_TRUE(app.getCurrentPosition().latitude == 38.35412);
    ASSERT_STR_EQ(app.getMyGrid(), "DM08GI54");
    ASSERT_TRUE(app.getDistance() > 10000000.0); // JJ/ZA distance is > 10,000 km

    // Event: Turn knob clockwise in edit mode (col 0: 'Z' -> 'A')
    UserEvent evRot;
    evRot.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evRot, 2000);
    ASSERT_STR_EQ(app.getTargetGrid(), "AA00AA");

    // Event: Tap Calibration Pill -> Transitions to Calibrating mode
    UserEvent evPill;
    evPill.type = UserEventType::TouchCalibrationPill;
    app.handleUserEvent(evPill, 2100);
    ASSERT_TRUE(app.getUiMode() == UiMode::Calibrating);

    // Bezel depress while CalModal is open: must be ignored and must NOT arm edit mode
    UserEvent evBezelPress;
    evBezelPress.type = UserEventType::EncoderButtonPressed;
    app.handleUserEvent(evBezelPress, 2150);
    ASSERT_TRUE(!app.getGridEditor().isEditing());
    ASSERT_TRUE(app.getUiMode() == UiMode::Calibrating);

    // Event: Turn knob clockwise in calibrating mode -> Nudges proposed offset +0.1 deg
    float initProposed = calibration.getProposedOffset();
    app.handleUserEvent(evRot, 2200);
    ASSERT_NEAR(calibration.getProposedOffset(), initProposed + 0.1f, 1e-4);

    // Event: Commit Save Button -> Persists offset and returns to Normal mode
    UserEvent evSave;
    evSave.type = UserEventType::SaveButtonCommitted;
    app.handleUserEvent(evSave, 2300);
    ASSERT_TRUE(app.getUiMode() == UiMode::Normal);
    ASSERT_NEAR(calibration.getOffset(), initProposed + 0.1f, 1e-4);

    // Event: Tap Local Grid -> Copies local station grid, disarms edit mode, and sets Rover Mode active
    UserEvent evLoc;
    evLoc.type = UserEventType::TouchLocalGrid;
    app.handleUserEvent(evLoc, 2400);
    ASSERT_STR_EQ(app.getTargetGrid(), "DM08GI");
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_TRUE(!app.getGridEditor().isEditing());
    // Distance to origin is exact 0.0 m physically (not quantized to grid center!)
    ASSERT_NEAR(app.getDistance(), 0.0, 1e-3);

    // Bezel rotation during rover transit: does NOT roll characters and does NOT disarm rover mode
    UserEvent evBump;
    evBump.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evBump, 2450);
    ASSERT_STR_EQ(app.getTargetGrid(), "DM08GI");
    ASSERT_TRUE(app.isRoverMode());

    // Rover transit movement: 10.0 km physical displacement
    GeoPoint originPos = app.getCurrentPosition();
    GeoPoint displaced10km = GpsSimulator::calculateRoverDisplacement(originPos, 10.0, 45.0);
    gpsManager.setLocation(displaced10km, 2500);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_NEAR(app.getDistance() / 1000.0, 10.0, 0.05);

    // Rover transit movement: 18.0 km physical displacement (alert clears >= 16 km)
    GeoPoint displaced18km = GpsSimulator::calculateRoverDisplacement(originPos, 18.0, 45.0);
    gpsManager.setLocation(displaced18km, 2600);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_NEAR(app.getDistance() / 1000.0, 18.0, 0.05);

    // 2-Step exit: Touch Target Grid character selects char and arms editing, but Rover Mode remains active
    UserEvent evChar;
    evChar.type = UserEventType::TouchGridCharacter;
    evChar.gridIndex = 2;
    app.handleUserEvent(evChar, 2700);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_TRUE(app.getGridEditor().isEditing());

    // Step 2: First bezel rotation while editing commits stop and disarms Rover Mode
    UserEvent evRoll;
    evRoll.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evRoll, 2750);
    ASSERT_TRUE(!app.isRoverMode());
    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Bearing);

    // Non-blocking modal button unpress timer across 32-bit millis() rollover
    UserEvent evSnap;
    evSnap.type = UserEventType::SnapOffsetTriggered;
    app.handleUserEvent(evSnap, 0xFFFFFFF0UL);
    app.update(0xFFFFFFF0UL + 50); // 50 ms elapsed
    app.update(50UL);              // 56 + 50 = 106 ms elapsed (< 120 ms)
    app.update(100UL);             // 56 + 100 = 156 ms elapsed (>= 120 ms -> released)

    // Decoupled geodesy calculation and navigation rendering
    double prevDist = app.getDistance();
    app.getGridEditor().setGrid("EL29ja");
    // Model state distance remains unchanged until recalculateGeodesy() is called
    ASSERT_NEAR(app.getDistance(), prevDist, 1e-4);
    app.recalculateGeodesy();
    ASSERT_TRUE(app.getDistance() > 2000000.0);
#if !ENABLE_ARRL_DIST
    double expectedDistMeters = calculateArrlContestDistance(app.getMyGrid(), "EL29ja") * 1000.0;
    ASSERT_NEAR(app.getDistance(), expectedDistMeters, 1e-4);

    // Same grid contest distance rule in stationary mode (1.0 km minimum floor)
    app.setMyGrid("CM87wa00");
    app.getGridEditor().setGrid("CM87wa");
    app.recalculateGeodesy();
    ASSERT_NEAR(app.getDistance(), 1000.0, 1e-4);
    char bufRev[24];
    DisplayUi::formatReverseBearingAndDistance(app.getReverseBearing(), app.getDistance(), bufRev, sizeof(bufRev));
    ASSERT_TRUE(std::strstr(bufRev, "   1 k  ") != nullptr);
#endif
    app.renderNavigationRows();
    app.updateNavigation();

    // Verify self-testing mode suppression of live updates
    ASSERT_TRUE(!app.isSelfTesting());
    app.setSelfTesting(true);
    ASSERT_TRUE(app.isSelfTesting());
    GeoPoint posBefore = app.getCurrentPosition();
    gpsManager.setLocation(GeoPoint(40.1234, -120.5678, true), 3000);
    ASSERT_NEAR(app.getCurrentPosition().latitude, posBefore.latitude, 1e-5);
    ASSERT_NEAR(app.getCurrentPosition().longitude, posBefore.longitude, 1e-5);
    app.setSelfTesting(false);
    ASSERT_TRUE(!app.isSelfTesting());

    PASS();
}

void test_self_test_assertions() {
    TEST("SelfTest 9-stage invariant assertions validation (including Rover capabilities)");
    int failedTests[16];
    int failedCount = SelfTest::runAssertions(failedTests, sizeof(failedTests) / sizeof(failedTests[0]));
    ASSERT_TRUE(failedCount == 0);
    PASS();
}

void test_rover_stop_model() {
    TEST("RoverStop data model and operators");
    RoverStop s1;
    ASSERT_TRUE(s1.timestamp == 0);
    ASSERT_NEAR(s1.latitude, 0.0, 1e-9);
    ASSERT_NEAR(s1.longitude, 0.0, 1e-9);
    ASSERT_STR_EQ(s1.grid, "");
    ASSERT_NEAR(s1.displacement_km, 0.0f, 1e-6);
    ASSERT_TRUE(!s1.qualified_16km);

    RoverStop s2(1730000000, 37.421998, -122.084000, "CM87wa22", 24.25f, true);
    ASSERT_TRUE(s2.timestamp == 1730000000);
    ASSERT_NEAR(s2.latitude, 37.421998, 1e-6);
    ASSERT_NEAR(s2.longitude, -122.084000, 1e-6);
    ASSERT_STR_EQ(s2.grid, "CM87wa22");
    ASSERT_NEAR(s2.displacement_km, 24.25f, 1e-4);
    ASSERT_TRUE(s2.qualified_16km);

    RoverStop s3 = s2;
    ASSERT_TRUE(s2 == s3);
    ASSERT_TRUE(!(s2 != s3));

    s3.qualified_16km = false;
    ASSERT_TRUE(s2 != s3);

    // Overlong grid string safety
    RoverStop s4(100, 10.0, 20.0, "1234567890ABCDEF", 5.0f, false);
    ASSERT_TRUE(std::strlen(s4.grid) == 8);
    ASSERT_STR_EQ(s4.grid, "12345678");

    // Null grid safety
    s4.setGrid(nullptr);
    ASSERT_STR_EQ(s4.grid, "");

    // Assert deterministic packed struct size (34 bytes: 4 + 8 + 8 + 9 + 4 + 1)
    ASSERT_TRUE(sizeof(RoverStop) == 34);

    PASS();
}

void test_rover_log_manager_stack_ops() {
    TEST("RoverLogManager stack operations, inspection, and sequential undo");
    RoverLogManager::resetHostStorage();

    RoverLogManager mgr;
    mgr.begin();

    // Initial empty state
    ASSERT_TRUE(mgr.isEmpty());
    ASSERT_TRUE(mgr.getStopCount() == 0);
    ASSERT_TRUE(mgr.getLastStop() == nullptr);
    ASSERT_TRUE(mgr.getPriorStop() == nullptr);
    ASSERT_TRUE(mgr.getStop(0) == nullptr);

    RoverStop temp;
    ASSERT_TRUE(!mgr.getLastStop(temp));
    ASSERT_TRUE(!mgr.getPriorStop(temp));
    ASSERT_TRUE(!mgr.getStop(0, temp));

    // Empty stack pop protection
    ASSERT_TRUE(!mgr.popLastStop());
    ASSERT_TRUE(mgr.getStopCount() == 0);

    // Append Stop 1 (Origin)
    RoverStop stop1(1000, 37.421998, -122.084000, "CM87wa22", 0.0f, false);
    ASSERT_TRUE(mgr.appendStop(stop1));
    ASSERT_TRUE(!mgr.isEmpty());
    ASSERT_TRUE(mgr.getStopCount() == 1);
    ASSERT_TRUE(mgr.getLastStop() != nullptr);
    ASSERT_TRUE(*mgr.getLastStop() == stop1);
    ASSERT_TRUE(mgr.getPriorStop() == nullptr); // only 1 stop
    ASSERT_TRUE(mgr.getStop(0) != nullptr && *mgr.getStop(0) == stop1);
    ASSERT_TRUE(mgr.getStop(1) == nullptr);

    // Append Stop 2 (First transit destination)
    RoverStop stop2(2000, 37.581204, -122.215401, "CM87wb44", 24.2f, true);
    ASSERT_TRUE(mgr.appendStop(stop2));
    ASSERT_TRUE(mgr.getStopCount() == 2);
    ASSERT_TRUE(mgr.getLastStop() != nullptr && *mgr.getLastStop() == stop2);
    ASSERT_TRUE(mgr.getPriorStop() != nullptr && *mgr.getPriorStop() == stop1);

    // Append Stop 3 (Second transit destination)
    RoverStop stop3(3000, 37.774929, -122.419418, "CM88ec12", 28.5f, true);
    ASSERT_TRUE(mgr.appendStop(stop3));
    ASSERT_TRUE(mgr.getStopCount() == 3);
    ASSERT_TRUE(mgr.getLastStop() != nullptr && *mgr.getLastStop() == stop3);
    ASSERT_TRUE(mgr.getPriorStop() != nullptr && *mgr.getPriorStop() == stop2);

    ASSERT_TRUE(mgr.getStop(0, temp) && temp == stop1);
    ASSERT_TRUE(mgr.getStop(1, temp) && temp == stop2);
    ASSERT_TRUE(mgr.getStop(2, temp) && temp == stop3);
    ASSERT_TRUE(!mgr.getStop(3, temp));

    // Range-based for loop traversal via getStops()
    size_t iterCount = 0;
    uint32_t expectedTs[3] = {1000, 2000, 3000};
    for (const auto &stop : mgr.getStops()) {
        ASSERT_TRUE(stop.timestamp == expectedTs[iterCount]);
        iterCount++;
    }
    ASSERT_TRUE(iterCount == 3);

    // Sequential Undo / Pop
    // Pop Stop 3 -> fallback to Stop 2
    ASSERT_TRUE(mgr.popLastStop());
    ASSERT_TRUE(mgr.getStopCount() == 2);
    ASSERT_TRUE(mgr.getLastStop() != nullptr && *mgr.getLastStop() == stop2);
    ASSERT_TRUE(mgr.getPriorStop() != nullptr && *mgr.getPriorStop() == stop1);

    // Pop Stop 2 -> fallback to Stop 1
    ASSERT_TRUE(mgr.popLastStop());
    ASSERT_TRUE(mgr.getStopCount() == 1);
    ASSERT_TRUE(mgr.getLastStop() != nullptr && *mgr.getLastStop() == stop1);
    ASSERT_TRUE(mgr.getPriorStop() == nullptr);

    // Pop Stop 1 -> stack empty
    ASSERT_TRUE(mgr.popLastStop());
    ASSERT_TRUE(mgr.isEmpty());
    ASSERT_TRUE(mgr.getStopCount() == 0);
    ASSERT_TRUE(mgr.getLastStop() == nullptr);
    ASSERT_TRUE(mgr.getPriorStop() == nullptr);

    // Pop again on empty stack returns false safely
    ASSERT_TRUE(!mgr.popLastStop());
    ASSERT_TRUE(mgr.isEmpty());

    PASS();
}

void test_rover_log_manager_duplicate_grid_search() {
    TEST("RoverLogManager duplicate grid subsquare search and casing");
    RoverLogManager::resetHostStorage();

    RoverLogManager mgr;
    mgr.begin();

    // Empty log search
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM87wa"));
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM87wa22"));
    ASSERT_TRUE(!mgr.hasVisitedGrid(nullptr));
    ASSERT_TRUE(!mgr.hasVisitedGrid(""));
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM87")); // < 6 chars

    // Add stops
    RoverStop stop1(1000, 37.42, -122.08, "CM87wa22", 0.0f, false);
    RoverStop stop2(2000, 37.58, -122.21, "CM87wb44", 24.2f, true);
    RoverStop stop3(3000, 37.77, -122.41, "CM88ec12", 28.5f, true);
    mgr.appendStop(stop1);
    mgr.appendStop(stop2);
    mgr.appendStop(stop3);

    // Subsquares in log (6-character check)
    ASSERT_TRUE(mgr.hasVisitedGrid("CM87wa"));
    ASSERT_TRUE(mgr.hasVisitedGrid("CM87wb"));
    ASSERT_TRUE(mgr.hasVisitedGrid("CM88ec"));

    // Case-insensitivity assertions
    ASSERT_TRUE(mgr.hasVisitedGrid("cm87wa"));
    ASSERT_TRUE(mgr.hasVisitedGrid("Cm87Wb"));
    ASSERT_TRUE(mgr.hasVisitedGrid("cm88EC"));

    // 8-character inputs matching 6-character subsquare
    ASSERT_TRUE(mgr.hasVisitedGrid("CM87wa99"));
    ASSERT_TRUE(mgr.hasVisitedGrid("cm87wb00"));

    // Grids not visited
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM87wc"));
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM87aa"));
    ASSERT_TRUE(!mgr.hasVisitedGrid("FN31aa"));
    ASSERT_TRUE(!mgr.hasVisitedGrid("DM04ms"));

    // Discarding a stop removes its grid from visited history
    ASSERT_TRUE(mgr.popLastStop()); // removes CM88ec
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM88ec"));
    ASSERT_TRUE(mgr.hasVisitedGrid("CM87wb")); // still visited
    ASSERT_TRUE(mgr.hasVisitedGrid("CM87wa")); // still visited

    // Clear history removes all visited grids
    mgr.clearHistory();
    ASSERT_TRUE(mgr.isEmpty());
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM87wa"));
    ASSERT_TRUE(!mgr.hasVisitedGrid("CM87wb"));

    PASS();
}

void test_rover_log_manager_capacity_and_nvs() {
    TEST("RoverLogManager capacity clamping, FIFO rollover, and NVS persistence");
    RoverLogManager::resetHostStorage();

    // 1. Factory defaults and bounds clamping
    RoverLogManager mgr1;
    mgr1.begin();
    ASSERT_TRUE(mgr1.getMaxStops() == DEFAULT_MAX_ROVER_STOPS); // default 100

    mgr1.setMaxStops(5); // below MIN_ROVER_STOPS (10)
    ASSERT_TRUE(mgr1.getMaxStops() == MIN_ROVER_STOPS);

    mgr1.setMaxStops(999); // above ABS_MAX_ROVER_STOPS (500)
    ASSERT_TRUE(mgr1.getMaxStops() == ABS_MAX_ROVER_STOPS);

    mgr1.setMaxStops(10, true);
    ASSERT_TRUE(mgr1.getMaxStops() == 10);

    // 2. FIFO Rollover at capacity
    // Fill to capacity of 10 stops
    for (uint32_t i = 1; i <= 10; ++i) {
        char g[9];
        snprintf(g, sizeof(g), "CM87w%c11", (char)('a' + (i - 1)));
        RoverStop s(1000 + i, 37.0 + i * 0.1, -122.0, g, (float)(i * 5.0), true);
        ASSERT_TRUE(mgr1.appendStop(s));
    }
    ASSERT_TRUE(mgr1.getStopCount() == 10);
    ASSERT_TRUE(mgr1.getStop(0)->timestamp == 1001);
    ASSERT_TRUE(mgr1.getLastStop()->timestamp == 1010);

    // Append 11th stop -> stop 1 rolls over, stop 2 becomes index 0
    RoverStop s11(1011, 38.1, -122.0, "CM88aa11", 55.0f, true);
    ASSERT_TRUE(mgr1.appendStop(s11));
    ASSERT_TRUE(mgr1.getStopCount() == 10);
    ASSERT_TRUE(mgr1.getStop(0)->timestamp == 1002);
    ASSERT_TRUE(mgr1.getLastStop()->timestamp == 1011);
    ASSERT_TRUE(mgr1.getPriorStop()->timestamp == 1010);

    // Append 12th stop -> stop 2 rolls over, stop 3 becomes index 0
    RoverStop s12(1012, 38.2, -122.0, "CM88ab11", 60.0f, true);
    ASSERT_TRUE(mgr1.appendStop(s12));
    ASSERT_TRUE(mgr1.getStopCount() == 10);
    ASSERT_TRUE(mgr1.getStop(0)->timestamp == 1003);
    ASSERT_TRUE(mgr1.getLastStop()->timestamp == 1012);

    // 3. NVS Persistence across instances
    // A fresh RoverLogManager instance loading from NVS must recover state exactly
    RoverLogManager mgr2;
    mgr2.begin();
    ASSERT_TRUE(mgr2.getMaxStops() == 10);
    ASSERT_TRUE(mgr2.getStopCount() == 10);
    ASSERT_TRUE(mgr2.getStop(0)->timestamp == 1003);
    ASSERT_TRUE(mgr2.getLastStop()->timestamp == 1012);
    ASSERT_TRUE(mgr2.getPriorStop()->timestamp == 1011);
    ASSERT_TRUE(mgr2.hasVisitedGrid("CM88ab"));
    ASSERT_TRUE(!mgr2.hasVisitedGrid("CM87wa")); // rolled over!

    // 4. Truncation on capacity reduction
    mgr2.setMaxStops(15, true); // expand capacity to 15 without data loss
    ASSERT_TRUE(mgr2.getMaxStops() == 15);
    ASSERT_TRUE(mgr2.getStopCount() == 10);

    // Append 5 more stops up to 15
    for (uint32_t i = 13; i <= 17; ++i) {
        char g[9];
        snprintf(g, sizeof(g), "CM88a%c11", (char)('a' + (i - 13)));
        RoverStop s(1000 + i, 38.0 + (i - 12) * 0.1, -122.0, g, (float)(i * 5.0), true);
        ASSERT_TRUE(mgr2.appendStop(s));
    }
    ASSERT_TRUE(mgr2.getStopCount() == 15);
    ASSERT_TRUE(mgr2.getStop(0)->timestamp == 1003);
    ASSERT_TRUE(mgr2.getLastStop()->timestamp == 1017);

    // Now reduce capacity from 15 down to 10 (valid clamped range 10-500) -> 5 oldest dropped
    mgr2.setMaxStops(10, true);
    ASSERT_TRUE(mgr2.getMaxStops() == 10);
    ASSERT_TRUE(mgr2.getStopCount() == 10);
    ASSERT_TRUE(mgr2.getStop(0)->timestamp == 1008);
    ASSERT_TRUE(mgr2.getLastStop()->timestamp == 1017);

    // Another instance loads truncated state
    RoverLogManager mgr3;
    mgr3.begin();
    ASSERT_TRUE(mgr3.getMaxStops() == 10);
    ASSERT_TRUE(mgr3.getStopCount() == 10);
    ASSERT_TRUE(mgr3.getStop(0)->timestamp == 1008);
    ASSERT_TRUE(mgr3.getLastStop()->timestamp == 1017);

    // 5. Clear history resets stops but preserves max_stops capacity
    mgr3.clearHistory();
    ASSERT_TRUE(mgr3.isEmpty());
    ASSERT_TRUE(mgr3.getStopCount() == 0);
    ASSERT_TRUE(mgr3.getMaxStops() == 10);

    RoverLogManager mgr4;
    mgr4.begin();
    ASSERT_TRUE(mgr4.isEmpty());
    ASSERT_TRUE(mgr4.getStopCount() == 0);
    ASSERT_TRUE(mgr4.getMaxStops() == 10);

    PASS();
}

void test_rover_card_evaluator() {
    TEST("RoverCard 4-state qualification evaluator and backtrack reversibility");

    // State A: In-Transit Under 16 km (Displacement < 16.0 km, GPS fresh)
    ASSERT_TRUE(evaluateRoverState(0.0, false, true) == RoverState::InTransitUnder16k);
    ASSERT_TRUE(evaluateRoverState(5.5, false, true) == RoverState::InTransitUnder16k);
    ASSERT_TRUE(evaluateRoverState(15.9, false, true) == RoverState::InTransitUnder16k);
    ASSERT_TRUE(evaluateRoverState(15.99, true, true) == RoverState::InTransitUnder16k); // revisit under 16km is still State A

    // State B: Qualified New Location (Displacement >= 16.0 km, unvisited, GPS fresh)
    ASSERT_TRUE(evaluateRoverState(16.0, false, true) == RoverState::QualifiedNewGrid);
    ASSERT_TRUE(evaluateRoverState(18.2, false, true) == RoverState::QualifiedNewGrid);
    ASSERT_TRUE(evaluateRoverState(45.0, false, true) == RoverState::QualifiedNewGrid);

    // State C: Qualified Revisited Grid (Displacement >= 16.0 km, visited earlier, GPS fresh)
    ASSERT_TRUE(evaluateRoverState(16.0, true, true) == RoverState::QualifiedRevisitedGrid);
    ASSERT_TRUE(evaluateRoverState(22.4, true, true) == RoverState::QualifiedRevisitedGrid);
    ASSERT_TRUE(evaluateRoverState(60.0, true, true) == RoverState::QualifiedRevisitedGrid);

    // State D: GPS Signal Lost / Stale (GPS not fresh, holds last valid value)
    ASSERT_TRUE(evaluateRoverState(5.0, false, false) == RoverState::GpsLostStale);
    ASSERT_TRUE(evaluateRoverState(18.2, false, false) == RoverState::GpsLostStale);
    ASSERT_TRUE(evaluateRoverState(22.4, true, false) == RoverState::GpsLostStale);

    // Dynamic Backtrack: Instantaneous, hysteresis-free reversibility
    // Forward drive
    ASSERT_TRUE(evaluateRoverState(15.9, false, true) == RoverState::InTransitUnder16k);
    ASSERT_TRUE(evaluateRoverState(16.0, false, true) == RoverState::QualifiedNewGrid);
    ASSERT_TRUE(evaluateRoverState(16.2, false, true) == RoverState::QualifiedNewGrid);

    // Canyon switchback retreat: instantaneous drop from Green (State B) to Red (State A)
    ASSERT_TRUE(evaluateRoverState(15.8, false, true) == RoverState::InTransitUnder16k);
    ASSERT_TRUE(evaluateRoverState(14.2, false, true) == RoverState::InTransitUnder16k);

    PASS();
}

void test_rover_card_formatting_and_colors() {
    TEST("RoverCard exact 16-char formatting, buffer safety, and theme colors");

    // Colors
    ASSERT_TRUE(getRoverStateTextColor(RoverState::InTransitUnder16k) == 0xF800);
    ASSERT_TRUE(getRoverStateTextColor(RoverState::QualifiedNewGrid) == 0x03E0);
    ASSERT_TRUE(getRoverStateTextColor(RoverState::QualifiedRevisitedGrid) == 0x011C);
    ASSERT_TRUE(getRoverStateTextColor(RoverState::GpsLostStale) == 0xDC00);

    // Distance Field Colors: calculated directly from displacement and revisit status
    ASSERT_TRUE(calculateRoverDistanceColor(0.0, false) == COLOR_ROVER_TRANSIT_RED);
    ASSERT_TRUE(calculateRoverDistanceColor(6.4, false) == COLOR_ROVER_TRANSIT_RED);
    ASSERT_TRUE(calculateRoverDistanceColor(15.9, false) == COLOR_ROVER_TRANSIT_RED);
    ASSERT_TRUE(calculateRoverDistanceColor(15.99, true) == COLOR_ROVER_TRANSIT_RED); // Revisit under 16km is still Transit Red
    ASSERT_TRUE(calculateRoverDistanceColor(16.0, false) == COLOR_ROVER_QUALIFIED_GREEN);
    ASSERT_TRUE(calculateRoverDistanceColor(18.2, false) == COLOR_ROVER_QUALIFIED_GREEN);
    ASSERT_TRUE(calculateRoverDistanceColor(16.0, true) == COLOR_ROVER_REVISIT_BLUE);
    ASSERT_TRUE(calculateRoverDistanceColor(22.4, true) == COLOR_ROVER_REVISIT_BLUE);

    // Status Words (9 characters)
    ASSERT_STR_EQ(getRoverStateStatusWord(RoverState::InTransitUnder16k), "TRANSIT  ");
    ASSERT_STR_EQ(getRoverStateStatusWord(RoverState::QualifiedNewGrid), "QUALIFIED");
    ASSERT_STR_EQ(getRoverStateStatusWord(RoverState::QualifiedRevisitedGrid), "REVISIT  ");
    ASSERT_STR_EQ(getRoverStateStatusWord(RoverState::GpsLostStale), "NO GPS   ");

    char buf[32];

    // State A: "TRANSIT    6.4 k"
    ASSERT_TRUE(formatRoverCard(RoverState::InTransitUnder16k, 6.4, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "TRANSIT    6.4 k");

    // State B: "QUALIFIED 18.2 k"
    ASSERT_TRUE(formatRoverCard(RoverState::QualifiedNewGrid, 18.2, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "QUALIFIED 18.2 k");

    // State C: "REVISIT   22.4 k"
    ASSERT_TRUE(formatRoverCard(RoverState::QualifiedRevisitedGrid, 22.4, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "REVISIT   22.4 k");

    // State D: "NO GPS    11.2 k"
    ASSERT_TRUE(formatRoverCard(RoverState::GpsLostStale, 11.2, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "NO GPS    11.2 k");

    // Boundary cases: 0.0 km
    ASSERT_TRUE(formatRoverCard(RoverState::InTransitUnder16k, 0.0, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "TRANSIT    0.0 k");

    // Negative clamping to 0.0 km
    ASSERT_TRUE(formatRoverCard(RoverState::InTransitUnder16k, -5.0, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "TRANSIT    0.0 k");

    // Max 3-digit distance (999.9 km)
    ASSERT_TRUE(formatRoverCard(RoverState::QualifiedNewGrid, 999.9, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "QUALIFIED999.9 k");

    // Overflow clamping to 999.9 km
    ASSERT_TRUE(formatRoverCard(RoverState::QualifiedNewGrid, 1250.0, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "QUALIFIED999.9 k");

    // Boundary floor-truncation assertions:
    // 15.96 km must truncate to 15.9 k (never round up to 16.0 k while still under 16 km)
    ASSERT_TRUE(formatRoverCard(RoverState::InTransitUnder16k, 15.96, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "TRANSIT   15.9 k");

    ASSERT_TRUE(formatRoverCard(RoverState::InTransitUnder16k, 15.999, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "TRANSIT   15.9 k");

    // Exactly 16.0 km renders 16.0 k
    ASSERT_TRUE(formatRoverCard(RoverState::QualifiedNewGrid, 16.0, buf, sizeof(buf)));
    ASSERT_TRUE(std::strlen(buf) == 16);
    ASSERT_STR_EQ(buf, "QUALIFIED 16.0 k");

    // Buffer safety checks
    char smallBuf[16]; // Too small (needs 17 for 16 chars + null terminator)
    ASSERT_TRUE(!formatRoverCard(RoverState::InTransitUnder16k, 6.4, smallBuf, sizeof(smallBuf)));
    ASSERT_TRUE(!formatRoverCard(RoverState::InTransitUnder16k, 6.4, nullptr, 32));

    PASS();
}

void test_rover_history_modal_formatters() {
    TEST("RoverHistoryModal string formatters and buffer safety");

    char buf[48];

    // Null / empty stop
    RoverHistoryModal::formatLastStop(0, nullptr, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Last Stop: None");

    RoverHistoryModal::formatDisplacementStatus(nullptr, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Displacement: 0.0 km (NONE)");

    RoverHistoryModal::formatTotalStops(0, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Total Logged Stops: 0");

    // Origin stop (Stop 1, 0.0 km, not qualified)
    // 14:15 UTC = 14 * 3600 + 15 * 60 = 50400 + 900 = 51300 s
    RoverStop stop1(51300, 37.421998, -122.084000, "CM87wa22", 0.0f, false);
    RoverHistoryModal::formatLastStop(1, &stop1, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Last: #1 - CM87wa22 (14:15Z)");

    RoverHistoryModal::formatDisplacementStatus(&stop1, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Disp:   0.0 km (ORIGIN)");

    // Qualified move (Stop 2, 24.2 km)
    // 16:32 UTC = 16 * 3600 + 32 * 60 = 57600 + 1920 = 59520 s
    RoverStop stop2(59520, 37.581204, -122.215401, "CM87wb44", 24.2f, true);
    RoverHistoryModal::formatLastStop(2, &stop2, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Last: #2 - CM87wb44 (16:32Z)");

    RoverHistoryModal::formatDisplacementStatus(&stop2, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Disp:  24.2 km (QUALIFIED)");

    RoverHistoryModal::formatTotalStops(4, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Total Logged Stops: 4");

    // Under 16k uncompleted / partial stop
    RoverStop stopUnder(60000, 37.5, -122.1, "CM87wb11", 12.5f, false);
    RoverHistoryModal::formatDisplacementStatus(&stopUnder, buf, sizeof(buf));
    ASSERT_STR_EQ(buf, "Disp:  12.5 km (UNDER 16k)");

    // Null buffer safety
    RoverHistoryModal::formatLastStop(1, &stop1, nullptr, 0);
    RoverHistoryModal::formatDisplacementStatus(&stop1, nullptr, 0);
    RoverHistoryModal::formatTotalStops(1, nullptr, 0);

    PASS();
}

void test_rover_input_gestures_and_hit_testing() {
    TEST("InputManager Rover touch hit-testing, Row 1 long-press, and history modal events");

    InputManager im;
    im.begin();

    // Hit-testing in Rover History Modal
    ASSERT_TRUE(InputManager::getRoverModalButtonFromTouch(ROVER_BTN_DISCARD_X + 10, ROVER_BTN_DISCARD_Y + 10) == ModalButtonId::Discard);
    ASSERT_TRUE(InputManager::getRoverModalButtonFromTouch(ROVER_BTN_CANCEL_X + 10, ROVER_BTN_CANCEL_Y + 10) == ModalButtonId::Cancel);
    ASSERT_TRUE(InputManager::getRoverModalButtonFromTouch(10, 10) == ModalButtonId::None);

    // Row 1 Short Tap (< 1500 ms) in Normal mode -> TouchLocalGrid
    im.injectTouch(true, 200, 35, 1000); // Touch down on Row 1
    im.injectTouch(false, 0, 0, 1300);   // Release after 300 ms (< 1500 ms)
    UserEvent ev;
    ASSERT_TRUE(im.pollEvent(ev));
    ASSERT_TRUE(ev.type == UserEventType::TouchLocalGrid);
    ASSERT_TRUE(!im.pollEvent(ev));

    // Row 1 Long Press (>= 1500 ms) in Normal mode -> LongPressLocalGridTriggered
    im.injectTouch(true, 200, 35, 2000); // Touch down
    im.injectTouch(true, 200, 35, 3000); // Held for 1000 ms -> no event yet
    ASSERT_TRUE(!im.pollEvent(ev));
    im.injectTouch(true, 200, 35, 3500); // Held for 1500 ms -> triggers long-press!
    ASSERT_TRUE(im.pollEvent(ev));
    ASSERT_TRUE(ev.type == UserEventType::LongPressLocalGridTriggered);

    // Release after long-press does NOT trigger short tap
    im.injectTouch(false, 0, 0, 3600);
    ASSERT_TRUE(!im.pollEvent(ev));

    // RoverHistory mode button touch and release
    im.setUiMode(UiMode::RoverHistory);

    // Press Discard button (short press does NOT discard, reports hold progress)
    im.injectTouch(true, ROVER_BTN_DISCARD_X + 20, ROVER_BTN_DISCARD_Y + 10, 4000);
    im.injectTouch(true, ROVER_BTN_DISCARD_X + 20, ROVER_BTN_DISCARD_Y + 10, 4600); // 600 ms / 1500 ms = 0.4f progress
    ASSERT_TRUE(im.pollEvent(ev));
    ASSERT_TRUE(ev.type == UserEventType::HoldDiscardButtonProgress);
    ASSERT_NEAR(ev.progress, 0.4f, 0.05f);

    // Released early -> cancels hold, progress 0.0, NO discard!
    im.injectTouch(false, 0, 0, 4700);
    ASSERT_TRUE(im.pollEvent(ev));
    ASSERT_TRUE(ev.type == UserEventType::HoldDiscardButtonProgress && ev.progress == 0.0f);
    ASSERT_TRUE(im.pollEvent(ev));
    ASSERT_TRUE(ev.type == UserEventType::ModalButtonUp);

    // Hold Discard for full LONG_PRESS_MS (1500 ms) -> DiscardLastStopTriggered committed
    im.injectTouch(true, ROVER_BTN_DISCARD_X + 20, ROVER_BTN_DISCARD_Y + 10, 4800);
    im.injectTouch(true, ROVER_BTN_DISCARD_X + 20, ROVER_BTN_DISCARD_Y + 10, 6400); // 1600 ms >= 1500 ms
    bool gotDiscard = false;
    while (im.pollEvent(ev)) {
        if (ev.type == UserEventType::DiscardLastStopTriggered) gotDiscard = true;
    }
    ASSERT_TRUE(gotDiscard);

    // Release finger after discard
    im.injectTouch(false, 0, 0, 6450);
    while (im.pollEvent(ev)) {}

    // Press Cancel button
    im.injectTouch(true, ROVER_BTN_CANCEL_X + 20, ROVER_BTN_CANCEL_Y + 10, 6600);
    ASSERT_TRUE(im.pollEvent(ev));
    ASSERT_TRUE(ev.type == UserEventType::ModalButtonDown && ev.buttonId == ModalButtonId::Cancel);

    // Release Cancel button -> CancelModalTriggered
    im.injectTouch(false, 0, 0, 6700);
    ASSERT_TRUE(im.pollEvent(ev));
    ASSERT_TRUE(ev.type == UserEventType::CancelModalTriggered);

    PASS();
}

void test_rover_app_controller_transitions_and_natural_exit() {
    TEST("AppController Rover Mode transitions, 2-step natural exit, and inactivity timeout");

    RoverLogManager::resetHostStorage();

    Board board;
    DisplayUi displayUi;
    InputManager inputManager;
    GpsManager gpsManager;
    Calibration calibration;
    GpsSimulator gpsSimulator;
    SerialCli serialCli(gpsManager, calibration, gpsSimulator);

    AppController app(board, displayUi, inputManager, gpsManager, calibration, gpsSimulator, serialCli);
    app.begin();

    // 0. Arming Rover Mode without GPS location fix must be rejected!
    UserEvent evTapMyGrid;
    evTapMyGrid.type = UserEventType::TouchLocalGrid;
    app.handleUserEvent(evTapMyGrid, 500);
    ASSERT_TRUE(!app.isRoverMode());
    ASSERT_TRUE(app.getRoverLogManager().isEmpty());

    // Set starting position at Mount Diablo (CM87wa) and set remote target grid
    GeoPoint startPos(37.421998, -122.084000, true);
    app.setCurrentPosition(startPos);
    app.setTargetGrid("CM97BE");
    ASSERT_STR_EQ(app.getMyGrid(), "CM87WK91");
    ASSERT_STR_EQ(app.getTargetGrid(), "CM97BE");
    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Bearing);
    ASSERT_TRUE(!app.isRoverMode());
    ASSERT_TRUE(app.getRoverLogManager().isEmpty());

    // 1. One-Tap Origin Capture: Tap Row 1 (My Grid)
    app.handleUserEvent(evTapMyGrid, 1000);

    // Rover Mode engaged
    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Rover);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_TRUE(!app.getGridEditor().isEditing()); // Bezel locked out

    // Origin grid is captured into target grid slot (disarmed)
    ASSERT_STR_EQ(app.getTargetGrid(), "CM87WK");

    // History stack has Stop #1 (ORIGIN, 0.0 km)
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 1);
    const RoverStop *stop1 = app.getRoverLogManager().getStop(0);
    ASSERT_TRUE(stop1 != nullptr);
    ASSERT_NEAR(stop1->displacement_km, 0.0f, 1e-4);
    ASSERT_TRUE(!stop1->qualified_16km);
    ASSERT_NEAR(app.getOriginPoint().latitude, startPos.latitude, 1e-6);
    ASSERT_NEAR(app.getOriginPoint().longitude, startPos.longitude, 1e-6);
    ASSERT_STR_EQ(app.getOriginGrid(), "CM87WK91");

    // Knob press while driving in Rover Mode: locked out to enforce 2-step natural exit
    UserEvent evKnobPress;
    evKnobPress.type = UserEventType::EncoderButtonPressed;
    app.handleUserEvent(evKnobPress, 1050);
    ASSERT_TRUE(!app.getGridEditor().isEditing());
    ASSERT_TRUE(app.isRoverMode());

    // Road vibration bezel bumps while driving: no character rolling and does NOT exit Rover Mode
    UserEvent evBump;
    evBump.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evBump, 1100);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_STR_EQ(app.getTargetGrid(), "CM87WK");

    // 2. Continuous Displacement: Drive 10.0 km (State A: InTransitUnder16k)
    GeoPoint pos10km = GpsSimulator::calculateRoverDisplacement(startPos, 10.0, 45.0);
    app.setCurrentPosition(pos10km);
    app.update(2000);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_NEAR(app.getDistance() / 1000.0, 10.0, 0.1);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::InTransitUnder16k);

    // 3. Drive to 24.2 km into new grid (State B: QualifiedNewGrid)
    GeoPoint pos24km = GpsSimulator::calculateRoverDisplacement(startPos, 24.2, 45.0);
    app.setCurrentPosition(pos24km);
    app.update(3000);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_NEAR(app.getDistance() / 1000.0, 24.2, 0.1);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    // 4. Dynamic Backtrack: Switchback turn drops displacement to 15.2 km (< 16 km)
    // Assert instantaneous drop back to State A (no hysteresis latching)
    GeoPoint pos15km = GpsSimulator::calculateRoverDisplacement(startPos, 15.2, 45.0);
    app.setCurrentPosition(pos15km);
    app.update(4000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::InTransitUnder16k);

    // 4B. Accidental MyGrid Tap in Rover Mode: Block restart and trigger 3-second warning
    app.handleUserEvent(evTapMyGrid, 4500);
    ASSERT_TRUE(app.isRoverMode()); // Still in Rover Mode
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 1); // Origin stop not duplicated
    ASSERT_NEAR(app.getOriginPoint().latitude, startPos.latitude, 1e-6); // Origin NOT overwritten
    ASSERT_NEAR(app.getOriginPoint().longitude, startPos.longitude, 1e-6);
    ASSERT_TRUE(app.isRoverWarningActive(4500)); // Warning is active

    // 1.5 seconds later (timestamp 6000): GPS updates continue, warning remains active
    GeoPoint pos15_5km = GpsSimulator::calculateRoverDisplacement(startPos, 15.5, 45.0);
    app.setCurrentPosition(pos15_5km);
    app.update(6000);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_TRUE(app.isRoverWarningActive(6000));

    // 3.0 seconds after tap (timestamp 7500): Warning automatically expires without user input
    app.update(7500);
    ASSERT_TRUE(!app.isRoverWarningActive(7500));
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_NEAR(app.getDistance() / 1000.0, 15.5, 0.1);

    // Resume forward driving to 25.0 km -> immediately recovers to State B
    GeoPoint pos25km = GpsSimulator::calculateRoverDisplacement(startPos, 25.0, 45.0);
    app.setCurrentPosition(pos25km);
    app.update(8500);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    // 5. Two-Step Natural Target Exit Protection
    // Step 1: Accidental touch on Target Grid character (Row 2)
    UserEvent evTouchChar;
    evTouchChar.type = UserEventType::TouchGridCharacter;
    evTouchChar.gridIndex = 2;
    app.handleUserEvent(evTouchChar, 9000);

    // Character selected and editing armed, BUT Rover Mode is NOT exited!
    ASSERT_TRUE(app.getGridEditor().isEditing());
    ASSERT_TRUE(app.getGridEditor().getCursorColumn() == 2);
    ASSERT_TRUE(app.isRoverMode()); // Still in Rover Mode!

    // Inactivity timeout (5 seconds = EDIT_TIMEOUT_MS): driver didn't turn knob
    app.update(14050); // 9000 + 5050 ms
    ASSERT_TRUE(!app.getGridEditor().isEditing()); // Cursor hidden, edit auto-locked
    ASSERT_TRUE(app.isRoverMode()); // Still in Rover Mode!

    // Step 1 again: Driver intentionally touches character to tune distant station
    app.handleUserEvent(evTouchChar, 15000);
    ASSERT_TRUE(app.getGridEditor().isEditing());
    ASSERT_TRUE(app.isRoverMode());

    // Step 2: Driver turns knob to roll character
    UserEvent evRollKnob;
    evRollKnob.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evRollKnob, 15200);

    // Commit Transition:
    // Rover Mode cleanly exited, stop committed to history, now in stationary Bearing Mode
    ASSERT_TRUE(!app.isRoverMode());
    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Bearing);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 2);

    const RoverStop *stop2 = app.getRoverLogManager().getStop(1);
    ASSERT_TRUE(stop2 != nullptr);
    ASSERT_NEAR(stop2->displacement_km, 25.0f, 0.2f);
    ASSERT_TRUE(stop2->qualified_16km);

    PASS();
}

void test_rover_history_undo_modal_and_autorearm() {
    TEST("Rover History & Undo modal, Discard Last Stop, and auto-rearm");

    RoverLogManager::resetHostStorage();

    Board board;
    DisplayUi displayUi;
    InputManager inputManager;
    GpsManager gpsManager;
    Calibration calibration;
    GpsSimulator gpsSimulator;
    SerialCli serialCli(gpsManager, calibration, gpsSimulator);

    AppController app(board, displayUi, inputManager, gpsManager, calibration, gpsSimulator, serialCli);
    app.begin();

    // Start at Site 1 (CM87wa)
    GeoPoint site1(37.421998, -122.084000, true);
    app.setCurrentPosition(site1);

    // Arm Rover Mode (creates Stop 1: CM87wa, 0.0 km)
    UserEvent evTapMyGrid;
    evTapMyGrid.type = UserEventType::TouchLocalGrid;
    app.handleUserEvent(evTapMyGrid, 1000);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 1);

    // Drive 24.0 km to Site 2 (CM87wb)
    GeoPoint site2 = GpsSimulator::calculateRoverDisplacement(site1, 24.0, 45.0);
    app.setCurrentPosition(site2);
    app.update(2000);

    // Natural exit to stationary mode by editing target (creates Stop 2: CM87wb, 24.0 km)
    UserEvent evTouchChar;
    evTouchChar.type = UserEventType::TouchGridCharacter;
    evTouchChar.gridIndex = 2;
    app.handleUserEvent(evTouchChar, 2500);

    UserEvent evRoll;
    evRoll.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evRoll, 2600);

    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Bearing);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 2);

    // Operator discovers Site 2 has severe RF noise / locked gate -> unviable!
    // Open Rover History Modal via 1.5s long-press on Row 1
    UserEvent evLongPress;
    evLongPress.type = UserEventType::LongPressLocalGridTriggered;
    app.handleUserEvent(evLongPress, 3000);
    ASSERT_TRUE(app.getUiMode() == UiMode::RoverHistory);

    // Bezel depress while Rover History modal is open: must be ignored
    UserEvent evBezel;
    evBezel.type = UserEventType::EncoderButtonPressed;
    app.handleUserEvent(evBezel, 3050);
    ASSERT_TRUE(!app.getGridEditor().isEditing());

    // Bezel rotation while Rover History modal is open: must be ignored and not roll target grid
    UserEvent evTurn;
    evTurn.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evTurn, 3060);
    ASSERT_TRUE(!app.getGridEditor().isEditing());

    // Test Cancel button: modal closes, no change to history or mode
    UserEvent evCancel;
    evCancel.type = UserEventType::CancelModalTriggered;
    app.handleUserEvent(evCancel, 3100);
    app.update(3300); // 200 ms later, modal button release clears
    ASSERT_TRUE(app.getUiMode() == UiMode::Normal);
    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Bearing);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 2);

    // Reopen modal and trigger DISCARD LAST STOP
    app.handleUserEvent(evLongPress, 4000);
    ASSERT_TRUE(app.getUiMode() == UiMode::RoverHistory);

    UserEvent evDiscard;
    evDiscard.type = UserEventType::DiscardLastStopTriggered;
    app.handleUserEvent(evDiscard, 4100);

    // Discard executed:
    // 1. Topmost stop (Site 2) popped from stack -> now 1 stop (Site 1)
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 1);
    ASSERT_TRUE(*app.getRoverLogManager().getLastStop() == *app.getRoverLogManager().getStop(0));

    // 2. Active origin restored to Site 1
    ASSERT_NEAR(app.getOriginPoint().latitude, site1.latitude, 1e-6);
    ASSERT_NEAR(app.getOriginPoint().longitude, site1.longitude, 1e-6);

    // 3. Auto-closes dialog and AUTO-RE-ARMS Rover Mode relative to Site 1!
    ASSERT_TRUE(app.getUiMode() == UiMode::Normal);
    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Rover);
    ASSERT_TRUE(!app.getGridEditor().isEditing());

    // Live displacement is immediately 24.0 km relative to Site 1!
    ASSERT_NEAR(app.getDistance() / 1000.0, 24.0, 0.2);

    // 4. GPS Loss in foliage / mountain canyon (State D: GpsLostStale)
    // In Rover Mode, advance time by 3.5 seconds without GPS update
    app.update(7700); // 4100 + 3600 ms (> 3000 ms)
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::GpsLostStale);
    ASSERT_NEAR(app.getLastValidDisplacementKm(), 24.0, 0.2); // Holds last known distance!

    // GPS Signal Recovered: new location received -> immediately returns to State B
    GeoPoint site3 = GpsSimulator::calculateRoverDisplacement(site1, 28.0, 45.0);
    app.setCurrentPosition(site3);
    app.update(8000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);
    ASSERT_NEAR(app.getDistance() / 1000.0, 28.0, 0.2);

    // 5. Verify discard when GPS fix is stale (>3000 ms) or absent
    // Natural exit to stationary mode at Site 3
    app.handleUserEvent(evTouchChar, 8500);
    app.handleUserEvent(evRoll, 8600);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 2);

    // Open history modal at 12000 ms (last GPS position was at 8000 ms, 4.0s ago -> stale)
    app.handleUserEvent(evLongPress, 12000);
    ASSERT_TRUE(app.getUiMode() == UiMode::RoverHistory);

    // Trigger Discard at 12500 ms (stale GPS fix)
    app.handleUserEvent(evDiscard, 12500);
    ASSERT_TRUE(app.getUiMode() == UiMode::Normal);
    ASSERT_TRUE(app.getOperationalMode() == OperationalMode::Rover);
    // Because GPS is stale (> 3000 ms old), Rover Card must immediately be GpsLostStale ("NO GPS")
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::GpsLostStale);
    ASSERT_NEAR(app.getLastValidDisplacementKm(), 28.0, 0.2);
    char cardBuf[32];
    formatRoverCard(app.getCurrentRoverState(), app.getLastValidDisplacementKm(), cardBuf, sizeof(cardBuf));
    ASSERT_STR_EQ(cardBuf, "NO GPS    28.0 k");
    ASSERT_TRUE(getRoverStateTextColor(app.getCurrentRoverState()) == COLOR_ROVER_NOGPS_AMBER);
    bool isRevisit28 = app.getRoverLogManager().hasVisitedGrid(app.getMyGrid());
    ASSERT_TRUE(calculateRoverDistanceColor(app.getLastValidDisplacementKm(), isRevisit28) == COLOR_ROVER_QUALIFIED_GREEN);

    PASS();
}

void test_serial_cli_rover_commands() {
    TEST("SerialCli extended rover commands: status, history, undo, max, clear, help");

    RoverLogManager::resetHostStorage();

    GpsManager gm;
    Calibration cal;
    GpsSimulator sim;
    RoverLogManager logMgr;
    logMgr.begin();

    SerialCli cli(gm, cal, sim, &logMgr);

    static std::string cliOut;
    cliOut.clear();
    cli.setOutputHandler([](const char *str, void *ctx) {
        auto *out = static_cast<std::string*>(ctx);
        if (out) *out += str;
    }, &cliOut);

    // 1. Rover Help
    cli.executeCommand("rover help");
    ASSERT_TRUE(cliOut.find("Rover Mode Serial Commands") != std::string::npos);
    ASSERT_TRUE(cliOut.find("rover [status]") != std::string::npos);
    ASSERT_TRUE(cliOut.find("rover start") != std::string::npos);
    ASSERT_TRUE(cliOut.find("rover stop") != std::string::npos);
    ASSERT_TRUE(cliOut.find("rover history") != std::string::npos);
    ASSERT_TRUE(cliOut.find("rover undo") != std::string::npos);
    ASSERT_TRUE(cliOut.find("rover max") != std::string::npos);
    ASSERT_TRUE(cliOut.find("rover clear") != std::string::npos);
    cliOut.clear();

    // 2. Initial Rover Status (Inactive, No Origin)
    cli.executeCommand("rover status");
    ASSERT_TRUE(cliOut.find("Rover Transit Status") != std::string::npos);
    ASSERT_TRUE(cliOut.find("INACTIVE (Bearing Mode)") != std::string::npos);
    ASSERT_TRUE(cliOut.find("Not set") != std::string::npos);
    cliOut.clear();

    // 3. Rover History (Empty)
    cli.executeCommand("rover history");
    ASSERT_TRUE(cliOut.find("ROVER LOCATION HISTORY (0 stops / max 100)") != std::string::npos);
    ASSERT_TRUE(cliOut.find("(No stops recorded)") != std::string::npos);
    cliOut.clear();

    // 4. Standalone Rover Start & Status (with simulated location)
    gm.setLocation(GeoPoint(37.421998, -122.084000, true));
    cli.executeCommand("rover start");
    ASSERT_TRUE(cliOut.find("Rover mode started at CM87WK91") != std::string::npos);
    ASSERT_TRUE(cli.isRoverMode());
    ASSERT_TRUE(logMgr.getStopCount() == 1); // Auto-logged Stop 1 (ORIGIN)
    cliOut.clear();

    // Set displacement to 8.5 km (State A: In-Transit < 16.0 km)
    cli.setRoverDistance(8500.0);
    cli.setRoverState(RoverState::InTransitUnder16k);
    cli.setCurrentGrid("CM87WK91");
    cli.setGpsFresh(true);

    cli.executeCommand("rover status");
    ASSERT_TRUE(cliOut.find("ACTIVE (Rover Mode)") != std::string::npos);
    ASSERT_TRUE(cliOut.find("37.421998, -122.084000") != std::string::npos);
    ASSERT_TRUE(cliOut.find("8.5 km") != std::string::npos);
    ASSERT_TRUE(cliOut.find("SAME GRID") != std::string::npos);
    ASSERT_TRUE(cliOut.find("STATE A: In-Transit") != std::string::npos);
    cliOut.clear();

    // Move to 24.5 km into new grid CM87wb (State B: QUALIFIED)
    cli.setRoverDistance(24500.0);
    cli.setRoverState(RoverState::QualifiedNewGrid);
    cli.setCurrentGrid("CM87WB02");
    cli.executeCommand("rover status");
    ASSERT_TRUE(cliOut.find("24.5 km") != std::string::npos);
    ASSERT_TRUE(cliOut.find("TRAVERSED GRID") != std::string::npos);
    ASSERT_TRUE(cliOut.find("STATE B: QUALIFIED") != std::string::npos);
    cliOut.clear();

    // 5. Standalone Rover Stop -> commits stop to history
    gm.setLocation(GeoPoint(37.581204, -122.215401, true));
    cli.executeCommand("rover stop");
    ASSERT_TRUE(cliOut.find("Rover mode stopped. Position committed to history.") != std::string::npos);
    ASSERT_TRUE(!cli.isRoverMode());
    ASSERT_TRUE(logMgr.getStopCount() == 2);
    cliOut.clear();

    // Stop again when already inactive -> warning
    cli.executeCommand("rover stop");
    ASSERT_TRUE(cliOut.find("not currently active") != std::string::npos);
    cliOut.clear();

    // 6. Rover History Formatted ASCII Table Dump
    cli.executeCommand("rover history");
    ASSERT_TRUE(cliOut.find("ROVER LOCATION HISTORY (2 stops / max 100)") != std::string::npos);
    ASSERT_TRUE(cliOut.find("# | UTC Timestamp       | Latitude   | Longitude    | Grid     | Disp (km) | Status") != std::string::npos);
    ASSERT_TRUE(cliOut.find("1 |") != std::string::npos);
    ASSERT_TRUE(cliOut.find("ORIGIN") != std::string::npos);
    ASSERT_TRUE(cliOut.find("2 |") != std::string::npos);
    ASSERT_TRUE(cliOut.find("24.5 k") != std::string::npos);
    ASSERT_TRUE(cliOut.find("QUALIFIED") != std::string::npos);
    cliOut.clear();

    // Also check alias 'rover log'
    cli.executeCommand("rover log");
    ASSERT_TRUE(cliOut.find("ROVER LOCATION HISTORY (2 stops / max 100)") != std::string::npos);
    cliOut.clear();

    // 7. Rover Undo -> pops top stop, auto-rearms
    cli.executeCommand("rover undo");
    ASSERT_TRUE(cliOut.find("Discarded last stop") != std::string::npos);
    ASSERT_TRUE(logMgr.getStopCount() == 1);
    ASSERT_TRUE(cli.isRoverMode());
    cliOut.clear();

    // Undo initial stop -> stack now empty
    cli.executeCommand("rover undo");
    ASSERT_TRUE(cliOut.find("Rover history is now empty") != std::string::npos);
    ASSERT_TRUE(logMgr.isEmpty());
    cliOut.clear();

    // Undo on empty stack -> protected
    cli.executeCommand("rover undo");
    ASSERT_TRUE(cliOut.find("No stops to undo (history is empty)") != std::string::npos);
    cliOut.clear();

    // 8. Rover Max Stops Configuration
    cli.executeCommand("rover max");
    ASSERT_TRUE(cliOut.find("Max stops capacity: 100") != std::string::npos);
    cliOut.clear();

    cli.executeCommand("rover max 50");
    ASSERT_TRUE(cliOut.find("Max stops capacity set to 50") != std::string::npos);
    ASSERT_TRUE(logMgr.getMaxStops() == 50);
    cliOut.clear();

    // Clamping to MIN_ROVER_STOPS (10)
    cli.executeCommand("rover max 5");
    ASSERT_TRUE(cliOut.find("Max stops capacity set to 10") != std::string::npos);
    ASSERT_TRUE(logMgr.getMaxStops() == 10);
    cliOut.clear();

    // Clamping to ABS_MAX_ROVER_STOPS (500)
    cli.executeCommand("rover max 999");
    ASSERT_TRUE(cliOut.find("Max stops capacity set to 500") != std::string::npos);
    ASSERT_TRUE(logMgr.getMaxStops() == 500);
    cliOut.clear();

    cli.executeCommand("rover max 100");
    cliOut.clear();

    // 9. Rover Clear with confirmation prompt and abort
    RoverStop dummy(1000, 37.0, -122.0, "CM87wa", 0.0f, false);
    logMgr.appendStop(dummy);
    ASSERT_TRUE(logMgr.getStopCount() == 1);

    // Unconfirmed 'rover clear' sets pendingClear_ and prompts warning
    cli.executeCommand("rover clear");
    ASSERT_TRUE(cliOut.find("WARNING: This will permanently erase") != std::string::npos);
    ASSERT_TRUE(logMgr.getStopCount() == 1); // Not cleared yet!
    cliOut.clear();

    // Abort with 'no'
    cli.executeCommand("no");
    ASSERT_TRUE(cliOut.find("Rover clear aborted") != std::string::npos);
    ASSERT_TRUE(logMgr.getStopCount() == 1);
    cliOut.clear();

    // Trigger clear again and confirm with 'yes'
    cli.executeCommand("rover clear");
    cliOut.clear();
    cli.executeCommand("yes");
    ASSERT_TRUE(cliOut.find("Rover location history cleared from NVS") != std::string::npos);
    ASSERT_TRUE(logMgr.isEmpty());
    cliOut.clear();

    // Verify non-sticky confirmation: typing an unrelated command aborts pending clear
    logMgr.appendStop(dummy);
    ASSERT_TRUE(logMgr.getStopCount() == 1);
    cli.executeCommand("rover clear");
    cliOut.clear();
    cli.executeCommand("rover"); // Unrelated command
    ASSERT_TRUE(cliOut.find("Rover clear aborted") != std::string::npos);
    cliOut.clear();
    // Subsequent 'yes' must NOT clear history
    cli.executeCommand("yes");
    ASSERT_TRUE(logMgr.getStopCount() == 1); // History safe!
    cliOut.clear();

    // Single-line 'rover clear confirm' and 'rover clear yes'
    cli.executeCommand("rover clear confirm");
    ASSERT_TRUE(cliOut.find("Rover location history cleared from NVS") != std::string::npos);
    ASSERT_TRUE(logMgr.isEmpty());
    cliOut.clear();

    PASS();
}

void test_rover_simulation_scenarios_e2e() {
    TEST("End-to-End Simulation: 16 km transit, backtrack, signal loss, and loop course revisit");

    RoverLogManager::resetHostStorage();

    Board board;
    DisplayUi displayUi;
    InputManager inputManager;
    GpsManager gpsManager;
    Calibration calibration;
    GpsSimulator gpsSimulator;
    SerialCli serialCli(gpsManager, calibration, gpsSimulator);

    AppController app(board, displayUi, inputManager, gpsManager, calibration, gpsSimulator, serialCli);
    app.begin();

    static std::string cliOut;
    cliOut.clear();
    serialCli.setOutputHandler([](const char *str, void *ctx) {
        auto *out = static_cast<std::string*>(ctx);
        if (out) *out += str;
    }, &cliOut);

    auto injectKsxt = [&](const GeoPoint &pt, float hdg, BearingQuality q, unsigned long nowMs) {
        char buf[256];
        GpsSimulator::generateKsxt(buf, sizeof(buf), "20261108160000.00",
                                  pt.latitude, pt.longitude, hdg, q);
        serialCli.executeCommand(buf, nowMs);
        app.update(nowMs);
    };

    // ==============================================================
    // Scenario 1: 16 km Transit Journey (State A -> State B)
    // ==============================================================
    GeoPoint originSite(37.421998, -122.084000, true); // CM87wa
    injectKsxt(originSite, 45.0f, BEARING_FIXED, 1000);

    // Emulate Row 1 tap via CLI command 'rover start'
    serialCli.executeCommand("rover start", 1000);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_TRUE(cliOut.find("Rover mode started at CM87WK91") != std::string::npos);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 1);
    ASSERT_TRUE(app.getRoverLogManager().getStop(0)->timestamp == utcCalendarToEpoch(2026, 11, 8, 16, 0, 0));
    cliOut.clear();

    // Status at origin: State A, SAME GRID, 0.0 km
    serialCli.executeCommand("rover status", 1100);
    ASSERT_TRUE(cliOut.find("ACTIVE (Rover Mode)") != std::string::npos);
    ASSERT_TRUE(cliOut.find("0.0 km") != std::string::npos);
    ASSERT_TRUE(cliOut.find("SAME GRID") != std::string::npos);
    ASSERT_TRUE(cliOut.find("STATE A: In-Transit") != std::string::npos);
    ASSERT_TRUE(cliOut.find("FIXED (High-Precision RTK)") != std::string::npos);
    cliOut.clear();

    // Drive 12.0 km (still under 16 km -> State A, Traversed Grid)
    GeoPoint pos12km = GpsSimulator::calculateRoverDisplacement(originSite, 12.0, 45.0);
    injectKsxt(pos12km, 45.0f, BEARING_FIXED, 2000);

    serialCli.executeCommand("rover status", 2000);
    ASSERT_TRUE(cliOut.find("12.0 km") != std::string::npos);
    ASSERT_TRUE(cliOut.find("STATE A: In-Transit") != std::string::npos);
    cliOut.clear();

    // Break past 16 km threshold to 24.2 km into CM87wb -> State B (QUALIFIED)
    GeoPoint site2 = GpsSimulator::calculateRoverDisplacement(originSite, 24.2, 45.0);
    injectKsxt(site2, 45.0f, BEARING_FIXED, 3000);

    serialCli.executeCommand("rover status", 3000);
    ASSERT_TRUE(cliOut.find("24.2 km") != std::string::npos);
    ASSERT_TRUE(cliOut.find("TRAVERSED GRID") != std::string::npos);
    ASSERT_TRUE(cliOut.find("STATE B: QUALIFIED") != std::string::npos);
    cliOut.clear();

    // Verify stationary vehicle fix: standing still at site2 for 5 seconds receiving identical KSXT
    // keeps GPS watchdog fresh and NEVER triggers false-positive State D (NO GPS)
    injectKsxt(site2, 45.0f, BEARING_FIXED, 4000);
    injectKsxt(site2, 45.0f, BEARING_FIXED, 5000);
    injectKsxt(site2, 45.0f, BEARING_FIXED, 6000);
    injectKsxt(site2, 45.0f, BEARING_FIXED, 7000);
    injectKsxt(site2, 45.0f, BEARING_FIXED, 8000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    // Stop transit via CLI 'rover stop' -> commits Site 2 to contest history log
    serialCli.executeCommand("rover stop", 8100);
    ASSERT_TRUE(!app.isRoverMode());
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 2);
    const RoverStop *stop2 = app.getRoverLogManager().getLastStop();
    ASSERT_TRUE(stop2 != nullptr);
    ASSERT_NEAR(stop2->displacement_km, 24.2f, 0.2f);
    ASSERT_TRUE(stop2->qualified_16km);
    cliOut.clear();

    // ==============================================================
    // Scenario 2: Mountain Canyon Backtrack Retreat (State B -> State A -> State B)
    // ==============================================================
    // Depart Site 2 towards mountain ridge
    serialCli.executeCommand("rover start", 10000);
    ASSERT_TRUE(app.isRoverMode());
    cliOut.clear();

    // Climb ridge to 16.8 km -> State B (QUALIFIED)
    GeoPoint pos16_8km = GpsSimulator::calculateRoverDisplacement(site2, 16.8, 30.0);
    injectKsxt(pos16_8km, 30.0f, BEARING_FIXED, 11000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    serialCli.executeCommand("rover status", 11000);
    ASSERT_TRUE(cliOut.find("16.8 km") != std::string::npos);
    ASSERT_TRUE(cliOut.find("STATE B: QUALIFIED") != std::string::npos);
    cliOut.clear();

    // Switchback turn down canyon: straight-line distance retreats to 15.1 km (< 16 km)
    // Assert instantaneous, hysteresis-free drop to State A
    GeoPoint pos15_1km = GpsSimulator::calculateRoverDisplacement(site2, 15.1, 30.0);
    injectKsxt(pos15_1km, 30.0f, BEARING_FIXED, 12000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::InTransitUnder16k);

    serialCli.executeCommand("rover status", 12000);
    ASSERT_TRUE(cliOut.find("15.1 km") != std::string::npos);
    ASSERT_TRUE(cliOut.find("STATE A: In-Transit") != std::string::npos);
    cliOut.clear();

    // Ridge breakout to 21.5 km -> recovers immediately to State B
    GeoPoint site3 = GpsSimulator::calculateRoverDisplacement(site2, 21.5, 30.0);
    injectKsxt(site3, 30.0f, BEARING_FIXED, 13000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    // Commit Site 3
    serialCli.executeCommand("rover stop", 13100);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 3);
    cliOut.clear();

    // ==============================================================
    // Scenario 3: GPS Signal Drop and Watchdog Recovery (State B -> State D -> State B)
    // ==============================================================
    serialCli.executeCommand("rover start", 14000);
    ASSERT_TRUE(app.isRoverMode());
    cliOut.clear();

    GeoPoint pos19km = GpsSimulator::calculateRoverDisplacement(site3, 19.0, 45.0);
    injectKsxt(pos19km, 45.0f, BEARING_FIXED, 15000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    // Signal drop in dense canopy: advance time by 3.6 seconds (> 3000 ms watchdog) without GPS update
    app.update(18600);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::GpsLostStale);
    ASSERT_TRUE(getRoverStateTextColor(app.getCurrentRoverState()) == COLOR_ROVER_NOGPS_AMBER);
    bool isRevisit19 = app.getRoverLogManager().hasVisitedGrid(app.getMyGrid());
    ASSERT_TRUE(calculateRoverDistanceColor(app.getLastValidDisplacementKm(), isRevisit19) == COLOR_ROVER_QUALIFIED_GREEN);

    serialCli.executeCommand("rover status", 18600);
    ASSERT_TRUE(cliOut.find("STATE D: NO GPS") != std::string::npos);
    ASSERT_TRUE(cliOut.find("19.0 km") != std::string::npos); // Holds last valid reading!
    ASSERT_TRUE(cliOut.find("Stale / Timeout") != std::string::npos);
    cliOut.clear();

    // GPS fix recovered -> returns to State B
    GeoPoint site4 = GpsSimulator::calculateRoverDisplacement(site3, 22.0, 45.0);
    injectKsxt(site4, 45.0f, BEARING_FIXED, 19000);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    serialCli.executeCommand("rover status", 19000);
    ASSERT_TRUE(cliOut.find("STATE B: QUALIFIED") != std::string::npos);
    ASSERT_TRUE(cliOut.find("Fresh") != std::string::npos);
    cliOut.clear();

    serialCli.executeCommand("rover stop", 19100);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 4);
    cliOut.clear();

    // ==============================================================
    // Scenario 4: Loop Course Revisit (State C Assertion)
    // ==============================================================
    // Depart Site 4 and travel back to origin grid CM87wa (already activated at Stop 1)
    serialCli.executeCommand("rover start", 20000);
    ASSERT_TRUE(app.isRoverMode());
    cliOut.clear();

    // Drive back to originSite coordinates (CM87wa) >= 16 km displacement from Site 4
    injectKsxt(originSite, 45.0f, BEARING_FIXED, 21000);
    app.update(21000);

    // Assert State C (REVISITED)
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedRevisitedGrid);

    // Signal drop during State C (REVISITED): "NO GPS" in Amber, distance retains Royal Blue
    app.update(24500); // 3500 ms (> 3000 ms watchdog) without GPS update
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::GpsLostStale);
    ASSERT_TRUE(getRoverStateTextColor(app.getCurrentRoverState()) == COLOR_ROVER_NOGPS_AMBER);
    bool isRevisitOrigin = app.getRoverLogManager().hasVisitedGrid(app.getMyGrid());
    ASSERT_TRUE(calculateRoverDistanceColor(app.getLastValidDisplacementKm(), isRevisitOrigin) == COLOR_ROVER_REVISIT_BLUE);

    // Re-acquire GPS fix: returns to State C
    injectKsxt(originSite, 45.0f, BEARING_FIXED, 24600);
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedRevisitedGrid);

    serialCli.executeCommand("rover status", 24600);
    ASSERT_TRUE(cliOut.find("STATE C: REVISITED") != std::string::npos);
    ASSERT_TRUE(cliOut.find("Previously Visited") != std::string::npos);
    cliOut.clear();

    // Commit revisited stop
    serialCli.executeCommand("rover stop", 21100);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 5);
    cliOut.clear();

    // Verify formatted ASCII contest history table has 5 stops with all details
    serialCli.executeCommand("rover history", 21200);
    ASSERT_TRUE(cliOut.find("ROVER LOCATION HISTORY (5 stops / max 100)") != std::string::npos);
    ASSERT_TRUE(cliOut.find("1 |") != std::string::npos);
    ASSERT_TRUE(cliOut.find("ORIGIN") != std::string::npos);
    ASSERT_TRUE(cliOut.find("2 |") != std::string::npos);
    ASSERT_TRUE(cliOut.find("3 |") != std::string::npos);
    ASSERT_TRUE(cliOut.find("4 |") != std::string::npos);
    ASSERT_TRUE(cliOut.find("5 |") != std::string::npos);
    cliOut.clear();

    // Verify CLI Undo pops Stop 5 and auto-rearms Rover Mode relative to Site 4
    serialCli.executeCommand("rover undo", 22000);
    ASSERT_TRUE(cliOut.find("Discarded last stop") != std::string::npos);
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 4);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_NEAR(app.getOriginPoint().latitude, site4.latitude, 1e-5);
    ASSERT_NEAR(app.getOriginPoint().longitude, site4.longitude, 1e-5);
    cliOut.clear();

    // Verify CLI Max Stops adjustment
    serialCli.executeCommand("rover max 150", 22100);
    ASSERT_TRUE(cliOut.find("Max stops capacity set to 150") != std::string::npos);
    ASSERT_TRUE(app.getRoverLogManager().getMaxStops() == 150);
    cliOut.clear();

    // Verify CLI Clear
    serialCli.executeCommand("rover clear confirm", 22200);
    ASSERT_TRUE(cliOut.find("Rover location history cleared from NVS") != std::string::npos);
    ASSERT_TRUE(app.getRoverLogManager().isEmpty());
    cliOut.clear();

    PASS();
}

void test_legacy_nmea_end_to_end() {
    TEST("Legacy NMEA ($GPHDT + $GNGLL) end-to-end compatibility and Rover Mode");

    RoverLogManager::resetHostStorage();

    Board board;
    DisplayUi displayUi;
    InputManager inputManager;
    GpsManager gpsManager;
    Calibration calibration;
    GpsSimulator gpsSimulator;
    SerialCli serialCli(gpsManager, calibration, gpsSimulator);

    AppController app(board, displayUi, inputManager, gpsManager, calibration, gpsSimulator, serialCli);
    app.begin();

    // 1. Feed pure legacy $GPHDT (5 samples to fill filter window) and $GNGLL messages
    for (int i = 0; i < 5; i++) {
        gpsManager.feedSentence("$GPHDT,225.4,T*2B", 1000 + i * 200);
    }
    gpsManager.feedSentence("$GNGLL,3725.3199,N,12205.0400,W,164500.00,A,A*50", 1800);
    app.update(1800);

    // Assert Heading parsed with FIXED quality (white, with 270 deg perpendicular antenna offset), position parsed, time synced
    ASSERT_TRUE(gpsManager.hasHeading());
    ASSERT_NEAR(gpsManager.getHeading(), std::fmod(225.4 + 270.0, 360.0), 0.1);
    ASSERT_TRUE(gpsManager.getBearingQuality() == BEARING_FIXED);

    ASSERT_TRUE(gpsManager.hasLocation());
    ASSERT_NEAR(gpsManager.getLocation().latitude, 37.421998, 1e-4);
    ASSERT_NEAR(gpsManager.getLocation().longitude, -122.084000, 1e-4);
    ASSERT_STR_EQ(app.getMyGrid(), "CM87WK91");

    ASSERT_TRUE(gpsManager.hasTime());
    ASSERT_TRUE(gpsManager.getHours() == 16);
    ASSERT_TRUE(gpsManager.getMinutes() == 45);

    // Stream health evaluation: OK_FIXED
    ASSERT_TRUE(gpsManager.evaluateStreamHealth(1800) == GPS_LINK_OK_FIXED);

    // 2. Rover Mode with pure legacy messages
    UserEvent evTapMyGrid;
    evTapMyGrid.type = UserEventType::TouchLocalGrid;
    app.handleUserEvent(evTapMyGrid, 1100);
    ASSERT_TRUE(app.isRoverMode());
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 1);
    // Verified timestamp falls back to seconds-into-the-day (16*3600 + 45*60)
    ASSERT_TRUE(app.getRoverLogManager().getStop(0)->timestamp == 16 * 3600 + 45 * 60);

    // Drive 20 km North (lat moves from 37.4220 to ~37.6020)
    // 37 deg 36.1200' N = 37.6020 N
    gpsManager.feedSentence("$GPHDT,0.0,T*1F", 2000);
    gpsManager.feedSentence("$GNGLL,3736.1200,N,12205.0400,W,170500.00,A,A*57", 2000);
    app.update(2000);

    // Rover card must qualify State B (>= 16 km, new grid)
    ASSERT_TRUE(app.getCurrentRoverState() == RoverState::QualifiedNewGrid);

    // Natural exit via target edit
    UserEvent evTouchChar;
    evTouchChar.type = UserEventType::TouchGridCharacter;
    evTouchChar.gridIndex = 2;
    app.handleUserEvent(evTouchChar, 2500);

    UserEvent evRollKnob;
    evRollKnob.type = UserEventType::RotateClockwise;
    app.handleUserEvent(evRollKnob, 2600);

    ASSERT_TRUE(!app.isRoverMode());
    ASSERT_TRUE(app.getRoverLogManager().getStopCount() == 2);
    const RoverStop *stop2 = app.getRoverLogManager().getStop(1);
    ASSERT_TRUE(stop2 != nullptr);
    ASSERT_TRUE(stop2->qualified_16km);
    ASSERT_NEAR(stop2->displacement_km, 20.0f, 0.5f);

    // 3. Lost signal on legacy stream: empty $GPHDT triggers BEARING_LOST
    gpsManager.feedSentence("$GPHDT,,T*2B", 3000);
    app.update(3000);
    ASSERT_TRUE(gpsManager.getBearingQuality() == BEARING_LOST);

    PASS();
}

void test_ble_serial_manager() {
    TEST("BleSerialManager lifecycle, callbacks, and TX buffering");

    BleSerialManager ble;
    ASSERT_TRUE(!ble.isConnected());
    ble.begin("Compass-Test");
    ASSERT_STR_EQ(ble.getDeviceName(), "Compass-Test");

    bool connState = false;
    ble.setConnectionCallback([](bool connected, void *ctx) {
        *static_cast<bool*>(ctx) = connected;
    }, &connState);

    ble.setConnected(true);
    ASSERT_TRUE(ble.isConnected());
    ASSERT_TRUE(connState == true);

    ble.print("Hello ");
    ble.println("BLE");
    ASSERT_STR_EQ(ble.getTxBuffer().c_str(), "Hello BLE\r\n");

    ble.clearTxBuffer();
    ble.printFormat("Value: %d", 42);
    ASSERT_STR_EQ(ble.getTxBuffer().c_str(), "Value: 42");

    std::string receivedData;
    ble.setDataReceivedCallback([](const char *data, size_t len, void *ctx) {
        static_cast<std::string*>(ctx)->append(data, len);
    }, &receivedData);

    ble.simulateRx("status\n", 7);
    ASSERT_STR_EQ(receivedData.c_str(), "status\n");

    ble.setConnected(false);
    ASSERT_TRUE(!ble.isConnected());
    ASSERT_TRUE(connState == false);

    // Verify disconnect() helper
    ble.setConnected(true);
    ASSERT_TRUE(ble.isConnected());
    ble.disconnect();
    ASSERT_TRUE(!ble.isConnected());

    PASS();
}

void test_serial_cli_dual_transport() {
    TEST("SerialCli dual-transport isolation, BLE command routing, and broadcast");

    GpsManager gps;
    Calibration cal;
    GpsSimulator sim;
    RoverLogManager mgr;
    BleSerialManager ble;
    SerialCli cli(gps, cal, sim, &mgr, &ble);

    cli.begin();
    ble.setConnected(true);

    // 1. Verify BLE execution of 'status'
    ble.clearTxBuffer();
    ble.simulateRx("status\r\n", 8);
    ASSERT_TRUE(cli.getActiveSource() == CLI_SOURCE_BLE);
    ASSERT_TRUE(ble.getTxBuffer().find("GNSS Receiver Stream Diagnosis") != std::string::npos);

    // 2. Verify independent line buffering: partial keystrokes on BLE and USB do not collide
    ble.clearTxBuffer();
    cli.handleBleInput("ro", 2); // Partial command on BLE
    // Meanwhile, execute a command from USB directly
    std::string usbOutput;
    cli.setOutputHandler([](const char *s, void *ctx) {
        static_cast<std::string*>(ctx)->append(s);
    }, &usbOutput);
    cli.executeCommand("cal", 0, CLI_SOURCE_USB);
    ASSERT_TRUE(cli.getActiveSource() == CLI_SOURCE_USB);
    ASSERT_TRUE(usbOutput.find("Antenna Calibration Status") != std::string::npos);

    // Now complete the BLE command "ver status\r\n"
    cli.setOutputHandler(nullptr); // restore default output routing
    cli.handleBleInput("ver status\r\n", 12);
    ASSERT_TRUE(cli.getActiveSource() == CLI_SOURCE_BLE);
    ASSERT_TRUE(ble.getTxBuffer().find("Rover Transit Status") != std::string::npos);

    // 3. Verify broadcast reaches both USB and BLE
    ble.clearTxBuffer();
    std::string broadcastCapture;
    cli.setOutputHandler([](const char *s, void *ctx) {
        static_cast<std::string*>(ctx)->append(s);
    }, &broadcastCapture);
    cli.broadcastln("SYSTEM BOOT COMPLETED");
    ASSERT_TRUE(broadcastCapture.find("SYSTEM BOOT COMPLETED\r\n") != std::string::npos);
    ASSERT_TRUE(ble.getTxBuffer().find("SYSTEM BOOT COMPLETED\r\n") != std::string::npos);

    // 4. Verify 'disconnect' command terminates BLE connection
    cli.setOutputHandler(nullptr);
    ble.clearTxBuffer();
    cli.executeCommand("disconnect", 0, CLI_SOURCE_BLE);
    ASSERT_TRUE(!ble.isConnected());
    ASSERT_TRUE(ble.getTxBuffer().find("Disconnecting BLE connection...") != std::string::npos);

    PASS();
}

void test_app_controller_ble_integration() {
    TEST("AppController BLE lifecycle and UI connection state");

    Board board;
    DisplayUi display;
    InputManager input;
    GpsManager gps;
    Calibration cal;
    GpsSimulator sim;
    BleSerialManager ble;
    SerialCli cli(gps, cal, sim, nullptr, &ble);
    AppController app(board, display, input, gps, cal, sim, cli, nullptr, &ble);

    app.begin();
    ASSERT_TRUE(app.getBleManager() == &ble);

    // Verify initial disconnected state
    ASSERT_TRUE(!ble.isConnected());

    // Connect BLE
    ble.setConnected(true);
    ASSERT_TRUE(ble.isConnected());

    // Disconnect BLE
    ble.setConnected(false);
    ASSERT_TRUE(!ble.isConnected());

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
    test_gps_tx_discovery();
    test_gps_config_pipeline();
    test_serial_cli();
    test_input_manager_hit_testing();
    test_input_manager_gestures();
    test_ui_formatters_and_modal_geometry();
    test_self_test_failure_formatting();
    test_grid_editor();
    test_app_controller();
    test_self_test_assertions();
    test_rover_stop_model();
    test_rover_log_manager_stack_ops();
    test_rover_log_manager_duplicate_grid_search();
    test_rover_log_manager_capacity_and_nvs();
    test_rover_card_evaluator();
    test_rover_card_formatting_and_colors();
    test_rover_history_modal_formatters();
    test_rover_input_gestures_and_hit_testing();
    test_rover_app_controller_transitions_and_natural_exit();
    test_rover_history_undo_modal_and_autorearm();
    test_serial_cli_rover_commands();
    test_rover_simulation_scenarios_e2e();
    test_legacy_nmea_end_to_end();
    test_ble_serial_manager();
    test_serial_cli_dual_transport();
    test_app_controller_ble_integration();

    std::cout << "========================================" << std::endl;
    std::cout << "  All " << tests_passed << " / " << tests_run << " tests PASSED successfully!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
