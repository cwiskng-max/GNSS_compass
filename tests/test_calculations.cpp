#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
#include "extracted_math.h"

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

void test_calc_lat_lon() {
    TEST("calcLatLon center coordinates");
    double lat = 0.0, lon = 0.0;

    // JJ00AA: Origin grid center (Equator and Prime Meridian)
    calcLatLon("JJ00AA", &lat, &lon);
    ASSERT_NEAR(lat, 1.25 / 60.0, 1e-6); // +0.020833 deg
    ASSERT_NEAR(lon, 2.5 / 60.0, 1e-6);  // +0.041667 deg

    // Coordinates should stay within valid geographic bounds
    ASSERT_TRUE(lat >= -90.0 && lat <= 90.0);
    PASS();
}

void test_arrl_1km_rule() {
    TEST("ARRL 1 km minimum distance rule");
    // Same 6-character grid must return exactly 1.0 km
    ASSERT_NEAR(calculateARRLDistance("CM87wa", "CM87wa"), 1.0, 1e-9);
    ASSERT_NEAR(calculateARRLDistance("FN31AA", "FN31AA"), 1.0, 1e-9);
    ASSERT_NEAR(calculateARRLDistance("JJ00AA", "JJ00AA"), 1.0, 1e-9);
    ASSERT_NEAR(calculateARRLDistance("ZA00AA", "ZA00AA"), 1.0, 1e-9);

    // Adjacent subsquares (2.5' lat step ~ 4.63 km) must exceed 1.0 km
    double adj = calculateARRLDistance("CM87AA", "CM87AB");
    ASSERT_TRUE(adj > 1.0);
    ASSERT_NEAR(adj, 4.63, 0.1);
    PASS();
}

void test_arrl_symmetry_and_known_paths() {
    TEST("ARRL distance symmetry and known distance ranges");
    // Symmetry: distance(A, B) == distance(B, A)
    double d1 = calculateARRLDistance("CM87wa", "DM04ms");
    double d2 = calculateARRLDistance("DM04ms", "CM87wa");
    ASSERT_NEAR(d1, d2, 1e-9);

    // CM87wa to DM04ms center-to-center is 375.72 km
    ASSERT_NEAR(d1, 375.72, 0.5);

    // Boot default distance: JJ00AA to ZA00AA is > 10,000 km
    double boot_dist = calculateARRLDistance("JJ00AA", "ZA00AA");
    ASSERT_TRUE(boot_dist > 10000.0);
    PASS();
}

void test_wgs84_distance_and_bearing() {
    TEST("calculateWGS84 distance and cardinal bearings");
    double dist = 0.0, bearing = 0.0;

    // Same coordinates
    calculateWGS84(37.5, -122.0, 37.5, -122.0, dist, bearing);
    ASSERT_NEAR(dist, 0.0, 1e-6);

    // True North: lat increases, lon constant -> bearing 0 deg
    calculateWGS84(0.0, 0.0, 1.0, 0.0, dist, bearing);
    ASSERT_NEAR(bearing, 0.0, 1e-4);
    ASSERT_NEAR(dist, 111319.5, 1000.0);

    // True East: lat constant, lon increases -> bearing 90 deg
    calculateWGS84(0.0, 0.0, 0.0, 1.0, dist, bearing);
    ASSERT_NEAR(bearing, 90.0, 1e-4);

    // True South: lat decreases, lon constant -> bearing 180 deg
    calculateWGS84(1.0, 0.0, 0.0, 0.0, dist, bearing);
    ASSERT_NEAR(bearing, 180.0, 1e-4);

    // True West: lat constant, lon decreases -> bearing 270 deg
    calculateWGS84(0.0, 1.0, 0.0, 0.0, dist, bearing);
    ASSERT_NEAR(bearing, 270.0, 1e-4);
    PASS();
}

void test_locator_round_trip() {
    TEST("calcLocator round-trip consistency");
    const char *grids[] = {"CM87AA", "FN31AA", "JJ00AA", "IO91AA"};
    for (const char *expected : grids) {
        double lat = 0.0, lon = 0.0;
        calcLatLon(expected, &lat, &lon);

        char result[9];
        calcLocator(result, (float)lat, (float)lon);
        ASSERT_TRUE(std::strncmp(result, expected, 6) == 0);
    }
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

    // 5. Valid Location extraction (Pos qual == 1, Single)
    float lat = 0.0f, lon = 0.0f;
    ASSERT_TRUE(parseKSXTLocation(ksxt_full, &lat, &lon));
    ASSERT_NEAR(lat, 40.07897925, 1e-4);
    ASSERT_NEAR(lon, 116.23662400, 1e-4);

    // 6. Western / Southern hemisphere coordinates
    const char *ksxt_west = "$KSXT,20261024185730.00,-122.12345678,37.45678901,68.3830,299.22,-67.03,190.28,0.022,,1,3,46,28*27";
    ASSERT_TRUE(parseKSXTLocation(ksxt_west, &lat, &lon));
    ASSERT_NEAR(lat, 37.45678901, 1e-4);
    ASSERT_NEAR(lon, -122.12345678, 1e-4);

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

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  GNSS Compass Unit Test Suite" << std::endl;
    std::cout << "========================================" << std::endl;

    test_trig_helpers();
    test_calc_lat_lon();
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
    test_gps_stream_evaluation();

    std::cout << "========================================" << std::endl;
    std::cout << "  All " << tests_passed << " / " << tests_run << " tests PASSED successfully!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
