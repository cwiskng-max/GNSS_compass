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

    std::cout << "========================================" << std::endl;
    std::cout << "  All " << tests_passed << " / " << tests_run << " tests PASSED successfully!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
