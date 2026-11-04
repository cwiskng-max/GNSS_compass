# Maidenhead Geodetic Calculations & ARRL Contest Rules — Design Specification

* **Feature:** Maidenhead Grid Geodetics, Great-Circle Azimuths, ARRL Contest Distance Rules, & Host-Based Testing
* **Status:** Implemented (v0.2-dev)
* **Target Hardware:** Architecture-agnostic C++ (ESP32-S3 runtime + Linux / macOS Host Unit Testing)

---

## 1. Problem Statement & Motivation

Amateur radio operators in VHF, UHF, and microwave bands exchange 6-character **Maidenhead Grid Locators** (e.g. `CM87wa`) instead of raw geographic coordinates. To aim microwave dishes with $1^\circ$ to $3^\circ$ beamwidths, the station compass must convert discrete grid strings into precise forward azimuths, reverse bearings, and path distances.

Furthermore, contest scoring under the **ARRL 10 GHz and Up Contest** rules dictates:
1. Distance is computed between the exact geometric centers of the exchange grids using a spherical Earth model with mean radius $R = 6371.0\text{ km}$.
2. If two stations contact each other within the exact same 6-character subsquare (e.g. both in `CM87wa`), the scored distance is legally clamped to a **1.0 km minimum credit floor** rather than $0.0\text{ km}$.

---

## 2. Maidenhead Grid Geometry & Center Coordinate Math

The Maidenhead Locator System partitions the globe into hierarchically nested rectangles:

```text
+-------------------+-------------------+-------------------+
| Field (Chars 1-2) | Square (Chars 3-4)| Subsquare (Chars 5-6)
| A - R (18 x 18)   | 0 - 9 (10 x 10)   | A - X (24 x 24)   |
| 20° Lon x 10° Lat | 2° Lon x 1° Lat   | 5.0' Lon x 2.5' Lat
+-------------------+-------------------+-------------------+
```

### Subsquare Center Point Calculation
To eliminate directional bias when pointing antennas toward another station, `calcLatLon` calculates the coordinates of the **exact geometric center** of the $5.0' \times 2.5'$ subsquare:

$$\text{Lon}_{\text{center}} = -180^\circ + (c_0 - \text{'A'}) \cdot 20^\circ + (c_2 - \text{'0'}) \cdot 2^\circ + (c_4 - \text{'A'} + 0.5) \cdot \frac{5.0^\circ}{60}$$

$$\text{Lat}_{\text{center}} = -90^\circ + (c_1 - \text{'A'}) \cdot 10^\circ + (c_3 - \text{'0'}) \cdot 1^\circ + (c_5 - \text{'A'} + 0.5) \cdot \frac{2.5^\circ}{60}$$

```cpp
void calcLatLon(const char *grid, double *lat, double *lon) {
  char f1 = toupper(grid[0]) - 'A';
  char f2 = toupper(grid[1]) - 'A';
  char s1 = grid[2] - '0';
  char s2 = grid[3] - '0';
  char ss1 = toupper(grid[4]) - 'A';
  char ss2 = toupper(grid[5]) - 'A';

  *lon = -180.0 + (f1 * 20.0) + (s1 * 2.0) + (ss1 * (5.0 / 60.0)) + (2.5 / 60.0);
  *lat = -90.0  + (f2 * 10.0) + (s2 * 1.0) + (ss2 * (2.5 / 60.0)) + (1.25 / 60.0);
}
```

* **Origin Verification (`JJ00AA`):** Center coordinates evaluate to $+0.041667^\circ\text{ Lon}$, $+0.020833^\circ\text{ Lat}$ (just North-East of the Equator/Prime Meridian intersection).

---

## 3. Great-Circle Geodetics & Bearing Math

Using double-precision floating point, bearings and path distances are derived via spherical trigonometry.

### Forward Azimuth (Antenna Aiming Bearing)
$$\theta = \operatorname{atan2}\Big(\sin(\Delta\lambda)\cos(\phi_2), \; \cos(\phi_1)\sin(\phi_2) - \sin(\phi_1)\cos(\phi_2)\cos(\Delta\lambda)\Big)$$

$$\text{Bearing}_{\text{tgt}} = (\text{toDeg}(\theta) + 360.0) \pmod{360.0}$$

### Reverse Bearing
$$\theta_{\text{rev}} = \operatorname{atan2}\Big(-\sin(\Delta\lambda)\cos(\phi_1), \; \cos(\phi_2)\sin(\phi_1) - \sin(\phi_2)\cos(\phi_1)\cos(\Delta\lambda)\Big)$$

### ARRL Contest Distance with 1.0 km Minimum Clamp
For points on a sphere of radius $R = 6371.0\text{ km}$:

```cpp
double calculateARRLDistance(const char *grid1, const char *grid2) {
  // ARRL 10 GHz Contest Rule: Same subsquare evaluates to exactly 1.0 km
  if (strncasecmp(grid1, grid2, 6) == 0) {
    return 1.0;
  }

  double lat1, lon1, lat2, lon2;
  calcLatLon(grid1, &lat1, &lon1);
  calcLatLon(grid2, &lat2, &lon2);

  double phi1 = toRad(lat1);
  double phi2 = toRad(lat2);
  double delta_lambda = toRad(lon2 - lon1);

  // Spherical Law of Cosines
  double cos_d = sin(phi1) * sin(phi2) + cos(phi1) * cos(phi2) * cos(delta_lambda);
  if (cos_d > 1.0) cos_d = 1.0;
  if (cos_d < -1.0) cos_d = -1.0;

  double dist = 6371.0 * acos(cos_d);
  return (dist < 1.0) ? 1.0 : dist;
}
```

---

## 4. Host-Based C++ Unit Testing Architecture

To guarantee mathematical accuracy and prevent regressions across firmware builds, the calculation routines are extracted into a hardware-free header and compiled natively on Linux/macOS hosts.

### Architecture

```text
tests/
├── extracted_math.h        # Pure C++ mathematical routines without Arduino.h
├── test_calculations.cpp   # Comprehensive assertion suite
└── run_tests.sh            # Host runner executing in CI (gcc / clang)
```

### Coverage Scope
The native test suite (`tests/test_calculations.cpp`) verifies:
1. **Trigonometric Helpers:** Correct radian/degree conversion symmetry.
2. **Grid Edge Cases:** North/South poles ($\pm 90^\circ$), antimeridian ($\pm 180^\circ$), origin grid `JJ00AA`.
3. **Contest Rules:** Exact `1.0 km` return on identical grids (`CM87wa` vs `CM87wa`); adjacent subsquare scale.
4. **Bearing Symmetry:** Distance symmetry $d(A, B) = d(B, A)$ and known microwave beacon paths.
5. **Phase-Wrap Vector Smoothing:** Averaging $359.0^\circ$ and $1.0^\circ$ yields $0.0^\circ$ (not $180.0^\circ$).
6. **Watchdog & Diagnostics:** Link verdict transitions and staleness timers.

