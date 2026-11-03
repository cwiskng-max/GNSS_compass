#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <PCF8574.h>
#include <Adafruit_CST8XX.h>
#include <math.h>
#include <ctype.h>

// --- Bearing Quality & Link Diagnostic Enums ---
enum BearingQuality {
  BEARING_NONE = 0,    // Fresh start, never acquired -> WHITE (---.-)
  BEARING_FIXED = 1,   // High precision (RTK Fixed) -> WHITE
  BEARING_FLOAT = 2,   // Reduced quality (RTK Float) -> YELLOW
  BEARING_LOST = 3     // Signal lost / stale timeout -> RED
};

enum GpsLinkVerdict {
  GPS_LINK_DISCONNECTED = 0,   // No bytes received (>3s) or total bytes == 0
  GPS_LINK_NOISY_CORRUPT = 1,  // Raw bytes received, but no valid NMEA/KSXT sentences
  GPS_LINK_ACQUIRING = 2,      // Valid sentences streaming, waiting for satellites
  GPS_LINK_POSITION_ONLY = 3,  // Position valid, waiting for dual-antenna heading
  GPS_LINK_HEADING_FLOAT = 4,  // Dual-antenna heading active in Float mode (Yellow)
  GPS_LINK_OK_FIXED = 5        // Dual-antenna heading active in Fixed mode (White)
};
// --- End Bearing Quality & Link Diagnostic Enums ---

// I2C to PCF8574
#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39
#define BKL_PIN 6

#define I2C_TOUCH_ADDR 0x15
#define INCA(x) (((x+1)>'Z')?'A':(x+1))
#define DECA(x) (((x-1)<'A')?'Z':(x-1))
#define INCN(x) (((x+1)>'9')?'0':(x+1))
#define DECN(x) (((x-1)<'0')?'9':(x-1))
#define INCG(x) (x=(x>='0' && x<='9')?INCN(x):INCA(x))
#define DECG(x) (x=(x>='0' && x<='9')?DECN(x):DECA(x))

// --- Display Layout (480 x 480) ---
#define CHAR_W(sz)           (6 * (sz))
#define CHAR_H(sz)           (8 * (sz))

// Row 1: Local Maidenhead Grid
#define LOC_SZ               4
#define LOC_X                144
#define LOC_Y                35

// Row 2: Target Grid
#define REMSIZE              8
#define REMSTARTX            80
#define REMSTARTY            95
#define REMSTOPX             (6 * CHAR_W(REMSIZE) + REMSTARTX)
#define REMSTOPY             (2 * CHAR_H(REMSIZE) + REMSTARTY)

// Target Grid Touch Area (covers character row, caret row, and generous padding)
#define TOUCH_X_MIN          50
#define TOUCH_X_MAX          430
#define TOUCH_Y_MIN          70
#define TOUCH_Y_MAX          235

// Row 3: Antenna Heading & Target Bearing
#define BEARSZ               6
#define BEARX                30
#define BEARY                REMSTOPY
#define TGTBEARX             (BEARX + 6 * CHAR_W(BEARSZ))

// Set to 1 to show ARRL contest distance (Row 6), 0 to hide
#define ENABLE_ARRL_DIST     1

#if ENABLE_ARRL_DIST
// Row 4: UTC Time
#define TIME_SZ              6
#define TIME_X               132
#define TIME_Y               290

// Row 5: Reverse Bearing & Distance
#define REVBEAR_SZ           4
#define REVBEAR_X            90
#define REVBEAR_Y            352

// Row 6: ARRL Contest Distance
#define ARRL_SZ              3
#define ARRL_X               123
#define ARRL_Y               408
#else
// Row 4: UTC Time
#define TIME_SZ              6
#define TIME_X               132
#define TIME_Y               300

// Row 5: Reverse Bearing & Distance
#define REVBEAR_SZ           4
#define REVBEAR_X            90
#define REVBEAR_Y            372
#endif

// Row 7: Firmware Version Footer
#define FW_VERSION           "v0.3-dev"
#define VER_SZ               2
#define VER_Y                444

// Inactivity Auto-Lock Timeout (milliseconds; set to 0 to disable)
#define EDIT_TIMEOUT_MS      5000

#define aton(x) (x-'0')

 // Define the RX and TX pins for Serial 2
#define RXD2 44
#define TXD2 43

#define GPS_BAUD 115200

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

char remote[7] = "ZA00AA";
char mygrid[9] = "JJ00AA00";
bool hasLocation = false;
bool hasTime = false;
int hours = 0;
int minutes = 0;

#define ENCODER_CLK 42
#define ENCODER_DT 4

volatile int counter=0;
volatile int encState = 0;
volatile int oldState = -1;
volatile bool hasChanged = false;

PCF8574 pcf8574(0x21);

Adafruit_CST8XX tsPanel = Adafruit_CST8XX();

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  16 /* CS */, 2 /* SCK */, 1 /* SDA */,
  40 /* DE */, 7 /* VSYNC */, 15 /* HSYNC */, 41 /* PCLK */,
  46 /* R0 */, 3 /* R1 */, 8 /* R2 */, 18 /* R3 */, 17 /* R4 */,
  14 /* G0 */, 13 /* G1 */, 12 /* G2 */, 11 /* G3 */, 10 /* G4 */, 9 /* G5 */,
  5 /* B0 */, 45 /* B1 */, 48 /* B2 */, 47 /* B3 */, 21 /* B4 */
);

Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
  bus,
  GFX_NOT_DEFINED,  // RST pin (not used, we reset via PCF8574)
  0,                // rotation
  false,            // IPS
  480, 480,         // width, height
  st7701_type5_init_operations,
  sizeof(st7701_type5_init_operations),
  true,       // BGR
  10, 4, 20,  // hsync front porch, pulse width, back porch
  10, 4, 20   // vsync front porch, pulse width, back porch
);



void IRAM_ATTR encoder_irq() {
  encState = digitalRead(ENCODER_CLK);
  if (encState != oldState) {
    counter = (digitalRead(ENCODER_DT) == encState) ? +1 : -1;
    oldState = encState;
    hasChanged = true;
  }
}

void initEncoder() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoder_irq, CHANGE);
}

void parselatlon(char *buf,float *lat,float *lon, char **rptr) {

char *ptr;
int neg=0;

  ptr = &buf[0];
  if (ptr[0]=='-'){neg=1;ptr++;}
  *lat = aton(ptr[0])*10.0+aton(ptr[1]);
  *lat += strtof(&ptr[2],&ptr)/60.0;
  if (neg) {neg=0;*lat = -*lat;}

  if (ptr[3]=='-'){neg=1;ptr++;}
  *lon = aton(ptr[3])*100.0+aton(ptr[4])*10.0+aton(ptr[5]);
  *lon += strtof(&ptr[6],&ptr)/60.0;
  if (neg) {neg=0;*lon = -*lon;}

  *rptr = &ptr[3];

}

void calcLocator(char *dst, float lat, float lon) {
  int o1, o2, o3, o4;
  int a1, a2, a3, a4;
  double remainder;

  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);
  remainder = 12.0 * remainder - (double)o3;
  o4 = (int)(10.0 * remainder);

  // latitude
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);
  remainder = 24.0 * remainder - (double)a3;
  a4 = (int)(10.0 * remainder);

  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)o3 + 'A';
  dst[5] = (char)a3 + 'A';
  dst[6] = (char)o4 + '0';
  dst[7] = (char)a4 + '0';
  dst[8] = (char)0;

}

void initBacklight() {
  pinMode(BKL_PIN, OUTPUT);
  analogWrite(BKL_PIN, 250);
}

void initPins() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pcf8574.pinMode(P0, OUTPUT);        //tp RST
  pcf8574.pinMode(P2, OUTPUT);        //tp INT
  pcf8574.pinMode(P3, OUTPUT);        //lcd power
  pcf8574.pinMode(P4, OUTPUT);        //lcd reset
  pcf8574.pinMode(P5, INPUT_PULLUP);  //encoder SW

  if (!pcf8574.begin()) {
    Serial.println("Can't init pcf8574");
  }

  // LCD
  pcf8574.digitalWrite(P3, HIGH);
  delay(100);
  pcf8574.digitalWrite(P4, HIGH);
  delay(100);
  pcf8574.digitalWrite(P4, LOW);
  delay(120);
  pcf8574.digitalWrite(P4, HIGH);
  delay(120);

  // Touchpad
  pcf8574.digitalWrite(P0, HIGH);
  delay(100);
  pcf8574.digitalWrite(P0, LOW);
  delay(120);
  pcf8574.digitalWrite(P0, HIGH);
  delay(120);
  pcf8574.digitalWrite(P2, HIGH);
  delay(120);

  if (!tsPanel.begin(&Wire, I2C_TOUCH_ADDR)) {
    Serial.println("No touchscreen found");
  }
  Serial.println("reset done");
}


void initLCD() {
  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE, BLACK);

}

int remotec = 0;
bool isEditing = true;
unsigned long lastEditActivity = 0;

void printLocalGrid() {
  gfx->setTextSize(LOC_SZ);
  gfx->setCursor(LOC_X, LOC_Y);
  gfx->print(mygrid);
}

void printrem() {

  gfx->setTextSize(REMSIZE);

  for (int i = 0; i < 6; i++) {
    if (isEditing && i == remotec) {
      gfx->setTextColor(YELLOW, BLACK);
    } else {
      gfx->setTextColor(WHITE, BLACK);
    }
    gfx->setCursor(REMSTARTX + i * CHAR_W(REMSIZE), REMSTARTY);
    gfx->print(remote[i]);
  }

  gfx->setTextColor(YELLOW, BLACK);
  gfx->setCursor(REMSTARTX, REMSTARTY + CHAR_H(REMSIZE));
  gfx->print("      ");
  if (isEditing) {
    gfx->setCursor(REMSTARTX + remotec * CHAR_W(REMSIZE), REMSTARTY + CHAR_H(REMSIZE));
    gfx->print("^");
  }

  gfx->setTextColor(WHITE, BLACK);

}



// Convert degrees to radians
double toRad(double degree) {
    return degree * M_PI / 180.0;
}

// Convert radians to degrees
double toDeg(double rad) {
    return rad * 180.0 / M_PI;
}

// Convert 6-character Maidenhead grid to center coordinates
void calcLatLon(const char *grid, double *lat, double *lon) {
    *lon = (grid[0] - 'A') * 20.0 + (grid[2] - '0') * 2.0 + (grid[4] - 'A') * (5.0 / 60.0) - 180.0 + (2.5 / 60.0);
    *lat = (grid[1] - 'A') * 10.0 + (grid[3] - '0') * 1.0 + (grid[5] - 'A') * (2.5 / 60.0) - 90.0 + (1.25 / 60.0);
}

// ARRL contest distance calculation (spherical Earth R = 6371.0 km, 1 km minimum)
double calculateARRLDistance(const char *grid1, const char *grid2) {
    if (strncmp(grid1, grid2, 6) == 0) {
        return 1.0;
    }
    double lat1, lon1, lat2, lon2;
    calcLatLon(grid1, &lat1, &lon1);
    calcLatLon(grid2, &lat2, &lon2);

    double R = 6371.0; // Mean Earth radius in km (ARRL contest standard)
    double dLat = toRad(lat2 - lat1);
    double dLon = toRad(lon2 - lon1);
    double rLat1 = toRad(lat1);
    double rLat2 = toRad(lat2);

    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(rLat1) * cos(rLat2) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    if (a > 1.0) a = 1.0;
    if (a < 0.0) a = 0.0;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double d = R * c;

    return (d < 1.0) ? 1.0 : d;
}

// Calculate Distance (meters) and Bearing (degrees)
void calculateWGS84(double lat1, double lon1, double lat2, double lon2, double &distance, double &bearing) {
    double R = 6378137.0; // WGS84 semi-major axis in meters

    double dLat = toRad(lat2 - lat1);
    double dLon = toRad(lon2 - lon1);

    double rLat1 = toRad(lat1);
    double rLat2 = toRad(lat2);

    // Haversine Distance Calculation
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(rLat1) * cos(rLat2) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    distance = R * c;

    // Initial Bearing Calculation
    double y = sin(dLon) * cos(rLat2);
    double x = cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon);
    bearing = toDeg(atan2(y, x));

    // Normalize bearing to 0-360 degrees
    if (bearing < 0.0) {
        bearing += 360.0;
    }
}




// --- Bearing Quality & State Machine ---
#define HEADING_TIMEOUT_MS   1500
#define BSZ 5

const char* getNthField(const char* str, int n) {
  int count = 0;
  while (*str) {
    if (count == n) return str;
    if (*str == ',') count++;
    str++;
  }
  if (count == n) return str;
  return NULL;
}

bool parseKSXT(const char *sentence, float *heading_out, BearingQuality *quality_out) {
  const char *h_field = getNthField(sentence, 5);
  if (h_field == NULL || *h_field == ',' || *h_field == '*' || *h_field == '\0' || *h_field == '\r' || *h_field == '\n') {
    *quality_out = BEARING_LOST;
    return false;
  }

  const char *q_field = getNthField(sentence, 11);
  int qual = 0;
  if (q_field != NULL && *q_field >= '0' && *q_field <= '9') {
    qual = *q_field - '0';
  }

  if (qual == 3 || qual == 4) {
    *quality_out = BEARING_FIXED;
  } else if (qual == 2 || qual == 5) {
    *quality_out = BEARING_FLOAT;
  } else {
    // qual <= 1: 0 (invalid) or 1 (single-point / non-differential): NOT a differential bearing
    *quality_out = BEARING_LOST;
    return false;
  }

  *heading_out = strtof(h_field, NULL);
  return true;
}

bool parseKSXTTime(const char *sentence, int *hours_out, int *minutes_out) {
  const char *utc_field = getNthField(sentence, 1);
  if (utc_field == NULL || *utc_field == ',' || *utc_field == '*' || *utc_field == '\0') {
    return false;
  }

  int len = 0;
  while (isdigit((unsigned char)utc_field[len])) {
    len++;
  }

  int h = -1, m = -1;
  if (len >= 12) {
    // Format: yyyymmddhhmmss (14) or yyyymmddhhmm (12)
    h = (utc_field[8] - '0') * 10 + (utc_field[9] - '0');
    m = (utc_field[10] - '0') * 10 + (utc_field[11] - '0');
  } else if (len == 6 || len == 4) {
    // Format: hhmmss (6) or hhmm (4)
    h = (utc_field[0] - '0') * 10 + (utc_field[1] - '0');
    m = (utc_field[2] - '0') * 10 + (utc_field[3] - '0');
  } else {
    return false;
  }

  if (h >= 0 && h < 24 && m >= 0 && m < 60) {
    *hours_out = h;
    *minutes_out = m;
    return true;
  }
  return false;
}

bool parseKSXTLocation(const char *sentence, float *lat_out, float *lon_out) {
  const char *q_field = getNthField(sentence, 10);
  if (q_field == NULL || *q_field <= '0' || *q_field > '9') {
    return false;
  }

  const char *lon_field = getNthField(sentence, 2);
  const char *lat_field = getNthField(sentence, 3);
  if (lon_field == NULL || *lon_field == ',' || *lon_field == '*' ||
      lat_field == NULL || *lat_field == ',' || *lat_field == '*') {
    return false;
  }

  float lon = strtof(lon_field, NULL);
  float lat = strtof(lat_field, NULL);
  if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f || (lat == 0.0f && lon == 0.0f)) {
    return false;
  }

  *lat_out = lat;
  *lon_out = lon;
  return true;
}

bool parseHDT(const char *sentence, float *heading_out, BearingQuality *quality_out) {
  const char *h_field = getNthField(sentence, 1);
  if (h_field == NULL || *h_field == ',' || *h_field == '*' || *h_field == '\0' || *h_field == '\r' || *h_field == '\n') {
    *quality_out = BEARING_LOST;
    return false;
  }

  *heading_out = strtof(h_field, NULL);
  *quality_out = BEARING_FIXED;
  return true;
}

bool parseTHS(const char *sentence, float *heading_out, BearingQuality *quality_out) {
  const char *h_field = getNthField(sentence, 1);
  if (h_field == NULL || *h_field == ',' || *h_field == '*' || *h_field == '\0' || *h_field == '\r' || *h_field == '\n') {
    *quality_out = BEARING_LOST;
    return false;
  }

  const char *m_field = getNthField(sentence, 2);
  if (m_field == NULL || *m_field == 'V' || *m_field == 'v' || *m_field == ',' || *m_field == '*') {
    *quality_out = BEARING_LOST;
    return false;
  }

  *heading_out = strtof(h_field, NULL);
  *quality_out = BEARING_FIXED;
  return true;
}

bool parseHeadingSentence(const char *sentence, float *heading_out, BearingQuality *quality_out) {
  if (strncmp(sentence, "$KSXT", 5) == 0) {
    return parseKSXT(sentence, heading_out, quality_out);
  } else if (strncmp(sentence, "$GNTHS", 6) == 0 || strncmp(sentence, "$GPTHS", 6) == 0) {
    return parseTHS(sentence, heading_out, quality_out);
  } else if (strncmp(sentence, "$GNHDT", 6) == 0 || strncmp(sentence, "$GPHDT", 6) == 0) {
    return parseHDT(sentence, heading_out, quality_out);
  }
  return false;
}

double calcCurrentBearing(float s_avg, float c_avg) {
  double b = fmod(toDeg(atan2((double)s_avg / (double)BSZ, (double)c_avg / (double)BSZ)) + 270.0, 360.0);
  if (b < 0.0) b += 360.0;
  return b;
}

GpsLinkVerdict evaluateGpsStream(
    unsigned long now,
    unsigned long totalBytes,
    unsigned long lastByteTime,
    unsigned int gllCount,
    unsigned int ksxtCount,
    unsigned int hdtCount,
    bool hasLoc,
    bool hasHdg,
    BearingQuality quality)
{
  if (totalBytes == 0 || (now - lastByteTime > 3000)) {
    return GPS_LINK_DISCONNECTED;
  }
  if (gllCount == 0 && ksxtCount == 0 && hdtCount == 0) {
    return GPS_LINK_NOISY_CORRUPT;
  }
  if (hasHdg) {
    if (quality == BEARING_FIXED) return GPS_LINK_OK_FIXED;
    if (quality == BEARING_FLOAT) return GPS_LINK_HEADING_FLOAT;
  }
  if (hasLoc) {
    return GPS_LINK_POSITION_ONLY;
  }
  return GPS_LINK_ACQUIRING;
}
// --- End Bearing Parsing Functions ---

float sinbuff[BSZ], cosbuff[BSZ];
float sinavg = 0.0f, cosavg = 0.0f;
int sin_i = 0;

bool hasHeading = false;
BearingQuality currentBearingQuality = BEARING_NONE;
unsigned long lastHeadingUpdate = 0;
bool rovdist = false;
float mylongitude = 0.0, mylatitude = 0.0;
double remlongitude = 0.0, remlatitude = 0.0;
double distance = 0.0, bearing = 0.0, revbearing = 0.0;

// GPS Stream Diagnostic Counters
unsigned long totalGpsBytesReceived = 0;
unsigned long lastGpsByteMillis = 0;
unsigned int gllPacketCount = 0;
unsigned long lastGllMillis = 0;
unsigned int ksxtPacketCount = 0;
unsigned long lastKsxtMillis = 0;
unsigned int hdtPacketCount = 0;
unsigned long lastHdtMillis = 0;

void drawCurrentBearing() {
  uint16_t color = WHITE;
  if (currentBearingQuality == BEARING_FLOAT) {
    color = YELLOW;
  } else if (currentBearingQuality == BEARING_LOST) {
    color = RED;
  }

  gfx->setTextSize(BEARSZ);
  gfx->setCursor(BEARX, BEARY);
  gfx->setTextColor(color, BLACK);

  if (!hasHeading) {
    gfx->print("---.-");
  } else {
    char str[10];
    sprintf(str, "%5.1f", calcCurrentBearing(sinavg, cosavg));
    gfx->print(str);
  }
  gfx->setTextColor(WHITE, BLACK);
}

void updateBearings() {
  calcLatLon(remote, &remlatitude, &remlongitude);
  calculateWGS84(mylatitude, mylongitude, remlatitude, remlongitude, distance, bearing);
  calculateWGS84(remlatitude, remlongitude, mylatitude, mylongitude, distance, revbearing);

  char buf[20];
  int rdist = (int)floor(distance / 1000.0);
  sprintf(&buf[0], "%5.1f %4i k  ", revbearing, rdist);
  if ((rdist < 16) && rovdist) {
    gfx->setTextColor(RED, WHITE);
  }
  gfx->setTextSize(REVBEAR_SZ);
  gfx->setCursor(REVBEAR_X, REVBEAR_Y);
  gfx->print(buf);
  gfx->setTextColor(WHITE, BLACK);

  drawCurrentBearing();

  sprintf(&buf[0], "%5.1f", bearing);
  gfx->setTextSize(BEARSZ);
  gfx->setCursor(TGTBEARX, BEARY);
  gfx->print(buf);

#if ENABLE_ARRL_DIST
  char arrl_str[24];
  double arrl_dist = calculateARRLDistance(mygrid, remote);
  sprintf(arrl_str, "ARRL %6.1f k  ", arrl_dist);
  gfx->setTextSize(ARRL_SZ);
  gfx->setCursor(ARRL_X, ARRL_Y);
  gfx->print(arrl_str);
#endif
}

void setup() {

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.begin (115200);
  Serial.print("GNSS Compass ");
  Serial.println(FW_VERSION);
  Serial.println(F("Commands: 'status' (GPS stream diagnosis), 'test' (visual demo)"));

  for (int i = 0; i < BSZ; i++) {
    sinbuff[i] = 0.0;
    cosbuff[i] = 0.0;
  }

  initPins();
  initBacklight();
  initLCD();
  initEncoder();

  calcLocator(mygrid, mylatitude, mylongitude);
  gfx->setTextSize(LOC_SZ);
  gfx->setCursor(LOC_X, LOC_Y);
  gfx->print(mygrid);

  lastEditActivity = millis();
  printrem();
  updateBearings();

  gfx->setTextSize(TIME_SZ);
  gfx->setCursor(TIME_X, TIME_Y);
  gfx->print("--:--Z");

  // Display firmware version footer
  showFooterText(FW_VERSION, 0x7BEF);
}

// Display centered text in footer row (used for version and test step titles)
void showFooterText(const char *msg, uint16_t color) {
  gfx->fillRect(0, VER_Y, 480, CHAR_H(VER_SZ) + 4, BLACK);
  int w = strlen(msg) * CHAR_W(VER_SZ);
  gfx->setTextSize(VER_SZ);
  gfx->setTextColor(color, BLACK);
  gfx->setCursor((480 - w) / 2, VER_Y);
  gfx->print(msg);
  gfx->setTextColor(WHITE, BLACK);
}

// Display centered rounded banner card across the screen (used for test results and diagnostics)
void showStatusCard(const char *msg, uint16_t bgColor, uint16_t fgColor) {
  int w = strlen(msg) * CHAR_W(2);
  gfx->fillRoundRect(40, 215, 400, 52, 8, bgColor);
  gfx->setTextColor(fgColor, bgColor);
  gfx->setTextSize(2);
  gfx->setCursor((480 - w) / 2, 233);
  gfx->print(msg);
  gfx->setTextColor(WHITE, BLACK);
}

// Full GNSS Receiver & Data Stream diagnostic report
void printGpsStatusReport() {
  unsigned long now = millis();
  GpsLinkVerdict v = evaluateGpsStream(
    now, totalGpsBytesReceived, lastGpsByteMillis,
    gllPacketCount, ksxtPacketCount, hdtPacketCount,
    hasLocation, hasHeading, currentBearingQuality
  );

  Serial.println(F("\n========================================"));
  Serial.println(F("     GNSS Receiver Stream Diagnosis     "));
  Serial.println(F("========================================"));

  if (v == GPS_LINK_DISCONNECTED) {
    Serial.println(F("Physical Link:  DISCONNECTED / NO DATA"));
    if (totalGpsBytesReceived == 0) {
      Serial.println(F("Details:        0 bytes received on Serial2 since boot."));
    } else {
      Serial.print(F("Details:        Last byte received "));
      Serial.print((now - lastGpsByteMillis) / 1000.0, 1);
      Serial.println(F(" seconds ago."));
    }
    Serial.println(F("Verdict:        [FAIL] NO GPS DATA STREAM DETECTED"));
    Serial.println(F("Troubleshooting:"));
    Serial.println(F("  1. Verify GX12 3-pin cable is securely connected."));
    Serial.println(F("  2. Check 5V / 12V power to the UM982 receiver enclosure."));
    Serial.println(F("  3. Verify RX pin (GPIO 44) wiring to UM982 TX."));
  } else if (v == GPS_LINK_NOISY_CORRUPT) {
    Serial.println(F("Physical Link:  BYTES RECEIVED, BUT NO VALID NMEA/KSXT"));
    Serial.print(F("Total Bytes:    "));
    Serial.println(totalGpsBytesReceived);
    Serial.println(F("Verdict:        [FAIL] FRAMING / BAUD RATE MISMATCH"));
    Serial.println(F("Troubleshooting:"));
    Serial.println(F("  1. Verify receiver COM1 baud rate is set to 115200 (8N1)."));
    Serial.println(F("  2. Check for loose connection or electrical noise on signal line."));
  } else {
    Serial.println(F("Physical Link:  CONNECTED (Hardware Serial Link Active)"));
    Serial.print(F("Total Bytes:    "));
    Serial.print(totalGpsBytesReceived);
    Serial.print(F(" (last byte "));
    Serial.print(now - lastGpsByteMillis);
    Serial.println(F(" ms ago)"));

    Serial.println(F("\nMessage Streams:"));
    Serial.print(F("  - $GNGLL (Position):   "));
    Serial.print(gllPacketCount);
    Serial.print(F(" pkts (last "));
    if (gllPacketCount > 0) {
      Serial.print((now - lastGllMillis) / 1000.0, 1);
      Serial.println(F("s ago)"));
    } else {
      Serial.println(F("none)"));
    }

    Serial.print(F("  - $KSXT (Dual-Ant):    "));
    Serial.print(ksxtPacketCount);
    Serial.print(F(" pkts (last "));
    if (ksxtPacketCount > 0) {
      Serial.print(now - lastKsxtMillis);
      Serial.println(F(" ms ago)"));
    } else {
      Serial.println(F("none)"));
    }

    Serial.print(F("  - $GNHDT/$THS:         "));
    Serial.print(hdtPacketCount);
    Serial.println(F(" pkts"));

    Serial.println(F("\nFix Status:"));
    Serial.print(F("  - Time:                "));
    if (hasTime) {
      char tbuf[16];
      sprintf(tbuf, "%02d:%02d UTC", hours, minutes);
      Serial.println(tbuf);
    } else {
      Serial.println(F("SEARCHING (Syncing...)"));
    }

    Serial.print(F("  - Position:            "));
    if (hasLocation) {
      Serial.print(F("LOCKED (Grid: "));
      Serial.print(mygrid);
      Serial.println(F(")"));
    } else {
      Serial.println(F("SEARCHING (No fix yet)"));
    }

    Serial.print(F("  - Dual-Antenna Heading:"));
    if (hasHeading) {
      if (currentBearingQuality == BEARING_FIXED) {
        Serial.println(F("RTK FIXED (Pure White - High Precision)"));
      } else if (currentBearingQuality == BEARING_FLOAT) {
        Serial.println(F("RTK FLOAT (Bright Yellow - Degraded Sky View)"));
      } else {
        Serial.println(F("LOST / STALE (Bright Red - Tracking Dropped)"));
      }
    } else {
      Serial.println(F("ACQUIRING (Waiting for initial heading lock)"));
    }

    Serial.print(F("\nOverall Verdict:        "));
    if (v == GPS_LINK_OK_FIXED) {
      Serial.println(F("[PASS] FULLY OPERATIONAL (RTK Fixed)"));
    } else if (v == GPS_LINK_HEADING_FLOAT) {
      Serial.println(F("[PASS - WARN] OPERATIONAL (RTK Float - Degraded reception)"));
    } else if (v == GPS_LINK_POSITION_ONLY) {
      Serial.println(F("[WAIT] POSITION LOCKED, WAITING FOR HEADING BASELINE"));
    } else {
      Serial.println(F("[WAIT] RECEIVER TALKING, ACQUIRING SATELLITES"));
    }
  }
  Serial.println(F("========================================\n"));
}

// On-demand visual display self-test demo
void runVisualSelfTest() {
  Serial.println(F("\n=== Starting Visual Display Self-Test Demo ==="));

  // Save current live state
  char saved_remote[7];
  char saved_mygrid[9];
  memcpy(saved_remote, remote, sizeof(saved_remote));
  memcpy(saved_mygrid, mygrid, sizeof(saved_mygrid));
  float saved_mylat = mylatitude, saved_mylon = mylongitude;
  bool saved_hasHeading = hasHeading;
  bool saved_rovdist = rovdist;
  bool saved_isEditing = isEditing;
  BearingQuality saved_quality = currentBearingQuality;
  unsigned long saved_lastHeading = lastHeadingUpdate;
  isEditing = false;

  // Test 1: Same Grid (ARRL 1.0 km minimum rule)
  Serial.println(F("[Test 1/6] Same Grid (ARRL 1.0 km rule)"));
  showFooterText("TEST 1: SAME GRID", CYAN);
  strcpy(mygrid, "CM87wa00");
  strcpy(remote, "CM87wa");
  mylatitude = 38.354;
  mylongitude = -119.458;
  hasHeading = true;
  currentBearingQuality = BEARING_FIXED;
  sinavg = sin(toRad(180.0)) * BSZ;
  cosavg = cos(toRad(180.0)) * BSZ;
  rovdist = false;
  printLocalGrid();
  printrem();
  updateBearings();
  delay(3000);

  // Test 2: Rover Transit Alert (< 16 km, highlighted RED on WHITE)
  Serial.println(F("[Test 2/6] Rover Transit Alert (< 16 km, highlighted RED on WHITE)"));
  showFooterText("TEST 2: ROVER < 16K", YELLOW);
  strcpy(remote, "CM87wb");
  currentBearingQuality = BEARING_FIXED;
  rovdist = true;
  printrem();
  updateBearings();
  delay(3000);

  // Test 3: Distance Under 1,000 km (~600 km)
  Serial.println(F("[Test 3/6] Under 1,000 km formatting (~600 km)"));
  showFooterText("TEST 3: DIST < 1000K", CYAN);
  strcpy(remote, "DN13ab");
  currentBearingQuality = BEARING_FIXED;
  rovdist = false;
  printrem();
  updateBearings();
  delay(3000);

  // Test 4: Distance Over 1,000 km (~2,500 km)
  Serial.println(F("[Test 4/6] Over 1,000 km formatting (~2,500 km)"));
  showFooterText("TEST 4: DIST > 1000K", CYAN);
  strcpy(remote, "EL29ja");
  currentBearingQuality = BEARING_FIXED;
  printrem();
  updateBearings();
  delay(3000);

  // Test 5: Extreme Distance (> 10,000 km) and Recovery to 1.0 km
  Serial.println(F("[Test 5/6] Extreme distance (> 10,000 km)"));
  showFooterText("TEST 5: GHOST RECOVERY", CYAN);
  strcpy(remote, "ZA00AA");
  printrem();
  updateBearings();
  delay(2000);

  Serial.println(F("  -> Dropping back to 1.0 km (verifying no ghost characters)"));
  strcpy(remote, "CM87wa");
  printrem();
  updateBearings();
  delay(2000);

  // Test 6: Bearing Color States & Live GPS Receiver Link Diagnostic
  Serial.println(F("[Test 6/6] Bearing Color States & Live GPS Stream Diagnostic"));
  Serial.println(F("  -> Bearing Color: Pure White (RTK Fixed)"));
  showFooterText("TEST 6: BEARING WHITE", WHITE);
  currentBearingQuality = BEARING_FIXED;
  drawCurrentBearing();
  delay(1000);

  Serial.println(F("  -> Bearing Color: Bright Yellow (RTK Float / Degraded)"));
  showFooterText("TEST 6: BEARING YELLOW", YELLOW);
  currentBearingQuality = BEARING_FLOAT;
  drawCurrentBearing();
  delay(1000);

  Serial.println(F("  -> Bearing Color: Bright Red (Lost / Stale)"));
  showFooterText("TEST 6: BEARING RED", RED);
  currentBearingQuality = BEARING_LOST;
  drawCurrentBearing();
  delay(1000);

  // Restore bearing to fixed before showing live GPS stream ruling
  currentBearingQuality = BEARING_FIXED;
  drawCurrentBearing();
  showFooterText("TEST 6: GPS STREAM", CYAN);
  printGpsStatusReport();

  GpsLinkVerdict liveVerdict = evaluateGpsStream(
    millis(), totalGpsBytesReceived, lastGpsByteMillis,
    gllPacketCount, ksxtPacketCount, hdtPacketCount,
    hasLocation, hasHeading, currentBearingQuality
  );

  if (liveVerdict == GPS_LINK_OK_FIXED) {
    showStatusCard("GPS: RTK FIXED (OK)", GREEN, BLACK);
  } else if (liveVerdict == GPS_LINK_HEADING_FLOAT) {
    showStatusCard("GPS: RTK FLOAT (WARN)", YELLOW, BLACK);
  } else if (liveVerdict == GPS_LINK_ACQUIRING || liveVerdict == GPS_LINK_POSITION_ONLY) {
    showStatusCard("GPS: ACQUIRING (WAIT)", YELLOW, BLACK);
  } else {
    showStatusCard("GPS: NO DATA / CABLE", RED, WHITE);
  }
  delay(2500);
  gfx->fillRect(40, 215, 400, 52, BLACK);
  gfx->setTextColor(WHITE, BLACK);

  // Display PASS/FAIL Result Banner
  bool all_pass = true;
  if (fabs(calculateARRLDistance("CM87wa", "CM87wa") - 1.0) > 0.001) all_pass = false;
  double lat_wb, lon_wb, rover_dist, rover_bear;
  calcLatLon("CM87wb", &lat_wb, &lon_wb);
  calculateWGS84(38.354, -119.458, lat_wb, lon_wb, rover_dist, rover_bear);
  if ((int)floor(rover_dist / 1000.0) >= 16) all_pass = false;

  if (all_pass) {
    Serial.println(F("Result: PASS"));
    gfx->fillRoundRect(50, 215, 380, 52, 8, GREEN);
    gfx->setTextColor(BLACK, GREEN);
    gfx->setTextSize(3);
    gfx->setCursor(96, 230);
    gfx->print("SELF-TEST PASSED");
  } else {
    Serial.println(F("Result: FAIL"));
    gfx->fillRoundRect(50, 215, 380, 52, 8, RED);
    gfx->setTextColor(WHITE, RED);
    gfx->setTextSize(3);
    gfx->setCursor(96, 230);
    gfx->print("SELF-TEST FAILED");
  }
  delay(2500);
  gfx->fillRect(50, 215, 380, 52, BLACK);
  gfx->setTextColor(WHITE, BLACK);

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
  printLocalGrid();
  printrem();
  updateBearings();
  showFooterText(FW_VERSION, 0x7BEF);

  Serial.println(F("=== Visual Self-Test Complete (Live state restored) ===\n"));
}

void loop() {

  static char sbuff[1000];
  static int ptr = 0;

  while (1) {

    // Heading staleness watchdog: turns bearing RED after timeout if previously acquired
    if (hasHeading && currentBearingQuality != BEARING_LOST) {
      if (millis() - lastHeadingUpdate >= HEADING_TIMEOUT_MS) {
        currentBearingQuality = BEARING_LOST;
        drawCurrentBearing();
      }
    }

    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 't' || c == 'T') {
        while (Serial.available()) Serial.read();
        runVisualSelfTest();
      } else if (c == 's' || c == 'S' || c == 'g' || c == 'G' || c == 'd' || c == 'D') {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.equalsIgnoreCase("status") || cmd.equalsIgnoreCase("gps") || cmd.equalsIgnoreCase("diag") || cmd.equalsIgnoreCase("s") || cmd.equalsIgnoreCase("g")) {
          printGpsStatusReport();
        } else if (cmd.equalsIgnoreCase("test") || cmd.equalsIgnoreCase("demo")) {
          runVisualSelfTest();
        }
      } else if (c == 'h' || c == 'H' || c == '?') {
        while (Serial.available()) Serial.read();
        Serial.println(F("\n=== GNSS Compass Serial Commands ==="));
        Serial.println(F("  status (or gps) - Live GNSS receiver stream diagnostic"));
        Serial.println(F("  test            - Run visual display self-test demo"));
        Serial.println(F("  help (or ?)     - Show this help menu"));
        Serial.println(F("====================================\n"));
      } else {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.equalsIgnoreCase("test") || cmd.equalsIgnoreCase("demo")) {
          runVisualSelfTest();
        } else if (cmd.equalsIgnoreCase("status") || cmd.equalsIgnoreCase("gps") || cmd.equalsIgnoreCase("diag")) {
          printGpsStatusReport();
        } else if (cmd.equalsIgnoreCase("help") || cmd.equals("?")) {
          Serial.println(F("\n=== GNSS Compass Serial Commands ==="));
          Serial.println(F("  status (or gps) - Live GNSS receiver stream diagnostic"));
          Serial.println(F("  test            - Run visual display self-test demo"));
          Serial.println(F("  help (or ?)     - Show this help menu"));
          Serial.println(F("====================================\n"));
        }
      }
    }

    if (hasChanged) {
      hasChanged = false;

      if (isEditing) {
        lastEditActivity = millis();
        rovdist = false;
        if (counter>0) INCG(remote[remotec]);
        else DECG(remote[remotec]);

        printrem();
        updateBearings();
      }

    }

    static bool wasTouched = false;
    static unsigned long lastTouchTime = 0;
    if (tsPanel.touched()) {
      if (!wasTouched && (millis() - lastTouchTime >= 200)) {
        CST_TS_Point p = tsPanel.getPoint(0);
        if (p.x != 0 || p.y != 0) {
          wasTouched = true;
          lastTouchTime = millis();

          // Tap on local grid (Row 1): copy local grid to target for rover distance check
          if (100 <= p.x && p.x <= 340 && 0 <= p.y && p.y <= 75) {
            for (int ii = 0; ii < 6; ii++) {
              remote[ii] = mygrid[ii];
            }
            remlatitude = mylatitude;
            remlongitude = mylongitude;
            rovdist = true;
            printrem();
            updateBearings();
          }

          // Tap on target grid (Row 2): select character and enter edit mode
          if (TOUCH_X_MIN <= p.x && p.x <= TOUCH_X_MAX && TOUCH_Y_MIN <= p.y && p.y <= TOUCH_Y_MAX) {
            int touched_char = (p.x - REMSTARTX) / CHAR_W(REMSIZE);
            if (touched_char < 0) touched_char = 0;
            if (touched_char > 5) touched_char = 5;
            remotec = touched_char;
            isEditing = true;
            lastEditActivity = millis();
            printrem();
          }

          // Tap on firmware version footer (Row 7): trigger visual display demo
          if (100 <= p.x && p.x <= 380 && p.y >= 435) {
            runVisualSelfTest();
          }
        }
      }
    } else {
      wasTouched = false;
    }

    if (isEditing && EDIT_TIMEOUT_MS > 0) {
      if (millis() - lastEditActivity >= EDIT_TIMEOUT_MS) {
        isEditing = false;
        printrem();
      }
    }



    if (int num = gpsSerial.available()) {
      int num1 = gpsSerial.readBytesUntil('\n',&sbuff[ptr],num); // read all the available characters up to the end of line
      ptr += num1;
      totalGpsBytesReceived += num1;
      lastGpsByteMillis = millis();

      if (num != num1) {   // if the line ends before all the available characters were read
        gpsSerial.read();  // consume the '\n'
        totalGpsBytesReceived++;
        sbuff[ptr]=0;      // terminate the string
        Serial.println(sbuff);
        ptr=0;             // set the ptr back to 0 for the next line from the gps

        float new_heading = 0.0f;
        BearingQuality new_quality = BEARING_LOST;

        if (parseHeadingSentence(sbuff, &new_heading, &new_quality)) {
          if (strncmp(sbuff, "$KSXT", 5) == 0) {
            ksxtPacketCount++;
            lastKsxtMillis = millis();
          } else {
            hdtPacketCount++;
            lastHdtMillis = millis();
          }
          hasHeading = true;
          lastHeadingUpdate = millis();
          currentBearingQuality = new_quality;

          sinavg -= sinbuff[sin_i];
          sinavg += sinbuff[sin_i] = sin(toRad(new_heading));
          cosavg -= cosbuff[sin_i];
          cosavg += cosbuff[sin_i] = cos(toRad(new_heading));
          sin_i = (1 + sin_i) % BSZ;

          drawCurrentBearing();
        } else if (strncmp(sbuff, "$KSXT", 5) == 0 ||
                   strncmp(sbuff, "$GNTHS", 6) == 0 || strncmp(sbuff, "$GPTHS", 6) == 0 ||
                   strncmp(sbuff, "$GNHDT", 6) == 0 || strncmp(sbuff, "$GPHDT", 6) == 0) {
          if (strncmp(sbuff, "$KSXT", 5) == 0) {
            ksxtPacketCount++;
            lastKsxtMillis = millis();
          } else {
            hdtPacketCount++;
            lastHdtMillis = millis();
          }
          // Received heading sentence but no valid heading solution (e.g. baseline lost)
          if (hasHeading && currentBearingQuality != BEARING_LOST) {
            currentBearingQuality = BEARING_LOST;
            drawCurrentBearing();
          }
        }

        // Extract Time and Position from $KSXT (5 Hz)
        if (strncmp(sbuff, "$KSXT", 5) == 0) {
          int ksxt_h = 0, ksxt_m = 0;
          if (parseKSXTTime(sbuff, &ksxt_h, &ksxt_m)) {
            if (!hasTime || ksxt_h != hours || ksxt_m != minutes) {
              hasTime = true;
              hours = ksxt_h;
              minutes = ksxt_m;
              char tbuf[9];
              sprintf(tbuf, "%02d:%02dZ", hours, minutes);
              gfx->setTextSize(TIME_SZ);
              gfx->setCursor(TIME_X, TIME_Y);
              gfx->setTextColor(WHITE, BLACK);
              gfx->print(tbuf);
            }
          }

          float ksxt_lat = 0.0f, ksxt_lon = 0.0f;
          if (parseKSXTLocation(sbuff, &ksxt_lat, &ksxt_lon)) {
            if (!hasLocation || fabs(ksxt_lat - mylatitude) > 0.0001f || fabs(ksxt_lon - mylongitude) > 0.0001f) {
              hasLocation = true;
              mylatitude = ksxt_lat;
              mylongitude = ksxt_lon;
              calcLocator(mygrid, mylatitude, mylongitude);

              gfx->setTextSize(LOC_SZ);
              gfx->setCursor(LOC_X, LOC_Y);
              gfx->setTextColor(WHITE, BLACK);
              gfx->print(mygrid);

              updateBearings();
            }
          }
        } else if (strncmp(sbuff, "$GNGLL", 6) == 0) {
          gllPacketCount++;
          lastGllMillis = millis();
          char *ptr;
          char buf[9];

          parselatlon(&sbuff[7],&mylatitude,&mylongitude,&ptr);
          int gll_h = aton(ptr[0])*10.0+aton(ptr[1]);
          int gll_m = aton(ptr[2])*10.0+aton(ptr[3]);

          char *status_comma = strchr(ptr, ',');
          if ((status_comma != NULL && *(status_comma + 1) == 'A') || ptr[10] == 'A') {
            hasLocation = true;
            calcLocator(mygrid, mylatitude, mylongitude);

            gfx->setTextSize(LOC_SZ);
            gfx->setCursor(LOC_X, LOC_Y);
            gfx->setTextColor(WHITE, BLACK);
            gfx->print(mygrid);

            if (!hasTime || gll_h != hours || gll_m != minutes) {
              hasTime = true;
              hours = gll_h;
              minutes = gll_m;
              sprintf(&buf[0], "%02d:%02dZ", hours, minutes);
              gfx->setTextSize(TIME_SZ);
              gfx->setCursor(TIME_X, TIME_Y);
              gfx->setTextColor(WHITE, BLACK);
              gfx->print(buf);
            }

            updateBearings();
          }

        }
      }
    }
  }
}

