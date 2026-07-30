#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <PCF8574.h>
#include <Adafruit_CST8XX.h>
#include <math.h>

// I2C to PCF8574
#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39
#define BKL_PIN 6

#define I2C_TOUCH_ADDR 0x15
#define REMSTARTX 80
#define REMSTARTY 100
#define INCA(x) (((x+1)>'Z')?'A':(x+1))
#define DECA(x) (((x-1)<'A')?'Z':(x-1))
#define INCN(x) (((x+1)>'9')?'0':(x+1))
#define DECN(x) (((x-1)<'0')?'9':(x-1))
#define REMSIZE 8
#define REMSTOPX (REMSIZE*6*6+REMSTARTX)
#define REMSTOPY (REMSIZE*8*2+REMSTARTY)
#define BEARSZ 6
#define BEARX 30
#define BEARY REMSTOPY
#define INCG(x) (x=(x>='0' && x<='9')?INCN(x):INCA(x))
#define DECG(x) (x=(x>='0' && x<='9')?DECN(x):DECA(x))

#define aton(x) (x-'0')

 // Define the RX and TX pins for Serial 2
#define RXD2 44
#define TXD2 43

#define GPS_BAUD 115200

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

char remote[7] = "AA00AA";
char base[9] = "AA00AA00";

#define ENCODER_CLK 42  
#define ENCODER_DT 4   

volatile int counter=0;
volatile int encState = 0;
volatile int oldState = -1;
volatile bool hasChanged = true;

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


void calcLatLon(char *dst, float *lat, float *lon) {

  *lon = (dst[0]-'A')*20.0+(dst[2]-'0')*2.0+(dst[4]-'A')*.08333333-180.0+.08333333/2.0;
  *lat = (dst[1]-'A')*10.0+(dst[3]-'0')*1.0+(dst[5]-'A')*.04166666-90.0+.04166666/2.0;
  
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
  analogWrite(BKL_PIN, 200);
}

void initPins() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pcf8574.pinMode(P0, OUTPUT);        //tp RST
  pcf8574.pinMode(P2, OUTPUT);        //tp INT
  pcf8574.pinMode(P3, OUTPUT);        //lcd power
  pcf8574.pinMode(P4, OUTPUT);        //lcd reset
  pcf8574.pinMode(P5, INPUT_PULLUP);  //encoder SW

  if (!pcf8574.begin()) {
    gpsSerial.println("Can't init pcf8574");
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
}


void initLCD() {
  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE, BLACK);

}

int remotec;

void printrem() {

  gfx->setTextSize(REMSIZE); 
  gfx->setCursor(REMSTARTX,REMSTARTY);
  gfx->print(remote);
  gfx->setCursor(REMSTARTX,REMSTARTY+REMSIZE*8);
  gfx->print("      ");
  gfx->setCursor(REMSTARTX+remotec*6*REMSIZE,REMSTARTY+REMSIZE*8);
  gfx->print("^");
  
}



// Convert degrees to radians
double toRad(double degree) {
    return degree * M_PI / 180.0;
}

// Convert radians to degrees
double toDeg(double rad) {
    return rad * 180.0 / M_PI;
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




#define BSZ 5
float sinbuff[BSZ],cosbuff[BSZ];

void setup() {

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.begin (115200);
  
  for (int i = 0; i < BSZ; i++)
    sinbuff[i] = cosbuff[i] = 0.0;
    
  initPins();
  initBacklight();
  initLCD();
  initEncoder();
  printrem();
  
}

void loop() {

  static int i = 0;
  static float sinavg = 0.0,cosavg=0.0;
  static char sbuff[1000];
  static int ptr = 0;
  static float mylongitude,mylatitude;
  static float remlongitude,remlatitude;
  static int hours,minutes;
  static double distance, bearing, revbearing;
  
  while (1) {

    if (hasChanged) {
      hasChanged = false;
      
      if (counter>0) INCG(remote[remotec]);
      else DECG(remote[remotec]);

      printrem();
      calcLatLon(remote,&remlatitude,&remlongitude);
      calculateWGS84(mylatitude,mylongitude, remlatitude, remlongitude,distance,bearing);
      calculateWGS84(remlatitude,remlongitude, mylatitude, mylongitude,distance,revbearing);

      char buf[20];
      sprintf(&buf[0],"%5.1f %4i k  ",revbearing,(int)(distance/1000));
      gfx->setTextSize(4);
      gfx->setCursor(90,400);
      gfx->print(buf);
      
    }
    
    if (tsPanel.touched()) {
      CST_TS_Point p = tsPanel.getPoint(0);
      if (REMSTARTX<p.x && p.x<REMSTOPX && REMSTARTY<p.y && p.y<REMSTOPY){
        remotec = (p.x-REMSTARTX) / (REMSIZE*6);
        printrem();
      }
    }


    
    if (int num = gpsSerial.available()) {
      int num1 = gpsSerial.readBytesUntil('\n',&sbuff[ptr],num); // read all the available characters up to the end of line
      ptr += num1;
      if (num != num1) {   // if the line ends before all the available characters were read
        gpsSerial.read();  // consume the '\n'
        sbuff[ptr]=0;      // terminate the string
        ptr=0;             // set the ptr back to 0 for the next line from the gps
        
        if (strncmp(sbuff, "$GNHDT",6) == 0) {
          sinavg -= sinbuff[i];
          sinavg += sinbuff[i] = sin(toRad(strtof(&sbuff[7], NULL)));
          cosavg -= cosbuff[i];
          cosavg += cosbuff[i] = cos(toRad(strtof(&sbuff[7], NULL)));
         
          i = (1 + i) % BSZ;

          char str[20];
          sprintf(&str[0], "%5.1f %5.1f", fmod(((toDeg(atan2(sinavg/BSZ, cosavg/BSZ))) + 270.0),360.0),bearing);
          gfx->setCursor(BEARX,BEARY);
          gfx->setTextSize(BEARSZ);
          gfx->print(&str[0]);

        }

        if (strncmp(sbuff, "$GNGLL",6) == 0) {
          char *ptr;
          char buf[9];
                    
          parselatlon(&sbuff[7],&mylatitude,&mylongitude,&ptr);
          hours = aton(ptr[0])*10.0+aton(ptr[1]);
          minutes = aton(ptr[2])*10.0+aton(ptr[3]);

          if (ptr[10]=='A') {
          
            gfx->setCursor(140,40);
            calcLocator(&buf[0],mylatitude,mylongitude);
            gfx->setTextSize(4);
            gfx->print(buf);

            sprintf(&buf[0],"%02d:%02d",hours,minutes);
            gfx->setCursor(140,300);
            gfx->setTextSize(6);
            gfx->print(buf);
         }

        }
      }
    }
  }
}
