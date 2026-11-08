#pragma once

#include <cstdint>

// ==============================================================================
// Firmware Version
// ==============================================================================
#ifndef FW_VERSION
#define FW_VERSION           "v0.3-dev"
#endif

// ==============================================================================
// Hardware Pins & Peripherals
// ==============================================================================
// I2C Pins & Addresses
#define I2C_SDA_PIN          38
#define I2C_SCL_PIN          39
#define BKL_PIN              6
#define I2C_TOUCH_ADDR       0x15
#define PCF8574_ADDR         0x21
#define PCF8574_I2C_ADDR     0x21

// Hardware UART Pins for GNSS Receiver (Serial2)
#define RXD2                 44
#define TXD2                 43
#define GPS_BAUD             115200

// Rotary Encoder Pins
#define ENCODER_CLK          42
#define ENCODER_DT           4

// ==============================================================================
// Color Palette (RGB565)
// ==============================================================================
#ifndef BLACK
#define BLACK                0x0000
#endif
#ifndef WHITE
#define WHITE                0xFFFF
#endif
#ifndef RED
#define RED                  0xF800
#endif
#ifndef GREEN
#define GREEN                0x07E0
#endif
#ifndef BLUE
#define BLUE                 0x001F
#endif
#ifndef CYAN
#define CYAN                 0x07FF
#endif
#ifndef MAGENTA
#define MAGENTA              0xF81F
#endif
#ifndef YELLOW
#define YELLOW               0xFFE0
#endif

// Sunlight-optimized golden amber for Float GNSS quality
// (RGB565: R=31, G=53, B=0 = 0xFEA0, ~#FFD400)
#define RICH_YELLOW          0xFEA0

// UI Theme Colors
#define DARK_GRAY_BORDER     0x39E7
#define DIM_GRAY_TEXT        0x7BEF
#define MODAL_BG_COLOR       0x10A2
#define MODAL_CARD_BG        0x0841
#define BUTTON_BLUE_BG       0x1B19
#define BUTTON_CYAN_BORDER   0x3DFF
#define BUTTON_MUTED_BG      0x2104
#define BUTTON_RESET_BG      0x5000
#define BUTTON_CANCEL_BG     0x2945
#define BUTTON_SAVE_IDLE_BG  0x0320
#define BUTTON_SAVE_HOLD_BG  0x0540

// ==============================================================================
// Display Dimensions & Layout (480 x 480 Round Display)
// ==============================================================================
#define SCREEN_WIDTH         480
#define SCREEN_HEIGHT        480

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

// Target Grid Touch Area (covers character row, caret row, and padding)
#define TOUCH_X_MIN          50
#define TOUCH_X_MAX          430
#define TOUCH_Y_MIN          70
#define TOUCH_Y_MAX          235

// Row 3: Antenna Heading & Target Bearing
#define BEARSZ               6
#define BEARX                20
#define BEARY                REMSTOPY
#define TGTBEARX             280

// Row 3 Center: Calibration Status Pill & Touch Target
#define CAL_PILL_X           200
#define CAL_PILL_Y           225
#define CAL_PILL_W           80
#define CAL_PILL_H           44
#define CAL_TOUCH_X_MIN      180
#define CAL_TOUCH_X_MAX      300
#define CAL_TOUCH_Y_MIN      210
#define CAL_TOUCH_Y_MAX      280

// Set to 1 to show ARRL contest distance (Row 6), 0 to hide
#define ENABLE_ARRL_DIST     0

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
#define REVBEAR_Y            376
#endif

// Row 7: Firmware Version Footer
#define VER_SZ               2
#define VER_Y                444

// Modal Dialog Geometry
#define CAL_MODAL_X              35
#define CAL_MODAL_Y              75
#define CAL_MODAL_W              410
#define CAL_MODAL_H              335

// Modal Buttons
#define CAL_BTN_SNAP_X           55
#define CAL_BTN_SNAP_Y           234
#define CAL_BTN_SNAP_W           180
#define CAL_BTN_SNAP_H           40

#define CAL_BTN_RESET_X          245
#define CAL_BTN_RESET_Y          234
#define CAL_BTN_RESET_W          180
#define CAL_BTN_RESET_H          40

#define CAL_BTN_SAVE_X           55
#define CAL_BTN_SAVE_Y           284
#define CAL_BTN_SAVE_W           370
#define CAL_BTN_SAVE_H           48

#define CAL_BTN_CANCEL_X         140
#define CAL_BTN_CANCEL_Y         342
#define CAL_BTN_CANCEL_W         200
#define CAL_BTN_CANCEL_H         36

// Modal Touch Hit Areas
#define CAL_TOUCH_SNAP_X_MIN     55
#define CAL_TOUCH_SNAP_X_MAX     235
#define CAL_TOUCH_SNAP_Y_MIN     230
#define CAL_TOUCH_SNAP_Y_MAX     278

#define CAL_TOUCH_RESET_X_MIN    245
#define CAL_TOUCH_RESET_X_MAX    425
#define CAL_TOUCH_RESET_Y_MIN    230
#define CAL_TOUCH_RESET_Y_MAX    278

#define CAL_TOUCH_SAVE_X_MIN     55
#define CAL_TOUCH_SAVE_X_MAX     425
#define CAL_TOUCH_SAVE_Y_MIN     284
#define CAL_TOUCH_SAVE_Y_MAX     332

#define CAL_TOUCH_CANCEL_X_MIN   130
#define CAL_TOUCH_CANCEL_X_MAX   350
#define CAL_TOUCH_CANCEL_Y_MIN   340
#define CAL_TOUCH_CANCEL_Y_MAX   385

// Normal Mode Touch Hit Areas
#define TOUCH_LOCAL_GRID_X_MIN   100
#define TOUCH_LOCAL_GRID_X_MAX   340
#define TOUCH_LOCAL_GRID_Y_MIN   0
#define TOUCH_LOCAL_GRID_Y_MAX   75

#define TOUCH_FOOTER_X_MIN       100
#define TOUCH_FOOTER_X_MAX       380
#define TOUCH_FOOTER_Y_MIN       435
#define TOUCH_FOOTER_Y_MAX       480

// ==============================================================================
// Rover Mode & History Configuration
// ==============================================================================
#define ROVER_CFG_NAMESPACE          "rover_cfg"
#define ROVER_CFG_KEY_MAX_STOPS      "max_stops"
#define ROVER_LOG_NAMESPACE          "rover_log"
#define ROVER_LOG_KEY_COUNT          "count"

#define DEFAULT_MAX_ROVER_STOPS      100
#define MIN_ROVER_STOPS              10
#define ABS_MAX_ROVER_STOPS          500

// Row 5 Rover Qualification Card Layout & Colors (RGB565)
#define ROVER_CARD_X                 40
#define ROVER_CARD_Y                 372
#define ROVER_CARD_W                 400
#define ROVER_CARD_H                 44
#define ROVER_CARD_R                 8

#define COLOR_ROVER_TRANSIT_RED      0xF800
#define COLOR_ROVER_QUALIFIED_GREEN  0x03E0
#define COLOR_ROVER_REVISIT_BLUE     0x011C
#define COLOR_ROVER_NOGPS_AMBER      0xDC00

// ==============================================================================
// Timeouts & Operational Thresholds
// ==============================================================================
#define HEADING_TIMEOUT_MS        1500
#define EDIT_TIMEOUT_MS           5000
#define CAL_MODAL_TIMEOUT_MS      15000
#define LONG_PRESS_MS             1500
#define ROVER_ALERT_THRESHOLD_KM  16.0

// ==============================================================================
// Character Rolling Helpers (for Target Grid manual editing)
// ==============================================================================
#define INCA(x) (((x+1)>'Z')?'A':(x+1))
#define DECA(x) (((x-1)<'A')?'Z':(x-1))
#define INCN(x) (((x+1)>'9')?'0':(x+1))
#define DECN(x) (((x-1)<'0')?'9':(x-1))
#define INCG(x) (x=(x>='0' && x<='9')?INCN(x):INCA(x))
#define DECG(x) (x=(x>='0' && x<='9')?DECN(x):DECA(x))

// ASCII to numeric digit conversion (single-evaluation, type-safe inline)
inline int charToDigit(char c) {
    return (c >= '0' && c <= '9') ? (c - '0') : 0;
}
#define aton(x) charToDigit(x)
