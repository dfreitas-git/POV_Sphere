
#pragma once

#include <images.h>

class Adafruit_SSD1306;
extern Adafruit_SSD1306 oled;
class Ticker;
extern Ticker encoderTicker;
class ClickEncoder;
extern ClickEncoder encoder;

struct MenuItem;
extern MenuItem menuRPM;

// Mutable globlas
extern int brightness;               // Sphere LED brightness
extern uint8_t fiveBitBright;        // hold the mapping of the menu brightness (0-10) to the dotStar five-bit brightness (0-31)

extern int framebufferOffset;         // Shift where in the frame buffer we get the column to display.  Use this to scroll the image.
extern unsigned long lastScrollTime;
extern unsigned long lastAnimateTime;

// Core-0 will do the actual angle measurements, Core-1 will sync to them
extern volatile uint32_t measuredAngle;   // AS5600 raw
extern uint32_t lastMeasuredTime;         // micros() timestamp

// Core-1 PLL variables
extern int64_t angle_accum;            // 64-bits so integer math doesn't lose remainder precision
extern int32_t angle_q;                // current predicted angle (Q0, 0–4095)
extern volatile int32_t omega_ff;      // angle counts per microsecond (Scaled by OMEGA_SHIFT for integer math)
extern int32_t core_1_omega_ff;        // local copy used by core-1.  Core-1 will add any necessary phase correction to it.
extern uint16_t core_1_omega_trim;     // local copy used by core-1.
extern long phase_error;               // Current error between measured angle from core-0 and computed angle on core-1
extern volatile uint16_t omega_trim;   // Accumulated error between measured angle from core-0 and computed angle on core-1

// Core-1 column position vars
extern uint32_t nextColumnAngle;
extern uint16_t columnIndex;

// motor RPM calculating and control
extern uint16_t lastAngle;
extern float motorRPM;
extern volatile bool motorOnOffFlag;   // Turn the mhtor on/off

/* ==== Global Menu State ===== */
extern uint8_t imageToDisplayIndex;
extern uint8_t previousImageToDisplayIndex;

// Images to display on the Sphere
extern const uint8_t NUMBER_OF_DISPLAY_FILES;
extern const char* imageToDisplay[];

extern volatile bool scrollOnOffFlag;   // Whether or not the sphere is rotating
extern bool demoAll;  // Run each image for 10 seconds on the sphere



// Constant globals

// ###########################################################
//   DotStar, LED, DMA, critical timing code running on core-1
// ###########################################################
constexpr uint8_t BRIGHTNESS_R = 200;  // 0–255 (≈75%)  Use this to brighten/dim LED's after gamma correction
constexpr uint8_t BRIGHTNESS_G = 225;  // 0–255 (≈75%)  Use this to brighten/dim LED's after gamma correction
constexpr uint8_t BRIGHTNESS_B = 255;  // 0–255 (≈75%)  Use this to brighten/dim LED's after gamma correction

constexpr int PIN_SPI_MOSI = 13;  // MOSI ( DATA)
constexpr int PIN_SPI_SCLK = 14;  // SCLK ( CLK)
constexpr int RING0_ENB = 2;       // SN74AHCT125 bus driver bit 0 select
constexpr int RING1_ENB = 4;       // SN74AHCT125 bus driver bit 1 select
constexpr int RING2_ENB = 16;      // SN74AHCT125 bus driver bit 2 select
constexpr int RING3_ENB = 17;      // SN74AHCT125 bus driver bit 3 select

// LED / frame geometry
constexpr uint32_t AS5600_COUNTS = 4096;      // number of counts in 360 degrees from the AS5600 sensor
constexpr int ROWS = 48;                      // vertical rows (LEDs per ring)
constexpr uint32_t COLUMNS = 120;             // total angular columns in a revolution
constexpr int RINGS = 4;                      // number of LED rings
constexpr int COLS_PER_RING = COLUMNS/RINGS;  // number of columns each ring fills
constexpr int ringEnable[] = {RING0_ENB, RING1_ENB, RING2_ENB, RING3_ENB};  // Pins that control the mux for the serial DMA

// dotStar specifics
constexpr int BYTES_PER_LED = 4;       // dotStart uses 4 bytes per LED (global, B, G, R in common libs)
constexpr int START_FRAME_BYTES = 4;
constexpr int END_FRAME_BYTES = 4;
constexpr int LEDS_PER_COLUMN = ROWS;
constexpr int FRAME_COLUMN_BYTES = LEDS_PER_COLUMN * 3; // one byte per R/G/B
constexpr int TOTAL_COLUMNS_RGB_BYTES = COLUMNS * LEDS_PER_COLUMN * 3;  // total number of bytes in the framebuffers

// We'll build each column as: [4-byte start frame][48 * 4 bytes LED frames][4-byte end frame]
constexpr int COLUMN_PAYLOAD = START_FRAME_BYTES + (LEDS_PER_COLUMN * BYTES_PER_LED) + END_FRAME_BYTES;
constexpr int TOTAL_COLUMNS_BYTES = COLUMNS * COLUMN_PAYLOAD;

constexpr uint8_t SCROLL_UPDATE_TIME = 100;   // How often (in milliseconds) to update the framebuffer offset pointer.  Controls how fast the image scrolls around the Sphere.
constexpr uint16_t DEMO_DISPLAY_TIME = 10000; // Amount of time we give each animation to display during the demo mode.
constexpr uint8_t MAX_TRIM = 2;               // The most we allow core-0 to adjust the angle being computed by core-1

// Core-1 angle values calculated and pll-locked to core-0 actual angle measurements
constexpr uint8_t  OMEGA_SHIFT = 16;
constexpr uint32_t OMEGA_TRIM_PERIOD = 30000000;  // The amount of time (in microseconds) we'd want to apply frequency trim over to our core-1 VCO
constexpr uint8_t  PHASE_DEADBAND = 10;           // Any angle error less than this and we don't apply any more correction to the VCO

// Use integer math; keep remainder for precision
constexpr uint32_t COLUMN_STEP = AS5600_COUNTS / COLUMNS;      // 34
constexpr uint32_t COLUMN_REM  = AS5600_COUNTS % COLUMNS;      // 16

//##########################
// PID motor speed control
//##########################
constexpr int MOTOR_PWM_PIN = 25;           // To control the motor speed
constexpr uint32_t samplePeriod_us = 20000; // minimum time (us) between sampling the angle measurement in core-0
constexpr uint8_t PID_UPDATE_TIME = 100 ;   // How many milliseconds between updating the motor PWM.

// Output limits (map to motor PWM range)
constexpr float PWM_MIN = 125.0f;
constexpr float PWM_MAX = 255.0f;

// derivative low-pass (seconds). 0.01..0.2 typical
constexpr float DERIV_FILTER_TAU = 0.05f; 


// ###########################################################
//   UI core-0 OLED/rotary-encoder/switch definitions
// ###########################################################
/* ==== OLED Size and pins ======== */
constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr uint8_t SDA_OLED = 32;
constexpr uint8_t SCL_OLED = 33;
constexpr uint8_t OLED_RESET =    -1 ;    // no reset pin
constexpr uint8_t OLED_ADDR =     0x3C;

/* ==== Encoder pins ====== */
constexpr uint8_t ENC_A = 35;
constexpr uint8_t ENC_B = 34;
constexpr uint8_t ENC_BTN = 27;
