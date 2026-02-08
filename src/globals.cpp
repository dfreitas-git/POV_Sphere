
#include <globals.h>

int brightness = 3;               // Sphere LED brightness
uint8_t fiveBitBright;            // hold the mapping of the menu brightness (0-10) to the dotStar five-bit brightness (0-31)

int framebufferOffset=0;         // Shift where in the frame buffer we get the column to display.  Use this to scroll the image.
unsigned long lastScrollTime;
unsigned long lastAnimateTime;

// Core-0 will do the actual angle measurements, Core-1 will sync to them
volatile uint32_t measuredAngle;   // AS5600 raw
uint32_t lastMeasuredTime;         // micros() timestamp

// Core-1 PLL variables
int64_t angle_accum;            // 64-bits so integer math doesn't lose remainder precision
int32_t angle_q;                // current predicted angle (Q0, 0–4095)
volatile int32_t omega_ff;      // angle counts per microsecond (Scaled by OMEGA_SHIFT for integer math)
volatile uint16_t omega_trim;   // Accumulated error between measured angle from core-0 and computed angle on core-1
int32_t core_1_omega_ff;        // local copy used by core-1.  Core-1 will add any necessary phase correction to it.
uint16_t core_1_omega_trim;     // local copy used by core-1.
uint32_t lastAngleTime;         // Used by core-1 to calculate dt between angle calculations
long phase_error;               // Current error between measured angle from core-0 and computed angle on core-1

// Core-1 column position vars
uint32_t nextColumnAngle = 0;
uint16_t columnIndex = 0;

uint16_t lastAngle = 0;
float motorRPM = 0.0f;
volatile bool motorOnOffFlag = 0;   // Turn the mhtor on/off

// Motor control PID parameters
float targetRPM = 360.0;
float Kp = 0.2f;
float Ki = 0.8f;
float Kd = 0.02f;
float Kff = 0.55f; // small feedforward (adjustable)

// Runtime state
float pidIntegral = 0.0f;
float lastError = 0.0f;
float lastDerivative = 0.0f; // filtered derivative
uint32_t lastPidMs = 0;  // Last time PID was updated


//########################
// Framebuffer Variables
//########################

// Double buffers allocated on heap (to make swapping trivial)
uint8_t *frontBuffer = nullptr;   // Used by core-1.  Displayed buffer (contains COLUMNS columns sequentially)
uint8_t *backBuffer  = nullptr;   // Written by core-0 (next frame)
volatile bool backBufferFilled = false;  // Set to true when a valid image has been loaded into it.
volatile bool dmaBusy = false; // indicates a DMA transfer is in flight


/* ==== Global Menu State ===== */
uint8_t currentIndex = 0;
bool editingValue = false;
uint8_t imageToDisplayIndex = 0;
uint8_t previousImageToDisplayIndex = 0;

// Demo mode - When true, rotate through all the display animations
bool demoAll = false;

/* Blink control */
bool blinkOn = true;
uint32_t lastBlink = 0;
const uint32_t blinkInterval = 400;  // ms

// Images to display on the Sphere
const uint8_t NUMBER_OF_DISPLAY_FILES = IMG_COUNT;
const char* imageToDisplay[IMG_COUNT];

volatile bool scrollOnOffFlag = 0;   // Turn the scrolling of the image on/off