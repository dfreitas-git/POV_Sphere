
#include <Arduino.h>
#include <stdint.h>
#include <globals.h>


volatile bool motorOnOffFlag = 0;   // Turn the mhtor on/off
volatile int32_t omega_ff;      // angle counts per microsecond (Scaled by OMEGA_SHIFT for integer math)
volatile uint16_t omega_trim;   // Accumulated error between measured angle from core-0 and computed angle on core-1
volatile uint32_t measuredAngle;   // AS5600 raw
volatile bool scrollOnOffFlag = 0;   // Turn the scrolling of the image on/off

int brightness = 3;               // Sphere Global LED brightness control

//#########################################################################################################3

// Core-0 will do the actual angle measurements, Core-1 will sync to them
uint32_t lastMeasuredTime;         // micros() timestamp

// Core-1 PLL variables
int64_t angle_accum;            // 64-bits so integer math doesn't lose remainder precision
int32_t angle_q;                // current predicted angle (Q0, 0–4095)
int32_t core_1_omega_ff;        // local copy used by core-1.  Core-1 will add any necessary phase correction to it.
uint16_t core_1_omega_trim;     // local copy used by core-1.
long phase_error;               // Current error between measured angle from core-0 and computed angle on core-1

uint16_t lastAngle = 0;
float motorRPM = 0.0f;              // Current computed RPM

// Images to display on the Sphere
const uint8_t NUMBER_OF_DISPLAY_FILES = IMG_COUNT;
const char* imageToDisplay[IMG_COUNT];
