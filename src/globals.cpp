
#include <Arduino.h>
#include <stdint.h>
#include <globals.h>

volatile bool motorOnOffFlag = 0;  // Turn the mhtor on/off
volatile int32_t omega_ff;         // angle counts per microsecond (Scaled by OMEGA_SHIFT for integer math)
volatile uint16_t omega_trim;      // Accumulated error between measured angle from core-0 and computed angle on core-1
volatile uint32_t measuredAngle;   // AS5600 raw
volatile bool scrollOnOffFlag = 0; // Turn the scrolling of the image on/off
