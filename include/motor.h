
//Headers for the motor control functions

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <globals.h>
#include <Adafruit_AS5600.h>


extern Adafruit_AS5600 as5600;

// Prototypes
float updatePID(float rpmMeasured, float targetRPM);
void setupMotor();
void computeCurrentAngle();