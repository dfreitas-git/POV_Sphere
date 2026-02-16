
// Class definition  for the motor control class
// Handles measuring the AS5600 motor shaft angle measurement, computing the actual
// motor RPM, controlling a PID loop to regulate the RPM and generating PLL control
// terms to send to the core-1 rendering engine which runs a virtual-motor to compute
// the column angle for rendering column pixels.

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <Adafruit_AS5600.h>
#include <globals.h>

class Motor {
public:
    void begin();

    float updatePID(float rpmMeasured);
    void computeCurrentAngle();

    void setTargetRPM(float rpm);
    float getTargetRPM() const;
    float getMinRPM() const;
    float getMaxRPM() const;

    Adafruit_AS5600& getSensor();   // if needed elsewhere

private:
    // --- Configuration ---
    float targetRPM = 360.0f;
    float minRPM = 200.0f;
    float maxRPM = 400.0f;
    float Kp = 0.2f;
    float Ki = 0.8f;
    float Kd = 0.02f;
    float Kff = 0.55f;

    // --- Runtime PID state ---
    float pidIntegral = 0.0f;
    float lastError = 0.0f;
    float lastDerivative = 0.0f;
    uint32_t lastPidMs = 0;

    // --- Hardware ---
    Adafruit_AS5600 as5600;

    // --- Internal helpers ---
    uint32_t lastAngleTime = 0;
};
