
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

  void setMotorRPM(float rpm);
  float getMotorRPM() const;
  void setTargetRPM(float rpm);
  float getTargetRPM() const;
  float getMinRPM() const;
  float getMaxRPM() const;
  uint32_t getLastMeasuredTime();
  void setLastMeasuredTime(uint32_t time);
  uint16_t getLastAngle();
  void setLastAngle(uint16_t angle);
  uint32_t getAngle_q();
  void setAngle_q(uint32_t angle);
  uint32_t getCore_1_omega_ff();
  void setCore_1_omega_ff(uint32_t omega);
  uint16_t getCore_1_omega_trim();
  void setCore_1_omega_trim(uint16_t omega);
  long getPhase_error();
  void setPhase_error(long error);

  Adafruit_AS5600& getSensor();   // if needed elsewhere

private:

  // Core-0 will do the actual angle measurements, Core-1 will sync to them
  uint32_t lastMeasuredTime;         // micros() timestamp
  uint16_t lastAngle = 0;
  int64_t angle_accum;            // 64-bits so integer math doesn't lose remainder precision
  int32_t angle_q;                // current predicted angle (Q0, 0–4095)
  uint32_t lastAngleTime = 0;
  long phase_error;               // Current error between measured angle from core-0 and computed angle on core-1

  // --- Configuration ---
  float motorRPM = 0.0f;              // Current computed RPM
  float targetRPM = 360.0f;
  float minRPM = 200.0f;
  float maxRPM = 400.0f;
  float Kp = 0.2f;
  float Ki = 0.8f;
  float Kd = 0.02f;
  float Kff = 0.55f;

  // Core-1 PLL variables
  int32_t core_1_omega_ff;        // local copy used by core-1.  Core-1 will add any necessary phase correction to it.
  uint16_t core_1_omega_trim;     // local copy used by core-1.

  // --- Runtime PID state ---
  float pidIntegral = 0.0f;
  float lastError = 0.0f;
  float lastDerivative = 0.0f;
  uint32_t lastPidMs = 0;

  // --- Hardware ---
  Adafruit_AS5600 as5600;

};
