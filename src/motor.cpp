
// All motor control and AS5600 related code

#include <motor.h>

// magnetic angle sensor object
Adafruit_AS5600 as5600;  


//######################################################################
// Update the PWM value sent to the motor via a PID loop
// rpmMeasured = measured RPM (float),  targetRPM = desired RPM (global)
//######################################################################
float updatePID(float rpmMeasured, float targetRPM) {
  uint32_t now = millis();
  float dt = (lastPidMs == 0) ? 0.05f : ( (now - lastPidMs) / 1000.0f );
  lastPidMs = now;
  if (dt <= 0.0f) dt = 0.05f; // safety fallback

  // Error
  float error = targetRPM - rpmMeasured;

  // Integral term (with clamping to avoid wind-up)
  pidIntegral += error * dt;

  // compute I term candidate and clamp integral so I stays within output limits
  float Iterm = Ki * pidIntegral;

  // We'll clamp integral using a conservative bound to keep Iterm from driving output outside limits.
  // Compute allowable Iterm bounds:
  float Pterm = Kp * error;

  // approximate derivative (unfiltered) for bounds calculation:
  float derivUnfiltered = (error - lastError) / dt;
  float Dterm = Kd * lastDerivative; // use filtered derivative for display (we'll compute below)

  // Compute available headroom for Iterm
  float maxI = PWM_MAX - (Pterm + Kff * targetRPM); // conservative
  float minI = PWM_MIN - (Pterm + Kff * targetRPM);

  // Clamp Iterm then clamp pidIntegral accordingly
  if (Iterm > maxI) {
    Iterm = maxI;
    pidIntegral = Iterm / Ki;
  } else if (Iterm < minI) {
    Iterm = minI;
    pidIntegral = Iterm / Ki;
  }

  // Derivative (per second), then low-pass filter it
  float derivative = (error - lastError) / dt;

  // low-pass filter: alpha = dt / (tau + dt)
  float alpha = dt / (DERIV_FILTER_TAU + dt);
  lastDerivative = lastDerivative + alpha * (derivative - lastDerivative);
  Dterm = Kd * lastDerivative;

  // Feedforward (simple linear)
  float FFterm = Kff * targetRPM;

  // Combine
  float output = Pterm + Iterm + Dterm + FFterm;

  // clamp final output
  if (output > PWM_MAX) output = PWM_MAX;
  if (output < PWM_MIN) output = PWM_MIN;

  // Save error for next step
  lastError = error;

  // Debug print
  //Serial.printf("P:%.3f I:%.3f D:%.3f FF:%.3f -> out:%.1f (dt=%.3f)\n",
  //              Pterm, Iterm, Dterm, FFterm, output, dt);

  return output;
}

//#######################################
// Assign pins and be sure motor is off
//#######################################
void setupMotor() {
    ledcSetup(0, 20000, 8);   // 20 kHz PWM, 8-bit resolution
    ledcAttachPin(MOTOR_PWM_PIN, 0);
    ledcWrite(0,0);
}

//############################################################
// Core-1 computes the angle the Sphere every loop cycle.   
// We do actual measurements in core-0 at a more relaxed 
// rate and just sync-up core-1 periodically.  That allows 
// us to remove the very slow magnetic  angle sensor 
// (AS5600) from the fast LED/DMA/render loop and replace 
// it with fast calculations so the max rpm can be increased.
//############################################################
void computeCurrentAngle() {
    uint32_t now = micros();
    uint32_t dt = now - lastAngleTime;
    lastAngleTime = now;

    angle_accum += (core_1_omega_ff + core_1_omega_trim) * dt;
    angle_q = (angle_accum >> OMEGA_SHIFT) & 0x0FFF;
}