
// All motor control and AS5600 related code

#include "motor.h"

//#######################################
// Assign pins and be sure motor is off
//#######################################
void Motor::begin() {
    ledcSetup(0, 20000, 8);   // 20 kHz PWM, 8-bit resolution
    ledcAttachPin(MOTOR_PWM_PIN, 0);
    ledcWrite(0,0);
}

//########################################
// Setter / Getter
//########################################
void Motor::setTargetRPM(float rpm) {
    targetRPM = rpm;
}

float Motor::getTargetRPM() const {
    return targetRPM;
}

float Motor::getMinRPM() const {
    return minRPM;
}

float Motor::getMaxRPM() const {
    return maxRPM;
}

Adafruit_AS5600& Motor::getSensor() {
    return as5600;
}

//######################################################################
// Update the PWM value sent to the motor via a PID loop
// rpmMeasured = measured RPM (float),  targetRPM = desired RPM (global)
//######################################################################
float Motor::updatePID(float rpmMeasured) {
    uint32_t now = millis();
    float dt = (lastPidMs == 0) ? 0.05f : ( (now - lastPidMs) / 1000.0f );
    lastPidMs = now;
    if (dt <= 0.0f) dt = 0.05f;

    // How far off are we
    float error = targetRPM - rpmMeasured;

  // Integral term (with clamping to avoid wind-up)
    pidIntegral += error * dt;

    float Iterm = Ki * pidIntegral;
    float Pterm = Kp * error;

  // Compute available headroom for Iterm
    float maxI = PWM_MAX - (Pterm + Kff * targetRPM);
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
    float alpha = dt / (DERIV_FILTER_TAU + dt);
    lastDerivative = lastDerivative + alpha * (derivative - lastDerivative);
    float Dterm = Kd * lastDerivative;

    float FFterm = Kff * targetRPM;

    // Combine for overall control value
    float output = Pterm + Iterm + Dterm + FFterm;

    // clamp final output
    if (output > PWM_MAX) output = PWM_MAX;
    if (output < PWM_MIN) output = PWM_MIN;

    // Save error for next iteration
    lastError = error;

    return output;
}

//############################################################
// Angle estimation
// Core-1 maintains it's own PLL frequency and computes the angle 
// of the Sphere every loop cycle.   
// We do actual measurements in core-0 at a more relaxed 
// rate and just sync-up core-1 PLL periodically.  That allows 
// us to remove the very slow magnetic  angle sensor 
// (AS5600) from the fast LED/DMA/render loop and replace 
// it with fast calculations so the max rpm can be increased.
//############################################################
//############################################################
void Motor::computeCurrentAngle() {
    uint32_t now = micros();
    uint32_t dt = now - lastAngleTime;
    lastAngleTime = now;

    angle_accum += (core_1_omega_ff + core_1_omega_trim) * dt;
    angle_q = (angle_accum >> OMEGA_SHIFT) & 0x0FFF;
}
