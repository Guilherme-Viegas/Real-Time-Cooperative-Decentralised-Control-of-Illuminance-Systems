/*
 * BASIC UTILITY FUNCTIONS
 * Computing some theorical values
 */

#include <Arduino.h>
#include "utils.h"

//Value of the R equivelent depends on R2 which depends on the LUX, or G*PWM
float compute_theoric_Req(float r2) {
  return (R1 * r2)/(R1 + r2);
}

//Theoretic value of tau, it depends on R_eq and C1, so depends on R_2 which depends on LUX or G*PWM
//Returns tau in seconds (e.i 0.0003s)
float compute_theoric_tau(float r_eq) {
  return r_eq * C1;
}

float voltageToLux(float v0, float m, float b) {
    float function = (log10((VCC/v0)*R1 - R1) - log10(b)) / m;
    float lux = pow(10, function);
    return lux;
  }

int voltageToPwm(float inputVoltage) {
    return (int)(255.0 * inputVoltage / VCC);
}

float pwmToVoltage(float inputPwm) {
    return ((float)inputPwm) * VCC / 255.0;
}

float analogToVoltage(int analogInput) {
    return ((float)analogInput) * (VCC / 1023.0);
}
