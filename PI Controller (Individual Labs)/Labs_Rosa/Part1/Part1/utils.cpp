#include "utils.h"

//converts voltage to LUX
float voltage2Lux(float voltageOUT, float m){
  float b = 2E4;
  float lux = pow(10, (log10((VCC/voltageOUT)*R1 - R1) - log10(b)) / m);
  return lux;
};

float pwm2Voltage(int pwm){
  return float(VCC*pwm/MAX_DIGITAL);
}

float readLux(int ldr_analog, float m){
  float sensor = analogRead(ldr_analog);
  float reading = voltage2Lux(sensor*VCC/MAX_ANALOG, m);
  return reading;
}

float lux2Voltage(float lux, float m){
  float b = 2e4;
  float R2 = pow(10, m*log10(lux) + log10(b));
  return VCC*R1/(R1 + R2);
}

int voltage2pwm(float voltage){
  return voltage*MAX_DIGITAL/VCC;
}
