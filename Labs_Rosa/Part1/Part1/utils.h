#include <Arduino.h>
#include <ctype.h>

#define R1 1E4  // Resistor
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define MAX_DIGITAL 255.0 // maximum digital value 8 bits
#define VCC 5.0  // Power supply
#define C1 1E-6




float voltage2Lux(float voltageOut, float m);
float pwm2Voltage(int pwm);
float readLux(int ldr_analog, float m);
float lux2Voltage(float lux, float m);
int voltage2pwm(float voltage);
