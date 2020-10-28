#ifndef LDR_FIT_H
#define LDR_FIT

#include "Arduino.h"
#include "TimerOne.h"

#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 
#define MAX_DIGITAL 255.0 // maximum digital value 8 bits
#define MAX_LED 100.0 // maximum digital value 8 bits


float voltage_to_lux( float v0, float m, float b );
void compute_gain( float m, short ldr_pin, float *gain, float *offset);
void steps_up( float m );
void steps_down( float m );
void tau_interruption();

#endif
