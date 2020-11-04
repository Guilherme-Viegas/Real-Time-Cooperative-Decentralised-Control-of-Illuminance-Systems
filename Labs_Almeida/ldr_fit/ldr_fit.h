#ifndef LDR_FIT_H
#define LDR_FIT

#include "Arduino.h"
#include "TimerOne.h"
#include "util.h"

#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3


float voltage_to_lux( float v0, float m, float b );
void compute_gain( float m, short ldr_pin, byte led_pin, float *gain, float *offset, float *max_lux );
void steps_up( float m );
void steps_down( float m );
void tau_interruption();

#endif
