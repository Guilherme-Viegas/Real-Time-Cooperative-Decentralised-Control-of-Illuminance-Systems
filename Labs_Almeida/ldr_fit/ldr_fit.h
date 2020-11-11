#ifndef LDR_FIT_H
#define LDR_FIT

#include "Arduino.h"
#include "TimerOne.h"

#define R1 1E4  // Ω [OHM]
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define VCC 5.0 // reference voltage
#define MAX_ANALOG 1023.0 // maximum value of analog read


float voltage_to_lux( float v0, float m, float b );
void steps_up( float m );
void steps_down( float m );
void tau_interruption();

#endif
