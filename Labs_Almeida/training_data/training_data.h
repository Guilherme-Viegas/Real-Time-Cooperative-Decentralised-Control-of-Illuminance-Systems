#ifndef TRAINING_DATA_H
#define TRAININGD_ATA_H

#include "Arduino.h"
#include "TimerOne.h"

#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 
#define MAX_DIGITAL 255.0 // maximum digital value 8 bits
#define MAX_LED 100.0 // maximum digital value 8 bits
#define SIZE 1000100 // just in case


float voltage_to_lux(float v0, float m);
void compute_m(float m, boolean G);
void steps_up(float m);
void steps_down(float m);
void tau_interruption();

#endif
