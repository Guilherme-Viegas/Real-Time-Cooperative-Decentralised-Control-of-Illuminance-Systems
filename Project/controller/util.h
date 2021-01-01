#ifndef UTIL_H
#define UTIL

#include "Arduino.h"

#define MAX_DIGITAL 255.0 // maximum digital value 8 bits
#define MAX_LED 100.0 // maximum digital value 8 bit
#define VCC 5.0  // Power supply
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define R1 1E4  // Resistor

float boundPWM(float u);
void initInterrupt1();
ISR(TIMER1_COMPA_vect);
void bubbleSort(byte arr[], int n);
void swap(int *xp, int *yp);
bool isValueInside(byte *vector, int vect_size, byte value);
byte retrieve_index(byte *vector, int vect_size, byte value);
float computeNorm(float *vector, int vect_size);

#endif
