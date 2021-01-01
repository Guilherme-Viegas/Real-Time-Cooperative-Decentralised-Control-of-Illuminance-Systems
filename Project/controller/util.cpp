#include "util.h"

/*
 * Normalize PWM between 0 and 255
 *
 *@return PWM
 */
 
float boundPWM(float u){

    u = u < 0 ? 0 : u;
    u = u > MAX_DIGITAL ? MAX_DIGITAL : u;
    return u;
}



void initInterrupt1(){
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  // OCR1A = [16, 000, 000Hz / (prescaler * desired interrupt frequency)] - 1
  // OCR1A = [16M / (64 * 100)] - 1; 100Hz
  TCNT1 = 0;   // preload timer
  OCR1A = 2499;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << CS12);    // 64 prescaler 
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11);
 
  TIMSK1 |= (1 << OCIE1A);   // enable timer overflow interrupt
  interrupts();          // enable all interrupts
  
}

//Simple bubble sort algorithm for sorting addresses vector
void swap(byte *xp, byte *yp)  
{  
    int temp = *xp;  
    *xp = *yp;  
    *yp = temp;  
}  
  
// A function to implement bubble sort  
void bubbleSort(byte arr[], int n)  
{  
    int i, j;  
    for (i = 0; i < n-1; i++)      
      
    // Last i elements are already in place  
    for (j = 0; j < n-i-1; j++)  
        if (arr[j] > arr[j+1])  
            swap(&arr[j], &arr[j+1]);  
}

bool isValueInside(byte *vector, int vect_size, byte value) {
  for(byte i = 0; i < vect_size; i++ ) {
    if(vector[i] == value) {
      return true;
    }
  }
  return false;
}

byte retrieve_index(byte *vector, int vect_size, byte value) {
  for(byte i = 0; i < vect_size; i++ ) {
    if(vector[i] == value) {
      return i;
    }
  }
  return -1;
}

float computeNorm(float *vector, int vect_size) {
  float norm = 0;
  for(int i=0; i < vect_size; i++) {
    norm += pow(vector[i], 2);
  }
  norm = pow(norm, 0.5);
  return norm;
}
