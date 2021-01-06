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
