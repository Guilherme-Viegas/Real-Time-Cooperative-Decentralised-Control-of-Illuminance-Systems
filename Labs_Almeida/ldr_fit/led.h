#ifndef LED_H
#define LED

#include "Arduino.h"

#define MAX_DIGITAL 255.0 // maximum digital value 8 bits
#define MAX_LED 100.0 // maximum digital value 8 bit
#define VCC 5.0  // Power supply 
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits

#define unsigned char byte

class Led{
  
  private:
    byte t_pin = 0;
    byte t_final_value_pwm = 0.0;
    byte t_initial_value_pwm = 0.0;

  public:
    Led( byte pin = 0 );   // constructor
    void setBrightness( byte pwm );
    byte getInitialValuePwm();
    byte getFinalValuePwm();
};

byte get_pwm_serial();

#endif
