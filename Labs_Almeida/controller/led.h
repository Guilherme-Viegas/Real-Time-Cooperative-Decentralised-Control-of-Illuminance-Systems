#ifndef LED_H
#define LED

#include "util.h"

typedef class Led{
  
  private:
    byte t_pin = 0;
    byte t_final_value_pwm = 0.0;
    byte t_initial_value_pwm = 0.0;

  public:
    Led( );   // constructor
    
    void setBrightness( float v );
    float getBrightness();
    void setPin(byte pin){ t_pin = pin; } // sets the LED pin
    
}LED_;

#endif
