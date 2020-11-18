#ifndef LED_H
#define LED

#include "util.h"

typedef class Led{
  
  private:
    byte t_pin = 0;
    byte t_final_value_pwm = 0.0;
    byte t_initial_value_pwm = 0.0;

  public:
    Led( byte pin );   // constructor
    
    void setBrightness( float v );
    float getBrightness();
    
}LED_;

#endif
