#ifndef LED_H
#define LED

#include "Arduino.h"

typedef class Led{
  
  private:
    byte t_pin = 0;
    byte t_final_value_pwm = 0.0;
    byte t_initial_value_pwm = 0.0;

  public:
    Led( byte pin );   // constructor
    
    void setBrightness( byte pwm, boolean reference = false );
    float getInitialValuePwm();
    float getFinalValuePwm();
    void setFinalValuePwm( float pwm );
    
}LED_;

#endif
