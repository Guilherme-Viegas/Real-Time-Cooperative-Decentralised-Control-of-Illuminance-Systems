#include "led.h"

// ---------------------------------------------Led Class---------------------------------------------

Led::Led( byte pin ){
  t_pin = 3;
  pinMode(t_pin, OUTPUT);
  Serial.println("You created a new LED object locate at pin" + String(t_pin) + "!");
}

/*
 * Sets the brightness
 *
 * @param PWM
 * @param simulator
 */
void Led::setBrightness( byte pwm, boolean reference ){
  analogWrite(t_pin, pwm);
  if(reference) setFinalValuePwm(pwm);
  }

/*
 * @return initial value of pwm (just before step)
 */
float Led::getInitialValuePwm(){ return t_initial_value_pwm; }

/*
 * @return final value of pwm (steady state)
 */
float Led::getFinalValuePwm(){ return t_final_value_pwm; }

/*
 * @param value of pwm (steady state)
 */
void Led::setFinalValuePwm( float pwm ){ 
  t_initial_value_pwm = t_final_value_pwm;
  t_final_value_pwm = pwm;
  }



// ---------------------------------------------Auxiliar Functions---------------------------------------------
