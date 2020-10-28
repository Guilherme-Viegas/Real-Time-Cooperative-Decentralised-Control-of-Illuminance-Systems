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
 */
void Led::setBrightness( byte pwm ){
  
  analogWrite(t_pin, pwm);
  t_initial_value_pwm = t_final_value_pwm;
  t_final_value_pwm = pwm;
}

/*
 * @return initial value of pwm (just before step)
 */
byte Led::getInitialValuePwm(){ return t_initial_value_pwm; }

/*
 * @return final value of pwm (steady state)
 */
byte Led::getFinalValuePwm(){ return t_final_value_pwm; }


// ---------------------------------------------Auxiliar Functions---------------------------------------------

/*
 * Reads Serial and convert to PWM 
 * 
 * @return PWM
 */
byte get_pwm_serial(){

  String work_percentage = Serial.readString();  // read input at PWM pin 'led_pin'
  return ( work_percentage.toFloat() *(MAX_DIGITAL/MAX_LED) );
}
