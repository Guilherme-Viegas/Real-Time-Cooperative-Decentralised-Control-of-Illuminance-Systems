#include <Arduino.h>
#include "utils.h"
#include "pid.h"

Pid::Pid(float _T, float _p, float _i, float _d) {
  T = _T;

  Kp = _p;
  Ki = _i;
  Kd = _d;
  
  K1 = Ki * (T / 2);
  if(_d != 0) {
    K2 = Kd/(Kd + T);
  }
}

//If error is too little, return a nule error
float Pid::dead_zone(float _error) {
  if(_error > 0 and _error < 2/51) { //2/51 because _error is in voltage, we want an error basically minor than 2 pwm, so in volts 2/51
    return 0.0;
  } else if(_error < 0 and _error > - 2/51) {
    return 0.0;
  } else {
    return _error;
  }
}

float Pid::compute_pid(float y, float y_ref) {
  float error = dead_zone( y_ref - y );
  float p = Kp * error;
  float i = integral_previous + K1 * (error + error_previous);
  float d = 0;
  if(Kd != 0) {
    d = K2 * (derivative_previous - (y_ref - y_previous));
  }

  //Update the previous values...
  y_previous = y;
  integral_previous = i;
  derivative_previous = d;
  error_previous = error;

  if(is_saturated) { //If u computed is bigger than 255 or less than 0 we reset the integral term ( There was the option of Integral Saturation instead of just resetting due to discontinuities )
    i = 0;
    integral_previous = 0;
  }
  
  return p+i+d;
}

//Getters
float Pid::getT() {
  return T;
}

float Pid::get_Ki() {
  return Ki;
}

float Pid::get_Kd() {
  return Kd;
}

float Pid::get_Kp() {
  return Kp;
}

//Setters

void Pid::set_is_saturated(bool flag) {
  is_saturated = flag;
}

void Pid::setT(float _T) {
  T = _T;
}

void Pid::set_Ki(float _Ki) {
  Ki = _Ki;
}

void Pid::set_Kd(float _Kd) {
  Kd = _Kd;
}

void Pid::set_Kp(float _Kp) {
  Kp = _Kp;
}
