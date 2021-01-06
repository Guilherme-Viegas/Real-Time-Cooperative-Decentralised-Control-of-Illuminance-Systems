#include <Arduino.h>
#include "pid.h"

pid::pid(float _T, float _kp, float _ki, float _kd, float _a){
    kp = _kp;
    ki = _ki;
    kd = _kd;
    T = _T;
    a = _a;
}
pid::~pid(){}
void pid::printParam(){
  Serial.println(String(T) + " " + String(kp) + " " + String(ki) + " " + String(kd) + " " + String(a));
}


int pid::ref2pwm(float ref, LDR simulator){
    float ref_pwm = simulator.lux2pwm(ref);
    if(ref_pwm > 255){
      ref_pwm = 255; 
    }
    else if(ref_pwm < 0){
      ref_pwm = 0;
    }
    return ref_pwm;
}
int pid::saturation(int u){
  u = (u < 0) ? 0 : u;
  u = (u > 255) ? 255 : u;
  return u;
}
float pid::getKp(){
  return kp;  
}
float pid::getKi(){
  return ki;  
}
float pid::getT(){
  return T;  
}
