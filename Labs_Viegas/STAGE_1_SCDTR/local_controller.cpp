/* Local controller class
 * Structure for local controller with constructor arguments m, b, tau parameters...
 * Functions for local controller like simulator and tau functions
 */


#include <Arduino.h>
#include "utils.h"
#include "local_controller.h"

//using namespace std;

void Local_controller::init(int _led_pwm_port, int _ldr_analog, float _G, float _m, float _b, float _tau_a_up, float _tau_b_up, float _tau_c_up, float _tau_a_down, float _tau_b_down, float _tau_c_down, float _dead_time) {
   led_pwm_port = _led_pwm_port;
   ldr_analog = _ldr_analog;
   G = _G;
   m = _m;
   b = _b;
   tau_a_up = _tau_a_up;
   tau_b_up = _tau_b_up;
   tau_c_up = _tau_c_up;
   tau_a_down = _tau_a_down;
   tau_b_down = _tau_b_down;
   tau_c_down = _tau_c_down;
   dead_time = _dead_time;
}

float Local_controller::compute_linear_gain() {
  float readVals[52] = {0};  //52 because is 255 / 5, so 52 samples //We get points in samples of 5pwm's
  int read = -1;
  float sum_slopes = 0;

  for(int i=0; i <= 255; i+=5) { //Input PWM
      analogWrite(led_pwm_port, i);
      delay(50); //Wait for ldr to stabilize
      read = analogRead(ldr_analog);
      readVals[i/5] = voltageToLux(analogToVoltage(read), m, b); // m and b used from rect on datasheet (m=-0.9583, b=51920) //Now i'll use the calibrated m and b
      if(i>0) {
        sum_slopes += (readVals[i/5] - readVals[(i/5)-1]) / 5.0;
      }
      Serial.print(i);
      Serial.print(' ');
      Serial.println(readVals[i/5]);
  }
  return sum_slopes / 51.0;
}


//Tau function is different if we go from down to up values OR from up to down
//Now we got the tau curve fit parameters, so this function now can calculate a tau for every LUX
float Local_controller::tau_up_function(float lux) {
  return (tau_a_up * exp(tau_b_up * lux) + tau_c_up);
}

//Now for down stepping
float Local_controller::tau_down_function(float lux) {
  return (tau_a_down * exp(tau_b_down * lux) + tau_c_down);
}



//The simulator function
//vf is the final v(when its constant again
//vi is the initial v, right before we enter a pwm
//ti is the initial instant in micros, right before we enter a pwm (Arduino instant)
//t is the instant we want to know
//Because the function has (t - ti) we ca use the real arduino time at the moment of simulation or make ti=0s and t equals the time after the starting pulse
float Local_controller::simulate_Vt(float vf, float vi, long ti, long t, float lux) {
  float exp_expression = -1;
  if(vf < vi) { //Means it's a step down
    exp_expression = exp(-(t - ti) / tau_down_function(lux));
  } else { //Means it's a step up
    exp_expression = exp(-(t - ti) / tau_up_function(lux));
  }

  return (vf - (vf - vi) * exp_expression);
}

//Value of R2 depends on the LUX x, which is G*u(t), or G*PWM
//Returns R2 in Ohms (e.i 13000ohm)
float Local_controller::compute_theoric_R2(float inputPwm) {
  float function = m*(log10( inputPwm * G )) + log10(b);
  return pow(10, function);
}

void Local_controller::print_params() {
  Serial.print("LED PORT: ");
  Serial.println(led_pwm_port);
  Serial.print("LDR PORT: ");
  Serial.println(ldr_analog);

  Serial.print("LINEAR GAIN G: ");
  Serial.println(G);
  
  Serial.print("TAU A UP: ");
  Serial.println(tau_a_up);
  Serial.print("TAU B UP: ");
  Serial.println(tau_b_up);
  Serial.print("TAU C UP: ");
  Serial.println(tau_c_up);

  Serial.print("TAU A DOWN: ");
  Serial.println(tau_a_down);
  Serial.print("TAU B DOWN: ");
  Serial.println(tau_b_down);
  Serial.print("TAU C DOWN: ");
  Serial.println(tau_c_down);

  Serial.print("DEAD TIME: ");
  Serial.println(dead_time);
  Serial.println();
}

//GETTERS
float Local_controller::get_led_port() {
  return led_pwm_port;
}

float Local_controller::get_ldr_port() {
  return ldr_analog;
}

float Local_controller::getG() {
  return G;
}

float Local_controller::getM() {
  return m;
}

float Local_controller::getB() {
  return b;
}

//SETTERS

void Local_controller::setG(float _G) {
  G = _G;
}
