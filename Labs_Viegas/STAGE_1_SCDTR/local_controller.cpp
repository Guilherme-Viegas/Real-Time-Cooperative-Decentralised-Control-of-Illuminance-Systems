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
  int read_val = -1;
  float sum_slopes = 0;
  

  for(int i=0; i <= 255; i+=5) { //Input PWM
      analogWrite(led_pwm_port, i);
      delay(50); //Wait for ldr to stabilize
      read_val = analogRead(ldr_analog);
      if(i==0) {
        offset = voltageToLux(analogToVoltage(read_val), m, b);
      }
      readVals[i/5] = voltageToLux(analogToVoltage(read_val), m, b); // m and b used from rect on datasheet (m=-0.9583, b=51920) //Now i'll use the calibrated m and b
      if(i>0) {
        sum_slopes += (readVals[i/5] - readVals[(i/5)-1]) / 5.0;
      }
  }
  analogWrite(led_pwm_port, 0);
  G = (sum_slopes / 51.0);
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


//THE SIMULATOR FUNCTION
//We wanna predict the out lux of our system (ldr read) based on a pwm input (or lux, just pwm*G)
//input_lux is the lux value we pretend to achieve
//current_lux is the current lux present on the box (May be achieved by reading ldr or just knowing waht pwm we last put on led and then multiply by G)
//ti is the initial instant in micros, right before we enter a pwm (Arduino instant)
//t is the instant we want to know
//Because the function has (t - ti) we ca use the real arduino time at the moment of simulation or make ti=0s and t equals the time after the starting pulse
float Local_controller::simulate(float input_lux, float vi, long ti, long t) {
  float exp_expression = -1;
  float vf = VCC * (R1 / ( R1 + compute_theoric_R2(input_lux) ) );
  if(vf < vi) { //Means it's a step down (we're going from a bigger lux in the box to a lower one)
    exp_expression = exp(-(t - ti) / tau_down_function(input_lux));
  } else { //Means it's a step up
    exp_expression = exp(-(t - ti) / tau_up_function(input_lux));
  }

  return (vf - (vf - vi) * exp_expression);
}

//Value of R2 depends on the LUX x, which is G*u(t), or G*PWM
//Returns R2 in Ohms (e.i 13000ohm)
float Local_controller::compute_theoric_R2(float input_lux) {
  float function = m*(log10( input_lux )) + log10(b);
  return pow(10, function);
}

//GETTERS
byte Local_controller::get_led_port() {
  return led_pwm_port;
}

byte Local_controller::get_ldr_port() {
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
