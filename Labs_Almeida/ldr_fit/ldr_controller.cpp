#include "ldr_controller.h"


// ----------------------------------------LdrController Class----------------------------------------

LdrController::LdrController( int pin ){
  Serial.println("You created a new LDR object!");
  t_pin = pin;
  }

/*
 * Sets self parameters - Gain
 * 
 * @param m static gain
 * @param b offset
 */
void LdrController::setGain( byte led_pin ){

  t_m = -0.66; // computed value
  t_b = log10(5E4); // value for the resistor during the dark
  
  compute_gain(t_m, t_pin, led_pin, &t_gain, &t_offset, &t_maxLux);
  // t_gain = 0.1067;
  // t_offset = 7.6622;
  // t_maxLux = 35.26; //255*t_gain + t_offset;
  /*small box values
   * t_gain = 1.1678;
   * t_offset = 0.0213;
   */

}

/*
 * Converts analog voltage to lux
 * 
 * @param v0, analog voltage
 * 
 * @return Lux
 */
float LdrController::voltageToLux( float v0 ){

  // normalize values
  if( v0 < 0.0 ){ v0 = 0.0; }
  else if( v0 > VCC ){ v0 = VCC; };
  
  return voltage_to_lux(v0, t_m, t_b);
}

/*
 * Converts lux to pwm or reverse
 * 
 * @param x = lux if reverse == false else pwm
 * @param reverse compute pwm = f(lux) else lux = f(pwm)
 * 
 * @return Tau
 */
float LdrController::luxToPWM( float x, bool reverse ){

  if(reverse){ return x*t_gain + t_offset; }
  else{ return ( x - t_offset ) / t_gain; } 
  
}

/* 
 * Gets the brightness [analog scale]
 *  
 * @return analog number [0 - 1023]
 */
float LdrController::getVoltage(){ return analogRead(t_pin) * VCC/MAX_ANALOG; }

/*
 * Limit Lux gap
 * 
 * @param input Lux
 * 
 * @return Bounded lux
 */
float LdrController::boundLUX( float lux ){

    lux = lux < t_offset ? t_offset : lux;
    lux = lux > t_maxLux ? t_maxLux : lux;
    return lux;
}


// ---------------------------------------------Tau Class---------------------------------------------

Tau::Tau(){ Serial.println("You created a new Tau object!"); }

/*
 * Sets quation parameters
 * 
 * @param A & B & C mathematical parameters
 */
void Tau::setParametersABC( float A, float B, float C ){ 
  // either set correct or impossible values 
  if( A >= 0 ){ t_a = A; }else{ t_a = -1.0; }
  if( B < 0 ){ t_b = B; }else{ t_b = 1.0; }
  if( C >= 0 ){ t_c = C; }else{ t_c = -1.0; }
  
  t_is_define = ( t_a != -1.0 ) and ( t_b != 1.0 ) and ( t_c != -1.0 );
  if(t_is_define){ Serial.println("The tau parameters are set."); }
}

/*
 * Gets tau from pwm
 * 
 * @param PWM
 * 
 * @return Tau
 */
float Tau::fTau( short x ){
  if( x < 0 || x > 255)
  {
    Serial.println("x must be in [ 0 ; 255 ] (pwm).");
    return -1.0;  
  }
  else if( !t_is_define )
  {
    Serial.println("The parameters are not defined. ");
    return -1.0;
  }
  else{ return t_a*exp(t_b*x)+t_c; }
}
