#include "ldr_controller.h"


// ----------------------------------------LdrController Class----------------------------------------

LdrController::LdrController(){ Serial.println("You created a new LDR object!"); }

/*
 * Sets self parameters - Gain
 * 
 * @param pin localion of the ldr´
 * @param m static gain
 * @param b offset
 */
void LdrController::setParametersPinMB( int pin, float m, float b ){

  t_m = m; // computed value
  t_b = b; // value for the resistor during the dark
  t_pin = pin; // analag pin
  
  // compute_gain(t_m, t_pin, &t_gain, &t_offset);
  t_gain = 1.0571;
  t_offset = 0.0213;

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
 * Print the current brightness, it is considered that lux and v0 have a linear behaviour.
 */
void LdrController::printBrightness( float luxi, float luxf, long int time, float tau ){

  luxi = luxToPWM(luxi, true);
  luxf = luxToPWM(luxf, true);

  float expoente = ( millis() - time ) / tau ;

  float lux_out = luxf - (luxf - luxi )*exp( -expoente );

  Serial.print( voltageToLux( getVoltage() ) );
  Serial.print("\t");
  Serial.println( lux_out );

  
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
