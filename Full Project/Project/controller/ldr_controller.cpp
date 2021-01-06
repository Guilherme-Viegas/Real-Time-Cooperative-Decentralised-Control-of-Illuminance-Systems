#include "ldr_controller.h"

#define PRINT true

// ----------------------------------------LdrController Class----------------------------------------

LdrController::LdrController( ){
  // Serial.println("You created a new LDR object!");
  }

/*
 * Sets self parameters - Gain
 *
 * @param m static gain
 * @param b offset
 */
void LdrController::setGain( byte led_pin, float m, float b ){

  t_m = m; // computed value
  t_bb = b; // value for the resistor during the dark

  //computeGain( led_pin );
  t_gain = 0.2930;
  t_offset = 0;
  t_maxLux = 75.30;
  //255*t_gain + t_offset;

}

/*
 * Converts lux to output voltage or reverse
 *
 * @param x = lux if reverse == false else vo
 * @param reverse compute vo = f(lux) else lux = f(vo)
 *
 * @return pwm if reverse == false else lux
 */
float LdrController::luxToOutputVoltage( float x, boolean reverse ){

    // unsigned short R1 = 1E4; // defined in ldr_fit.h
    float R2;

    if( !reverse ){    // Lux to Vo
        R2 = pow( 10, t_m * log10( x ) + t_bb );
        return VCC * R1 / ( R1 + R2 );
    }
    else{     // Vo to Lux
        R2 = ( VCC/x - 1) * R1;
        return pow( 10, ( log10( R2 ) - t_bb ) / t_m ) ;
    }
}

/*
 * Converts lux to pwm or reverse
 *
 * @param x = lux if reverse == false else pwm
 * @param reverse compute pwm = f(lux) else lux = f(pwm)
 *
 * @return pwm if reverse == false else lux
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
float LdrController::getOutputVoltage(){ return analogRead(t_pin) * VCC/MAX_ANALOG; }

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

Tau::Tau(){ /*Serial.println("You created a new Tau object!"); */}

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

  t_isDefine = ( t_a != -1.0 ) and ( t_b != 1.0 ) and ( t_c != -1.0 );
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
    return -1.0;
  }
  else if( t_isDefine ){ return t_a*exp(t_b*x)+t_c; }

}
