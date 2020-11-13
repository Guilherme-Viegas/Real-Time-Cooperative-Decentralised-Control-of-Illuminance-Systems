#include "ldr_controller.h"

#define PRINT true

// ----------------------------------------LdrController Class----------------------------------------

LdrController::LdrController( int pin ){
  // Serial.println("You created a new LDR object!");
  t_pin = pin;
  }

/*
 * Sets self parameters - Gain
 *
 * @param m static gain
 * @param b offset
 */
void LdrController::setGain( byte led_pin, float m ){

  t_m = m; // computed value
  t_bb = log10(1E5); // value for the resistor during the dark

  compute_gain( led_pin );
  // t_gain = 0.2930;
  // t_offset = 0;
  // t_maxLux = 75.30;
  //255*t_gain + t_offset;

}

/*
 * Computes the linear relation between lux and pwm
 *
 */
void LdrController::compute_gain( byte led_pin ){ 

  byte pwm = 0; // pwm to be written in led
  float flag = 1; // flag is 1 if the direction is up and 0 if it is down
  float lux = 0.0;  // lux computed 
  float voltageOut = 0.0; // voltage read in analog pin

  byte cicle_times = 1; // number of times the mountain is done
  unsigned short cicles = cicle_times*510; // number of instants per mountain
  float b_mean = 0.0; // mean of b
  float m_mean = 0.0; // mean of m
  unsigned short len_without_b = cicles - 1; // total times pwm is not 0

  Serial.print("Computing gain ...");

  for(int i=0; i<cicles; i++){  // has to start in 0
    
    analogWrite(led_pin, pwm); // sets the pwm 
    delay(50);
 
    voltageOut = analogRead(t_pin)*(VCC/MAX_ANALOG); // read V0
  
    lux = luxToOutputVoltage( voltageOut, true); // compute the lux

    if ( PRINT ){  // wites in Serial the data to compute tau in the python file
      Serial.print(pwm);
      Serial.print('\t');
      Serial.println(lux);
    }
    
    // gets lux boundaries (maxixum value, because the minimum value is the offset)
    t_maxLux = t_maxLux > lux ? t_maxLux : lux;

    if(pwm != 0){ // compute the gain in each instante
      m_mean += (lux-b_mean*(i/510 +1))/(float)pwm; // subtract the offset
    }else{  // adaptation becasue this is an indetermination but, there is never complete dark in the environment
      b_mean += lux;
    }
    
    // detect if the is reached the top or the bottom
    if(pwm == 255){ flag = -1; }  
    else if(pwm == 0){ flag = 1; }

    pwm += flag;  // update pwm value
    
   }
  
  // 'relax' the light in the box
  analogWrite(led_pin, 0); 
  delay(200);

  t_offset = b_mean/cicle_times;     // offset to compensate unpleasent light in the dark
  t_gain = m_mean/len_without_b;  // gain (G): x(t) = G*u(t)

  // assures that the maximum is possible
  t_maxLux = t_maxLux < (t_gain) * 255 + (t_offset) ? t_maxLux : (t_gain) * 255 + (t_offset) ; 

  Serial.println("Gain and offset is: Lux = " + String(t_gain, 4) + " * PWM + " + String(t_offset, 4));
  
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
  if(t_isDefine){ Serial.println("The tau parameters are set."); }
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
    Serial.print("x must be in [ 0 ; 255 ] (pwm).");
    return -1.0;
  }
  else if( t_isDefine ){ return t_a*exp(t_b*x)+t_c; }

}
