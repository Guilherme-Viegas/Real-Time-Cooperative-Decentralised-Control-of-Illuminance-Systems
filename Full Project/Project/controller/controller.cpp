#include "controller.h"

/*
 * Writes values in EEPROM
 * Initialize the interruption
 *
 * @param Kp proportional gain
 * @param Ki integral gain
 * @param Kd derivative gain
 */
ControllerPid::ControllerPid( byte pin_led, int pin_ldr ){
  // Serial.println("You created a new PID controller object!");
  // NICE VALUES: kp = 0.75; ki = 0.025;
  t_kp = 0.25;
  t_ki = 0.019;

  // LED and LDR pins
  t_ldrPin = pin_ldr;
  t_ledPin = pin_led;

  // SETS pins 
  led.setPin( t_ledPin );
  ldr.setPin( t_ldrPin );

}

/*
 * Computes the feedback signal
 *
 * @param output system output in analog scale

 */
void ControllerPid::computeFeedbackGain( float output ){
  noInterrupts();

  float sim = simulator(false); // simulator response (volt)
  
  output =  output * VCC/MAX_ANALOG; // output (volt)
  
  float error = sim - output;  // determines the error between the system output and the reference value (Volt)
  // Serial.println(getU(), 5);

  
  if( t_deadZone){// DEAD ZONE
    
    if (abs(error) <  2*VCC/MAX_DIGITAL){ // apply dead zone if the error is less than 2* 5 / 255 (V/PWM) : 

      if( t_reference > 2 && abs(error) > 0.5*VCC/MAX_DIGITAL){ /*exit*/ } // exits deadzone if the error is more than 0.5 * VCC/MAX_DIGITAL for pwm greater than 10.  
      else{
        error = 0;  // dead zone
        static float b = ldr.get_offset(); // minimum value
        if( t_reference == b ){  t_integralReset = true; }  // when the led is off, the system can not absorve energy, so it will never steady by itself!
      }
    }
    
  }

  if( t_integralReset){   // reset integral
    t_integralReset = false;  // integral available
    t_uInt = 0; // resets cumalative error
    t_lastError = 0;  // resets previous error
    
  }else{  // Itegral term
    // t_uInt += t_ki * error * t_sampleTime; // compute integral through naīve
    t_uInt += (t_ki*t_sampleTime/2)*(error+t_lastError); // compute integral through Tustin

    if( t_antiWindUp ){ // apllies anti wind up window
      float intLimitMax = VCC - t_kp*error - t_uff ; // the saturation when there is dark - the output is VCC [V]
      float intLimitMin = 0 - t_kp*error - t_uff; // the saturation when there is too much bright - the output is 0 [V]
      t_uInt = t_uInt > intLimitMax ? intLimitMax : ( t_uInt < intLimitMin ? intLimitMin : t_uInt ); // integral saturation
    }
  }
  t_lastError = error;
  t_ufb = t_kp*error + t_uInt; // return the feedback signal 

  interrupts();
}

/*
 * Sets Reference lux level
 *
 * @param reference desire illumination
 */
void ControllerPid::setReferenceLux( float reference, float pwm ){
  noInterrupts();

  //reference = ldr.boundLUX( reference );  // bound lux value

  t_lastReference = t_reference;  // updates reference
  t_reference = reference; // set reference  [Lux]
  t_integralReset = true; // reset integral error

  //float pwm = boundPWM( ldr.luxToPWM( t_reference ) );  // converts to PWM

  setUff( pwm ); // sets feedfoward signal

  interrupts();
}


/*
 *Sets feedfowward signal Volt
 */
void ControllerPid::setUff( float uff ){
  
    t_uff = t_feedfoward ? uff * VCC/MAX_DIGITAL : ldr.get_offset(); // feedfoward impulse
    t_ufb = 0; // forget last feedback
    t_to = millis();
}

/*
 * Get time of the new feedfoward input
 *
 * @return Time
 */
unsigned long ControllerPid::get_to(){ return t_to; }

/*
 * Return system response PWM
 */
float ControllerPid::getU(){
    float u = t_ufb + t_uff;  // compute u
    
    return round( boundPWM(u * MAX_DIGITAL/VCC) ) ;
}

/*
 * Get Ldr Pin
 *
 * @return Ldr Pin
 */
int ControllerPid::getLdrPin(){ return t_ldrPin; }

/*
 * Get Led Pin
 *
 * @return Led Pin
 */
byte ControllerPid::getLedPin(){ return t_ledPin; }


/*
 * Compute simulator
 *
 * @return Desired output voltage
 */
float ControllerPid::simulator( boolean print ){

  // avoid minor errors
  float luxi = boundPWM( ldr.luxToPWM( t_lastReference ) );
  float luxf = boundPWM( ldr.luxToPWM( t_reference ) ) ;

  float tau = luxf > luxi ? ldr.t_tau_up.fTau( luxf ) : ldr.t_tau_down.fTau( luxf );  // th error should be upper than the 1pwm 

  luxi = ldr.luxToOutputVoltage( t_lastReference );
  luxf = ldr.luxToOutputVoltage( t_reference );

  float expoente = ( millis() - t_to ) / tau ;

  float lux_out = luxf - (luxf - luxi )*exp( -expoente );

  if(print){
    // Reference
    Serial.print( t_reference );  // Lux
    Serial.print("\t");

    // simulator
    Serial.print( ldr.luxToOutputVoltage( lux_out, true ) );  // LUX
    Serial.print("\t");

    // led 
    Serial.print( ldr.luxToPWM( getU() , true ) );  // LUX
    Serial.print("\t"); 
  }
  
  return lux_out ;  // simulator value in Volt
}

/*
 * Print the system response
 */
void ControllerPid::output(){

  t_sum -= t_output[t_counter%t_meanSize]; // remove the last value
  t_output[t_counter%t_meanSize] = ldr.luxToOutputVoltage( ldr.getOutputVoltage(), true ); // read the new value
  t_sum += t_output[t_counter%t_meanSize]; // compute the sum
  t_counter ++; // updates new value
  
  //Serial.println( t_sum/t_meanSize ); // LUX - mean
  
}
