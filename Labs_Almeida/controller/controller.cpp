#include "controller.h"


/*
 * Writes values in EEPROM
 * Initialize the interruption
 *
 * @param Kp proportional gain
 * @param Ki integral gain
 * @param Kd derivative gain
 */
ControllerPid::ControllerPid( byte led, int ldr ){
  // Serial.println("You created a new PID controller object!");
  t_kp = 0.75;
  t_ki = 0.025;

  // LED and LDR pins
  t_ldrPin = ldr;
  t_ledPin = led;

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
  
  if( t_deadZone && abs(error) < VCC/MAX_DIGITAL ){ // apply dead zone if the error is less than 1 = 5 / 255 (V/PWM)
   
    error = 0;  // dead zone
    static float b = ldr.get_offset(); // minimum value
    if( t_reference == b ){  t_uInt = 0; }  // when the led is off, the system can not absorve energy, so it will never steady by itself!
  }

  if( t_integralReset){ // resets cumalative error
    t_integralReset = false;
    t_uInt = 0;
    
  }else{  // Itegral term
    t_uInt += t_ki * error * t_sampleTime; // compute integral

    if( t_antiWindUp ){ // apllies anti wind up window
      float intLimitMax = VCC - t_kp*error - t_uff ; // the saturation when there is dark - the output is VCC [V]
      float intLimitMin = 0 - t_kp*error - t_uff; // the saturation when there is too much bright - the output is 0 [V]
      t_uInt = t_uInt > intLimitMax ? intLimitMax : ( t_uInt < intLimitMin ? intLimitMin : t_uInt ); // integral saturation
    }
  }
 
  t_ufb = t_kp*error + t_uInt; // return the feedback signal 
  interrupts();
}

/*
 * Sets Reference lux level
 *
 * @param reference desire illumination
 */
void ControllerPid::setReferencePWM( float reference ){
  noInterrupts();

  float pwm = boundPWM( reference );
  setUff( pwm ); // sets feedfoward signal

  reference = ldr.boundLUX( ldr.luxToPWM( reference, true ) );  // converts to lux
  
  t_lastReference = t_reference;  // updates reference
  t_reference = reference; // set reference  [Lux]
  t_integralReset = true; // reset integral error

  interrupts();
}


/*
 *Sets feedfowward signal Volt
 */
void ControllerPid::setUff( float uff ){
  
    t_uff = t_feedfoward ? uff * VCC/MAX_DIGITAL : ldr.get_offset(); // feedfoward impulse
    t_to = millis();
}

/*
 * Get time of the new feedfoward input
 *
 * @return Time
 */
unsigned long ControllerPid::get_to(){ return t_to; }

/*
 * Return system response Volt
 */
float ControllerPid::getU(){  
    float u = t_ufb + t_uff;
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
    Serial.print( t_reference );  // Voltlux
    // Serial.print( ldr.luxToOutputVoltage(t_reference) *204.6 ); // PWM
    Serial.print("\t");

    // simulator
    Serial.print( ldr.luxToOutputVoltage( lux_out, true ) );  // LUX
    // Serial.print( lux_out * 204.6 );  // PWM
    Serial.print("\t");

    // led 
    Serial.print( ldr.luxToPWM( getU() , true ) );  // LUX
    // Serial.print( getU() * MAX_ANALOG/MAX_DIGITAL );  // PWM
    Serial.print("\t"); 
  }
  
  return lux_out ;  // simulator value in Volt
}


void ControllerPid::output(){

  t_sum -= t_output[t_counter%MEAN_SIZE]; // remove the last value
  t_output[t_counter%MEAN_SIZE] = ldr.luxToOutputVoltage( ldr.getOutputVoltage(), true ); // read the new value
  t_sum += t_output[t_counter%MEAN_SIZE]; // compute the sum
  t_counter ++; // updates new value
  
  Serial.println( t_sum/MEAN_SIZE ); // LUX
  // Serial.println( ldr.getOutputVoltage() * 204.6 ); // PWM
}
