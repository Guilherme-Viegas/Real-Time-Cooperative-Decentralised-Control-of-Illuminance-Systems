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
  Serial.println("You created a new PID controller object!");
  t_kp = 0;
  t_ki = 0;
  t_kd = 0;

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

  output = ldr.voltageToLux( output * VCC/MAX_ANALOG ); // output Lux

  float error = t_reference - output;  // determines the error between the system output and the reference value (Lux)
  // error = boundPWM( ldr.luxToPWM( error ) ); // convert error to PWM

  if( t_integral_reset ){ // resets cumalative error
    t_uint = 0;
    t_integral_reset = false;
  }else{  // compute integral
    t_uint += error * t_sampleTime;  
  }
  t_uder = (error - t_lastError) / t_sampleTime; // compute derivative

  t_lastError = error;  // remember current error
  
  t_ufb = t_kp*error + t_ki*t_uint + t_kd*t_uder;  // return the feedback signal
}

/*
 * Sets Reference lux level
 *
 * @param reference desire illumination
 */
void ControllerPid::setReferenceLux( float reference ){

  t_reference = ldr.boundLUX( reference ); // Normalize reference regarding external possibilities [Lux]
  led.setBrightness( boundPWM( ldr.luxToPWM( reference ) ), true ); // first instant
  t_integral_reset = true; // reset integral error
  setUff( boundPWM( ldr.luxToPWM( reference ) ) ); // sets feedfoward signal
}

/*
 * Reads Serial and convert to PWM
 */
void ControllerPid::setUff( byte uff ){
    t_uff = uff; // feedfoward impulse
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
byte ControllerPid::getU(){
    float u = t_ufb + t_uff;
    return boundPWM(u);
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
 * @return Ed Pin
 */
byte ControllerPid::getLedPin(){ return t_ledPin; }


void ControllerPid::simulator( boolean end ){

  float luxi = led.getInitialValuePwm();
  float luxf = led.getFinalValuePwm();

  float tau = luxf > luxi ? ldr.t_tau_up.fTau( luxf ) : ldr.t_tau_down.fTau( luxf );

  luxi = ldr.luxToPWM( luxi, true);
  luxf = ldr.luxToPWM( luxf, true);

  float expoente = ( millis() - t_to ) / tau ;

  float lux_out = luxf - (luxf - luxi )*exp( -expoente );

  Serial.print( lux_out );
  Serial.print("\t");
  Serial.print( ldr.boundLUX( lux_out ) );
  Serial.print("\t");
  Serial.print( t_reference );

  if(end){ Serial.println(""); }
  else{ Serial.print("\t"); }
  
}




void ControllerPid::output(){
  Serial.println( ldr.voltageToLux( ldr.getVoltage() ) );
}
