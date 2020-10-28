#include "ldr_controller.h"
#include "led.h"

// Global variables to be declared in both setup() and loop()

// INIT LED
Led led1(3);

// INIT LDR
LdrController ldr1; 

// Global variables
long int time = millis();

void setup() {

  Serial.begin(9600);
  delay(500);

  ldr1.setParametersPinMB(A0);  // define the pin, m and b are predefine. Compute gain
  ldr1.t_tau_up.setParametersABC( 20.415883, -0.060657, 3.221372); // values compute in the python file
  ldr1.t_tau_down.setParametersABC( 18.070167,  -0.038226, 2.598238); // values compute in the python file
  
  /*// examples
  Serial.println( ldr1.voltageToLux(3.5) ); 
  Serial.println( ldr1.luxToPWM(185) ); // expected appr 100
  Serial.println( ldr1.luxToPWM(100, true) ); // expected 185

  Serial.println( ldr1.t_tau_up.fTau(256) );
  Serial.println( amote177) );
  Serial.println( ldr1.t_tau_down.fTau(92) );
  Serial.println( ldr1.t_tau_down.fTau(-1) );
  // steps_up(m);
  // steps_down(m);
  */
  
  analogWrite(LED_PWM, 0);
  Serial.println("End of the Program");
  Serial.flush();
  
}


void loop() {
  
  if ( Serial.available() ){ // anytime there is an input
    led1.setBrightness( get_pwm_serial() );  // read input value and ajust the led brightness
    time = millis();
  }

  ldr1.printBrightness(led1.getInitialValuePwm(), led1.getFinalValuePwm(), time,
                       led1.getInitialValuePwm() > led1.getFinalValuePwm()  ? ldr1.t_tau_up.fTau( led1.getFinalValuePwm() ) : ldr1.t_tau_down.fTau( led1.getFinalValuePwm() ) );
                       
}
