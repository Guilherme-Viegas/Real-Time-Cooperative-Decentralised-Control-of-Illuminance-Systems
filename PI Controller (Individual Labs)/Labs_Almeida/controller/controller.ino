#include "controller.h"

// Global variables to be declared in both setup() and loop()

// INIT PID
ControllerPid pid(3, A0); // led and ldr pin

// Global variables

boolean LOOP = true;
boolean SIMULATOR = true;

void setup() {

  Serial.begin(2000000);
  delay(500);
  pid.ldr.setGain( pid.getLedPin(), -0.75 );  // define the pin, m and b are predefine. Compute gain
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file
  
  // examples
  /*for(int i=0; i < 20;i++){
    pid.ldr.setGain(3, -0.71 -i*0.001);
    Serial.println("STEP");
  }*/
  
  Serial.println("Set up completed");
  
  
  pid.setReferenceLux( 30 ); // sets the minimum value in the led ( zero instant )
  if(pid.has_feedback()){initInterrupt1();}
  
}


void loop() {
  
  if (LOOP){
  
    if ( Serial.available() ){
      String work_percentage = Serial.readString();  // read input at PWM pin 'led_pin'
      pid.setReferenceLux( work_percentage.toFloat() ); // read input value and ajust the reference brightness
    }  // anytime there is an input

    pid.led.setBrightness( pid.getU() );

    // simulator
    if(SIMULATOR){
      pid.simulator( true );
      pid.output();
    }
  }                                   
}

// interrupt service routine 
ISR(TIMER1_COMPA_vect)        
{ 
  pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
  // Serial.println( millis() );
}
