#include "hub.hpp"
#include "controller.h"

// Global variables to be declared in both setup() and loop()

// INIT PID
ControllerPid pid(3, A0); // led and ldr pin

// Global variables

boolean LOOP = true;
boolean SIMULATOR = false;


bool transmiting = false;

double counter = 0;

void setup() {
  Serial.begin(230400);
  delay(500);
  pid.ldr.setGain( pid.getLedPin(), -0.75 );  // define the pin, m and b are predefine. Compute gain
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file

  //Serial.println("Set up completed");
  
  
  pid.setReferenceLux( 3 ); // sets the minimum value in the led ( zero instant )
  if(pid.has_feedback()){initInterrupt1();}
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  initInterrupt1();

}

void loop() {

  if( Serial.available() ){ transmiting = hub(); }

  if (LOOP){

    // if ( Serial.available() ){
    //  String work_percentage = Serial.readString();  // read input at PWM pin 'led_pin'
    //  pid.setReferenceLux( work_percentage.toFloat() ); // read input value and ajust the reference brightness
    // }  // anytime there is an input
  
    pid.led.setBrightness( pid.getU() );
  
    // simulator
    if(SIMULATOR){
      pid.simulator( true );
      pid.output();
    }
  }      
    
}

// screen /dev/tty.usbmodem14601 baud_rate

// interrupt service routine 
ISR(TIMER1_COMPA_vect)        
{ 

  if(transmiting)
  { 
    counter++;
    // pwm
    Serial.write("+");
    Serial.write("s");
    Serial.write(1);    // address
    float_2_bytes( pid.ldr.luxToOutputVoltage( 5.0*analogRead( pid.getLdrPin() ) / 1023.0, true) ); // luminace
    float_2_bytes( 100.0*pid.getU()/255.0  ); // duty cicle
    //float_2_bytes(counter);
    //float_2_bytes(counter);
  }

   pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );

}
