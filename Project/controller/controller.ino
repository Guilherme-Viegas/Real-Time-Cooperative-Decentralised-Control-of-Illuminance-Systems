#include "controller.h"

// Global variables to be declared in both setup() and loop()

// INIT PID
ControllerPid pid(3, A0);

// Global variables

boolean LOOP = true;
boolean SIMULATOR = true;

void setup() {
  float tmp_m;
  float tmp_b;
  int eeprom_addr = 0;
  EEPROM.get(eeprom_addr, tmp_m);
  eeprom_addr += sizeof(float);
  EEPROM.get(eeprom_addr, tmp_b);
  
  Serial.begin(2000000);
  delay(500);
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file
  pid.ldr.setGain( pid.getLedPin(), tmp_m, tmp_b );
  
  //pid.led.setBrightness(100);
  //pidB.led.setBrightness(0);
  //pidC.led.setBrightness(0);

  delay(500);

  /*
  Serial.println("Arduino A");
  Serial.println(pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true));
  Serial.println(pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true));
  Serial.println(pid.ldr.luxToOutputVoltage(pid.ldr.getOutputVoltage(), true));

  Serial.println("Arduino B");
  Serial.println(pidB.ldr.luxToOutputVoltage(pidB.ldr.getOutputVoltage(), true));
  Serial.println(pidB.ldr.luxToOutputVoltage(pidB.ldr.getOutputVoltage(), true));
  Serial.println(pidB.ldr.luxToOutputVoltage(pidB.ldr.getOutputVoltage(), true));

  Serial.println("Arduino C");
  Serial.println(pidC.ldr.luxToOutputVoltage(pidC.ldr.getOutputVoltage(), true));
  Serial.println(pidC.ldr.luxToOutputVoltage(pidC.ldr.getOutputVoltage(), true));
  Serial.println(pidC.ldr.luxToOutputVoltage(pidC.ldr.getOutputVoltage(), true));
  */
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
