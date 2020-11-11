#include "controller.h"

#include "TimerOne.h"

// Global variables to be declared in both setup() and loop()

// INIT PID
ControllerPid pid(3, A0); // led and ldr pin

// Global variables

boolean LOOP = true;
boolean SIMULATOR = true;
int i = 0;

void setup() {

  Serial.begin(2000000);
  delay(500);
  pid.ldr.setGain(  pid.getLedPin() );  // define the pin, m and b are predefine. Compute gain
  pid.setReferenceLux( 0 ); // sets the minimum value in the led ( zero instant )
  pid.ldr.t_tau_up.setParametersABC( 29.207246, -0.024485, 11.085226); // values computed in the python file
  pid.ldr.t_tau_down.setParametersABC( 15.402250,  -0.015674, 8.313158); // values computed in the python file
  
  // examples
  /*for(int i=0; i < 20;i++){
    pid.ldr.setGain(3, -0.71 -i*0.001);
    Serial.println("STEP");
  }*/
  
  Serial.println("Set up completed");
  

  pid.setReferenceLux( 10 ); // reference lux value is bounded with the capabilities of the arduino
  initInterrupt1();
  
}


void loop() {
  
  if (LOOP){
  
    if ( Serial.available() ){
      String work_percentage = Serial.readString();  // read input at PWM pin 'led_pin'
      pid.setReferenceLux( work_percentage.toFloat() ); // read input value and ajust the reference brightness
      pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
    }  // anytime there is an input
    
    pid.led.setBrightness( pid.getU() );

    // simulator
    if(SIMULATOR){
      pid.simulator( true );
      pid.output();
    }
  }                                   
}

void initInterrupt1(){
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  // OCR1A = [16, 000, 000Hz / (prescaler * desired interrupt frequency)] - 1
  // OCR1A = [16M / (64 * 100)] - 1; 100Hz
  TCNT1 = 0;   // preload timer
  OCR1A = 2499;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << CS12);    // 64 prescaler 
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11);
 
  TIMSK1 |= (1 << OCIE1A);   // enable timer overflow interrupt
  interrupts();          // enable all interrupts
  
}

ISR(TIMER1_COMPA_vect)        // interrupt service routine 
{ 
  pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
  // Serial.println( millis() );
}
