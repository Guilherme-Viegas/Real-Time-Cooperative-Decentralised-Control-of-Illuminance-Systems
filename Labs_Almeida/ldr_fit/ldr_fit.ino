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

  Serial.begin(9600);
  delay(500);
  pid.ldr.setGain(  pid.getLedPin() );  // define the pin, m and b are predefine. Compute gain
  pid.led.setFinalValuePwm( pid.ldr.boundLUX( 0 ) ); // sets the minimum value in the led 
  pid.ldr.t_tau_up.setParametersABC( 25.016859, -0.054231, 4.355248); // values compute in the python file
  pid.ldr.t_tau_down.setParametersABC( 17.962167,  -0.028300, 2.750221); // values compute in the python file
  pid.setReferenceLux( 21 ); // reference lux value is bounded with the capabilities of the arduino

  initInterrupt1();
  
  // examples
  /*for(int i=0; i < 20;i++){
    ldr1.setParametersPinMB(A0, -0.6 -i*0.01);
    Serial.println("STEP");
  }
  
  Serial.println( ldr1.voltageToLux(3.5) ); 
  Serial.println( ldr1.luxToPWM(185) ); // expected appr 100
  Serial.println( ldr1.luxToPWM(100, true) ); // expected 185

  Serial.println( ldr1.t_tau_up.fTau(256) );
  Serial.println( 177) );
  Serial.println( ldr1.t_tau_down.fTau(92) );
  Serial.println( ldr1.t_tau_down.fTau(-1) );
  */
  // steps_up(-0.66);
  // steps_down(-0.66);
  
  analogWrite(3, 0);
  Serial.println("Set up completed");
  
}


void loop() {
  
  if (LOOP){
  
    if ( Serial.available() ){
      String work_percentage = Serial.readString();  // read input at PWM pin 'led_pin'
      pid.setReferenceLux( work_percentage.toFloat() ); // read input value and ajust the reference brightness
    }  // anytime there is an input
    
    pid.led.setBrightness( pid.getU(), false );

    // simulator
    if(SIMULATOR)
      pid.simulator( false );
      pid.output();
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
  noInterrupts();  
  pid.computeFeedbackGain( analogRead( pid.getLdrPin() ) );
  // Serial.println( millis() );
  interrupts();
}
