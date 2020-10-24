#include "training_data.h"

float m = -0.672;

void setup() {

  Serial.begin(9600);

  pinMode(LED_PWM, OUTPUT);
  delay(1e3);
  
  
  /*for(int i=0; i<20; i++){
    test_cont(-0.66-i*0.001);
    Serial.println("STEP");

    analogWrite(LED_PWM, 0);
    delay(1000);
  }*/

 
  // test_cont(m);
  test_steps_down(m);

  analogWrite(LED_PWM, 0);
  Serial.println("End");
}


void loop(){}
