#include "ldr_fit.h"

boolean LOOP = true;
boolean SIMULATOR = true;
int i = 0;
int m = -0.718;
void setup() {

  Serial.begin(2000000);
  delay(500);
  
  steps_up(m);
  steps_down(m);
  
}


void loop() {}
