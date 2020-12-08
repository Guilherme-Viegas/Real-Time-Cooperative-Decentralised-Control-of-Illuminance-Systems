#define BUFFER_SIZE 4
char welcome[BUFFER_SIZE];
#include "hub.hpp"

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 57600 baud
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.flush();

  //float2bytes(4095.97);
}

void loop() {

  if(Serial.available()){ hub(); }

    
}

// screen /dev/tty.usbmodem14601 baud_rate
