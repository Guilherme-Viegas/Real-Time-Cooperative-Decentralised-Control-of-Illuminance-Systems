/***
    writes on eeprom the following data:
    float m;
    float b;
    ...(Depois temos de escrever os taus tmb)
***/
#include <EEPROM.h>

byte addr = 3;
float m = -0.668;
float b = 4.65353;


void setup() {
  float f;
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  int eeAddress = 0;   //Location we want the data to be put.

  //One simple call, with the address first and the object second.
  EEPROM.put(eeAddress, addr);
  eeAddress += sizeof(byte);
  EEPROM.put(eeAddress, m);
  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  EEPROM.put(eeAddress, b);
  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  

}

void loop() {
  /* Empty loop */
}
