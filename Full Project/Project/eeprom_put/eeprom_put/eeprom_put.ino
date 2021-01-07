/***
    writes on eeprom the following data:
    float m;
    float b;
    tau's up
    tau's down
***/
#include <EEPROM.h>

byte addr = 2;
float m = -0.576;
float b = 4.1577;

float tau_a_up = 28.81354;
float tau_b_up = -0.020658;
float tau_c_up = 9.5327;

float tau_a_down = 16.37341;
float tau_b_down = -0.015004;
float tau_c_down = 7.26781;

void setup() {
  float f;
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  int eeAddress = 0;   //Location we want the data to be put.

  //One simple call, with the address first and the object second.
  //EEPROM.put(eeAddress, addr);
  eeAddress += sizeof(byte);
  //EEPROM.put(eeAddress, m);
  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  //EEPROM.put(eeAddress, b);
  eeAddress += sizeof(float); //Move address to the next byte after float 'f'.
  EEPROM.put(eeAddress, tau_a_up);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, tau_b_up);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, tau_c_up);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, tau_a_down);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, tau_b_down);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, tau_c_down);

  Serial.println("EEPROM WRITING FINISHED!");
}

void loop() {
  /* Empty loop */
}
