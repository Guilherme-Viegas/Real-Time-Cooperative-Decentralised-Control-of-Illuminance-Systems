#include "message.h"
can_frame broadcast_message(){
  can_frame msg;
  //id of a broadcast message
  msg.can_id = 0x000;
  msg.can_dlc = 4;
  msg.data[0] = 0x4F; //O
  msg.data[1] = 0x4C; //L
  msg.data[2] = 0x41; //A
  msg.data[3] = 0000;
  msg.data[4] = 0000;
  msg.data[5] = 0000;
  msg.data[6] = 0000;
  msg.data[7] = 0000;
  return msg;
}

void code2Ascii(can_frame msg){
  char* string = (char*)malloc(3*sizeof(char));
  string[0] = msg.data[0];
  string[1] = msg.data[1];
  string[2] = msg.data[2];
  Serial.println("Message: " + String(string));
  free(string);
}
