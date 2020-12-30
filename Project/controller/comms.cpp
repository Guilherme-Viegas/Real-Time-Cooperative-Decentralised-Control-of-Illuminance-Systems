#include "comms.h"

/*------------------------------------|
 * PROGRAM BEGINS                     |
--------------------------------------|*/

//THE READ INTERRUPT ROUTINES
void irqHandler(MCP2515 mcp2515) {
  can_frame frm;
  uint8_t irq = mcp2515.getInterrupts();
  //check messages in buffer 0
  if ( irq & MCP2515::CANINTF_RX0IF ) {
    mcp2515.readMessage( MCP2515::RXB0, & frm );
    if ( !cf_stream.put( frm ) ) //no space
      arduino_overflow = true;
  }

  //check messages in buffer 1
  if ( irq & MCP2515::CANINTF_RX1IF ) {
    mcp2515.readMessage( MCP2515::RXB1, & frm);
    if ( !cf_stream.put( frm ) ) //no space
      arduino_overflow = true;
  }
  irq = mcp2515.getErrorFlags(); //read EFLG
  if ( (irq & MCP2515::EFLG_RX0OVR) |
       (irq & MCP2515::EFLG_RX1OVR) ) {
    mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags();
  }
  mcp2515.clearInterrupts();
  interrupt = true; //notify loop()
}

//THE WRITE ROUTINES

MCP2515::ERROR write(uint32_t id, uint32_t val, MCP2515 mcp2515){
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val; //pack data
  for ( int i = 0; i < 4; i++ ){ //prepare can message
    frame.data[i] = msg.bytes[i];
  }
  //send data
  return mcp2515.sendMessage(&frame);
}
/*---------------------------------------------------------|
 * Message with no values only msg_type and sender_address |
-----------------------------------------------------------|*/
void writeMsg(MCP2515 mcp2515, int id, byte msg_type, byte sender_address, float value = 0){
  if(value){
    //write message with values
    byte* inBytes = (byte*)malloc(2*sizeof(byte));
    
    inBytes = convertFloat2Bytes(value);
    unsigned long val = (unsigned long)msg_type;
    val += (unsigned long)sender_address<<8;
    val += (unsigned long)inBytes[1]<<16;
    val += (unsigned long)inBytes[0]<<24;
    if ( write( id , val, mcp2515 ) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
    free(inBytes);
  }
  else{
    smallMsg(mcp2515, id, msg_type, sender_address);
  }
}

void smallMsg(MCP2515 mcp2515, int id, byte msg_type, byte sender_address) {
  int val = (sender_address<<8)+msg_type;  //Convert 2 bytes into 1 int, with byte 0 being msg_type and byte 1 being sender addr
  if ( write( id , val, mcp2515 ) != MCP2515::ERROR_OK )
      Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
}



can_frame readMsg(){
  can_frame frame;
  if ( interrupt ) {
    interrupt = false;
    if ( mcp2515_overflow ) {
      Serial.println( "\t\t\t\tMCP2516 RX Buf Overflow" );
      mcp2515_overflow = false;
    }

    if( arduino_overflow ) {
      Serial.println( "\t\t\t\tArduino Buffers Overflow" );
      arduino_overflow = false;
    }
    
    bool has_data;
    cli(); has_data = cf_stream.get( frame ); sei();
    if( !has_data ){
      frame.can_id = 100000;
    }
  } else {
    frame.can_id = 100000;
  }
  return frame;
}


/*
 * FUNCTION to get the float value of CAN message
*/
float getValueMsg(can_frame frame){
  byte value[2];    
  value[0] = frame.data[2];
  value[1] = frame.data[3];
  float x = bytes2float(value);
  return x;
}

byte* convertFloat2Bytes(float fnum)
{   
    // the maxium admissive value is 4095.94(9)
    fnum = fnum < pow(2,12)-0.05 ? fnum : pow(2,12)-1+0.9;  // 2^12-1 == 4095, 12 bits representation + 4bits to decimal representation(0.1 -> 0.9)
  
    uint16_t output = fnum < 0; // flag that represents if fnum is negative
    uint16_t inum = fnum;  // integer part
    fnum = fnum-inum; // floating part
    uint8_t dnum = round(10*fnum);  // integer decimal part

    // increments one when the decimal part rounds up
    if(dnum == 10){
        dnum = 0;
        inum++;
    }
    
    inum = inum<<4; // the integer will be represent in the first 1,5 bits
    output = output ? 15: inum + dnum; // send 15 when fnum is negative, which should not be possible
    
    byte* number = (byte*)malloc(2*sizeof(byte));
    number[0] = (byte) output;
    number[1] = (byte) (output>>8);
    return number;
}


float bytes2float(byte * myBytes){
  float welit = 0;
  byte mask_decimal = B00001111;
  byte mask_int = B11110000;
  // get decimal part of the number
  byte decimal = myBytes[1] & mask_decimal;
  int decimal_int = decimal;

  // get decimal part of the number
  byte split_int = (myBytes[1] & mask_int) >> 4;
  int integer = (myBytes[0] << 4) + split_int;

  welit = integer + decimal_int/10.0;
  
  return welit;
}
