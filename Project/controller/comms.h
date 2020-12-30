#ifndef COMMS_H
#define COMMS

#include "can_buffer.cpp"
#include "Arduino.h"

enum msg_types {hello=0, olleh=1, Ready=2, read_offset_value=3, read_lux_value = 4, turnOn_led=5, turnOff_led=6, your_time_master=7, ACK=8, start_consensus = 9};

//flags
volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
//circular buffer that saves the last 10 messages
volatile can_frame_stream cf_stream;

//union to pack/unpack long ints into bytes
union my_can_msg {
  unsigned long value;
  unsigned char bytes[4];
};

can_frame readMsg();
void writeMsg(MCP2515 mcp2515, int id, byte msg_type, byte sender_address, float value = 0);
MCP2515::ERROR write(uint32_t id, uint32_t val, MCP2515 mcp2515);
void irqHandler(MCP2515 mcp2515);
float getValueMsg(can_frame frame);
void smallMsg(MCP2515 mcp2515, int id, byte msg_type, byte sender_address);


byte* convertFloat2Bytes(float fnum);
float bytes2float(byte * myBytes);


#endif
