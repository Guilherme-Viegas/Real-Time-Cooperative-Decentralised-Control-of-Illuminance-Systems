#ifndef MESSAGE_H
#define MESSAGE

#include <SPI.h>
#include <mcp2515.h>
can_frame broadcast_message();
void code2Ascii(can_frame msg);



#endif
