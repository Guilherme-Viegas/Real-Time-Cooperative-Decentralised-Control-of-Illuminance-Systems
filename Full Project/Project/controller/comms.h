#ifndef COMMS_H
#define COMMS

#include <mcp2515.h>

byte* convertFloat2Bytes(float fnum);
float bytes2float(byte * myBytes);


#endif
