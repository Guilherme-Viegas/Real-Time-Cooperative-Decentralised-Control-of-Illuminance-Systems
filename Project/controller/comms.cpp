#include "comms.h"

byte* float2bytes(float fnum)
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
