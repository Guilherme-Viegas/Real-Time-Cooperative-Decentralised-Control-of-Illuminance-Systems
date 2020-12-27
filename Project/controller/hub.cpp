#include "hub.hpp"

#define BUFFER_SIZE 4
char welcome[BUFFER_SIZE];

/*
 * Head function, where it will distribute the instruction
 */
void hub(int num_addr)
{
    Serial.readBytes(welcome, BUFFER_SIZE);
    
    if( welcome[0] == 'R' && welcome[1] == 'P' && welcome[2] == 'i' )
      {
          greeting(num_addr);
      }
    else if( welcome[0] == 'g' && welcome[1] == 'l' && welcome[2] == 'i' )
    {
        // Greeting
        float2bytes( 78.551 );
        float2bytes( 13.855 );
    }
    else
    {
        // MISS
        Serial.write("x");
        Serial.write(7); // number of arduinos
        Serial.write(":(");
    }
}

/*
 * This function is used to represent a float number in 2 bytes.
 * 
 * The 12 most significatives bits represents the integer part with resolution 0:4095
 * Thr 4  less significatives bits represents the decimal part with resolution 0:9
 */
void float2bytes(float fnum)
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
    
    Serial.write( (byte) (output>>8) ); // wirte second byte - DEBUG: Serial.println((byte) (output>>8))
    Serial.write( (byte) output );  // write first byte - DEBUG: Serial.println((byte) output);
}

/*
 * Sends the first message to the server with the format: "A<_number_of_desks_>:)"
 */
void greeting(int numLamps)
{
    // Greeting
    numLamps = numLamps < 0 ? 0 : numLamps > 255 ? 255 : numLamps;  // normalize number of desks.
    Serial.write("A");
    Serial.write(numLamps); // number of arduinos
    Serial.write(":)");
}
