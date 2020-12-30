#include "hub.hpp"


/*
 * Head function, where it will distribute the instruction
 */
bool hub()
{   

    int temp = Serial.read();
    
    if(temp != '+'){
      //greeting(temp);
      return;
    }
    
    char welcome[BUFFER_SIZE];
    Serial.readBytes(welcome, BUFFER_SIZE);
    const int arduinos = 1;

    if( welcome[0] == 'R' && welcome[1] == 'P' && welcome[2] == 'i' && welcome[3] == 'G' )
    {   
        // I'm am an arduino HUB
        greeting(arduinos);
    }
    else if( welcome[0] == 'R' && welcome[1] == 'P' && welcome[2] == 'i' && welcome[3] == 'E' ) // last message
    {   
        // I am no longer an arduino HUB
        return false;
    }
    else if( welcome[0] == 'R' && welcome[1] == 'P' && welcome[2] == 'i' && welcome[3] == 'S' )
    {   

        send_time();
        
        bool state[arduinos] = {true};
        for(int a=0; a<arduinos; a++)
        {
          Serial.write("+");
          Serial.write("o");
          Serial.write(a+1);
          Serial.write(state[a]);
          Serial.write('*');
        }

        float lower_bound_occupied[arduinos] = {3.39};
        for(int a=0; a<arduinos; a++)
        {
          Serial.write("+");
          Serial.write("O");
          Serial.write(a+1);
          float_2_bytes(lower_bound_occupied[a]);
        }

        float lower_bound_unoccupied[arduinos] = {1.57};
        for(int a=0; a<arduinos; a++)
        {
          Serial.write("+");
          Serial.write("U");
          Serial.write(a+1);
          float_2_bytes(lower_bound_unoccupied[a]);
        }

        float costs[arduinos] = {1.0};
        for(int a=0; a<arduinos; a++)
        {
          Serial.write("+");
          Serial.write("c");
          Serial.write(a+1);
          float_2_bytes(costs[a]);
        }

        return true;
        
    }
    else
    {
        // MISS
        // Serial.write("x");
        // Serial.write(7); // number of arduinos
        // Serial.write(":(");
        Serial.write("+");
        Serial.write(welcome[0]);
        Serial.write(welcome[1]);
        Serial.write(welcome[2]);
        Serial.write(welcome[3]);
    }
    
    return false;
}

/*
 * This function is used to represent a float number in 2 bytes.
 * 
 * The 12 most significatives bits represents the integer part with resolution 0:4095
 * The 4  less significatives bits represents the decimal part with resolution 0:9
 * 
 * Sends '*' when there is nothing to be said, which corresnponds to 10 in the 4 less significative bits
 */
void float_2_bytes(float fnum)
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
    
    Serial.write("+");
    Serial.write("A");
    Serial.write(numLamps); // number of arduinos
    Serial.write(":)");
    
}

// the time spent occupates 2.5 bytes with integer number and 4bit with float
void send_time()
{   
    double time_ = millis() * 1e-3;
    Serial.write("+");
    Serial.write('t');
    Serial.write( ( (unsigned long)round(time_) & 0xFF000)  >> 12 );
    float_2_bytes( time_ - ((unsigned long)round(time_) & 0xFF000) );
}
