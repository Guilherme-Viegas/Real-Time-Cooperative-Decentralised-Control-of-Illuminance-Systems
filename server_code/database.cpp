#include "database.hpp"


office::office( uint8_t num_lamps ) : t_num_lamps(num_lamps)
{   
    if(DEBUG) std::cout << "Welcome to the Office!\n";    // greeting

    t_lamps_array = new lamp* [t_num_lamps];    // creats an array of lamps and return he array of poiters to lamps

    for( int l=0; l<t_num_lamps; l++)    // for each lamp will create one struct 
    {
        t_lamps_array[l] = new lamp { l+1 };;     // stores the address of the new lamp in the array of pointers to lamp object  
    }
}

office::~office()
{   
    if(DEBUG) std::cout << "\nExits the office, see you later aligator!\n"; // goodbye message
    for( int l=0; l<t_num_lamps; l++) { delete t_lamps_array[l]; }    // free the memory of each lamp
    delete[] t_lamps_array;  // free the memory of the array of teh lamps' address
}

/*
*   Writes new values on database
*/
void office::updates_database( char command[], uint8_t size )
{   
    float value = 0.0;

    switch (command[0])
    {
    case 't':
        t_time_since_restart = (double)(command[1]<<12) + bytes_2_float(command[2], command[3]);
        if(DEBUG) std::cout<< "Time since last restart: " << t_time_since_restart << " segundos.\n";
        break;
    
    case 'o':
        t_lamps_array[command[1]-1]->t_state = command[2];   // check if command[3] == '*'

        if(DEBUG) std::cout<< "The state was step to: " << ( t_lamps_array[command[1]-1]->t_state ? "occupied" : "unoccupied") << "\n";
        break;
    
    case 'O':
        value = bytes_2_float(command[2], command[3]);
        if( value < t_lamps_array[command[1]-1]->t_unoccupied_value )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        if( t_lamps_array[command[1]-1]->t_unoccupied_value > 0 ){ std::cout << "[CLIENT]\t>>\tack\n"; }
        // Updates the value in the dataset
        t_lamps_array[command[1]-1]->t_occupied_value = value;
    
        if(DEBUG) std::cout<< "The occupied value is " << t_lamps_array[command[1]-1]->t_occupied_value << "\n";
        break;
    
    case 'U':
        value = bytes_2_float(command[2], command[3]);
        if( value > t_lamps_array[command[1]-1]->t_occupied_value )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        if( t_lamps_array[command[1]-1]->t_unoccupied_value > 0 ){ std::cout << "[CLIENT]\t>>\tack\n"; }
        // Updates the value in the dataset
        t_lamps_array[command[1]-1]->t_unoccupied_value = value;

        if(DEBUG) std::cout<< "The unoccupied value is " << t_lamps_array[command[1]-1]->t_unoccupied_value << "\n";
        break;
    
    default:
        break;
    }

}

/*
*   Converts 2 bytes that contain 12 bit to integer and 4 bit to decimal part into a float number
*/
float office::bytes_2_float(uint8_t most_significative_bit, uint8_t less_significative_bit) const
{
    float decimal_number = less_significative_bit & 0xF;   // gets 4 less significatives bits
    float integer_number = (most_significative_bit << 4) + ((less_significative_bit & 0xF0) >> 4);   // gets 12 most significatives bits

    if(decimal_number == 15 )    // invalid read detected
    {
        if(DEBUG) std::cout << "INVALID NUMBER - number must be positive\t";
        decimal_number = 0;
    }
    
    // std::cout << "integer " << integer_number << " decimal " <<decimal_number << std::endl;
    return integer_number + decimal_number*0.1;
}


/* --------------------------------------------------------------------------------
   |                                  Lamp                                        |
   -------------------------------------------------------------------------------- */

lamp::lamp( int address ) : t_address( (uint8_t)address ) // stores personal address of CAN BUS
{
    if(DEBUG) std::cout << "I am a lamp at the address " << (int)t_address << " ;)\n";    // greeting
}

lamp::~lamp()
{
    if(DEBUG) std::cout << "Ups... seems that the lamp at address " << (int)t_address << " is not available anymore.\n";    // goodbye message
}