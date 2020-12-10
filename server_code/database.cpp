#include "database.hpp"


office::office( uint8_t numLamps )
{   
    std::cout << "Welcome to the Office!\n";    // greeting

    t_numLamps = numLamps;  // sets the number of desks

    auto temp = new lamp* [t_numLamps]; // creats an array of lamps
    t_lampsArray = temp;    // retrives the array of poiters to lamps

    for( int l=0; l<t_numLamps; l++)    // for each lamp will create one struct 
    {
        auto temp = new lamp { l+1 };  // creates a lamp object, and return its address
        t_lampsArray[l] = temp;     // stores the address of the new lamp in the array of pointers to lamp object  
    }
}

office::~office()
{   
    for( int l=0; l<t_numLamps; l++) { delete[] *(t_lampsArray+l); }    // free the memory of each lamp
    delete[] t_lampsArray;  // free the memory of the array of teh lamps' address
    std::cout << "I hope you had enjoy, see you later aligator!\n"; // goodbye message
}


/* --------------------------------------------------------------------------------
   |                                  Lamp                                        |
   -------------------------------------------------------------------------------- */

lamp::lamp( int address )
{
    std::cout << "I am a lamp at the address " << (int)t_address << " ;)\n";    // greeting
    t_address = (uint8_t)address;    // stores personal address of CAN BUS
}

/*lamp::~lamp()
{
    // std::cout << "Ups... seems that the lamp at " << t_address << " is not available anymore.\n";    // goodbye message
}*/