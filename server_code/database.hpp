#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <iostream>
#include <string>
#include <boost/asio.hpp>

/*
 * Represents the lamp-desk
 */
class lamp
{

private:    // this things are private
    uint8_t t_address = 0; 

public:     // this things are public

    lamp( int address );
    // ~lamp(){}; // https://stackoverflow.com/questions/7850374/stuck-in-infinite-loop-in-deallocating-memory
};

/*
 * Represents the database
 */
class office
{

private:    // this things are private
    lamp** t_lampsArray;
    // unsigned long t_timeSinceRestart = 0UL;
    bool t_officeIsOpen = false;
    int t_numLamps = -1;

public:     // this things are public

    office( uint8_t numLamps );
    ~office();
    
    bool getState(){ return t_officeIsOpen; }
    void setState( bool state ){ t_officeIsOpen = state; }

};




#endif