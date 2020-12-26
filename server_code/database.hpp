#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "circularbuffer.hpp"

#define N_POINTS_MINUTE 6000

/*
 * Represents the lamp-desk
 */
class lamp
{

private:    // this things are private
    uint8_t t_address = 0;
    
public:     // this things are public

    // variables
    bool t_state = false;   // false - the desk in unoccupied, true - the desk is occupied
    float t_occupied_value = -1.0;
    float t_unoccupied_value = -2.0;
    circular_array<float> t_lumminace{ N_POINTS_MINUTE };
    circular_array<float> t_duty_cile{ N_POINTS_MINUTE };

    // functions
    lamp( int address );
    ~lamp(); // https://stackoverflow.com/questions/7850374/stuck-in-infinite-loop-in-deallocating-memory
};

/*
 * Represents the database
 */
class office
{

private:    // this things are private
    lamp** t_lamps_array;
    double t_time_since_restart = 0.0;
    bool t_office_is_open = false; // TODO
    int t_num_lamps = -1; // TODO

    // functions
    float bytes_2_float(uint8_t most_significative_bit, uint8_t less_significative_bit) const;

public:     // this things are public

    office( uint8_t numLamps );
    ~office();
    
    bool get_state(){ return t_office_is_open; }
    void set_state( bool state ){ t_office_is_open = state; }
    void updates_database( char command[], uint8_t size );

};




#endif

