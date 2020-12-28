#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <iostream>
#include <string>
#include <algorithm> 

#include <boost/asio.hpp>
#include "circularbuffer.hpp"

#define N_POINTS_MINUTE 600
#define SAMPLE_TIME_MILIS 10

/*
 * Represents the lamp-desk
 */
class lamp
{

private:    // this things are private
    uint8_t t_address = 0;

    // performence metrics
    float t_accumulated_energy_consumption = 0.0;
    float t_instant_power = 0.0;
    float t_nominal_power = 1.0;
    float t_accumulated_visibility_error = 0.0;
    float t_accumulated_flicker_error = 0.0;

    float t_luminance_prev_1 = 0.0;
    float t_luminance_prev_2 = 0.0;
    float t_duty_cicle_prev = 0.0;
    unsigned short t_n_samples = 0;

    
public:     // this things are public

    // variables
    bool t_state = false;   // false - the desk in unoccupied, true - the desk is occupied
    float t_occupied_value = -1.0;
    float t_unoccupied_value = -2.0;
    float t_cost = -1.0;
    circular_array<float> t_lumminace{ N_POINTS_MINUTE };
    circular_array<float> t_duty_cicle{ N_POINTS_MINUTE };


    // functions
    lamp( int address );
    ~lamp(); // https://stackoverflow.com/questions/7850374/stuck-in-infinite-loop-in-deallocating-memory
    float get_accumulated_energy_consumption_at_desk(){ return t_accumulated_energy_consumption; }
    float get_instant_power_at_desk(){ return t_instant_power; }
    float get_accumulated_visibility_error_at_desk(){ return t_accumulated_visibility_error; }
    float get_accumulated_flicker_error_at_desk(){ return t_accumulated_flicker_error; }
    void compute_performance_metrics_at_desk( float new_luminance = 0.0, float new_duty_cicle = 0.0 );
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
    
    double get_elapesd_time_since_last_restart(){ return t_time_since_restart; }
    bool get_state(){ return t_office_is_open; }
    void set_state( bool state ){ t_office_is_open = state; }
    void updates_database( char command[], uint8_t size );

    float get_accumulated_energy_consumption();
    float get_instant_power();
    float get_accumulated_visibility_error();
    float get_accumulated_flicker_error();

};

#endif

