#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <iostream>
#include <string>
#include <algorithm>

#include <boost/asio.hpp>
#include "circularbuffer.hpp"

#define N_POINTS_MINUTE 6000
#define SAMPLE_TIME_MILIS 10

/*
 * Represents the lamp-desk
 */
class lamp
{

private: // this things are private
    uint8_t t_address = 0;

    // performence metrics
    float t_accumulated_energy_consumption = 0.0;
    float t_instant_power = 0.0;
    float t_accumulated_visibility_error = 0.0;
    float t_accumulated_flicker_error = 0.0;

    float t_luminance_prev_1 = 0.0;
    float t_luminance_prev_2 = 0.0;
    float t_duty_cicle_prev = 0.0;
    unsigned short t_n_samples = 0;
    bool t_state = false; // false - the desk in unoccupied, true - the desk is occupied
    float t_occupied_value = -1.0;
    float t_unoccupied_value = -2.0;
    float t_nominal_power = -1.0;

    std::mutex t_mutex;

public: // this things are public
    // variables

    circular_array<float> t_luminance{N_POINTS_MINUTE};
    circular_array<float> t_duty_cicle{N_POINTS_MINUTE};

    // functions
    lamp(int address);
    ~lamp(); // https://stackoverflow.com/questions/7850374/stuck-in-infinite-loop-in-deallocating-memory
    float get_accumulated_energy_consumption_at_desk();
    float get_instant_power_at_desk();
    float get_accumulated_visibility_error_at_desk();
    float get_accumulated_flicker_error_at_desk();
    void compute_performance_metrics_at_desk(float new_luminance = 0.0, float new_duty_cicle = 0.0);
    void set_state(bool state);
    bool get_state();
    void set_occupied_value(float value);
    float get_occupied_value();
    void set_unoccupied_value(float value);
    float get_unoccupied_value();
    void set_nominal_power(float value);
    float get_nominal_power();
};

/*
 * Represents the database
 */
class office
{

private: // this things are private
    float t_time_since_last_restart = 0.0;
    int t_num_lamps = -1; // TODO comando para dar update se houver um restart

    // streams
    boost::asio::ip::udp::socket *t_socket;

    std::vector<boost::asio::ip::udp::endpoint> t_udp_endpoints{};
    std::vector<char> t_udp_stream_type{};
    std::vector<int> t_udp_stream_address{};

    std::mutex t_mutex;

    // functions
    float bytes_2_float(uint8_t most_significative_bit, uint8_t less_significative_bit) const;
    void restart_it_all(int lamps);
    void udp_stream( int address );

public: // this things are public
    // it is access by the async_server
    lamp **t_lamps_array;

    // async operation
    std::vector<std::string> t_clients_address{};
    std::vector<std::string> t_clients_command{};
    std::vector<int> t_acknowledge{};

    office(uint8_t numLamps);
    ~office();

    double get_elapesd_time_since_last_restart() { return t_time_since_last_restart; }
    void updates_database(char command[], uint8_t size);
    void float_2_bytes(float fnum, u_int8_t bytes[2]) const;

    float get_accumulated_energy_consumption();
    float get_instant_power();
    float get_accumulated_visibility_error();
    float get_accumulated_flicker_error();
    int set_upd_stream(char type, int address, boost::asio::ip::udp::socket *socket, boost::asio::ip::udp::endpoint endpoint);
    int get_num_lamps();
};

#endif
