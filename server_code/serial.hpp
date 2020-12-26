#ifndef SERIAL_HPP
#define SERIAL_HPP

// /*
// This file supports the communications throughout the server
// */

#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "database.hpp"

// Ports
#define RPI_PORT "/dev/ttyACM0" // dmesg
#define MAC_PORT "/dev/tty.usbmodem143101" // ls /dev/tty.*
#define BAUD_RATE 230400//230400
#define ARDUINO_MESSAGE "Arduino"
#define BUFFER_SIZE 4


/*
 * Controls the Serial comunication
 */
class communications
{

private:    // this things are private

    // boost::asio::serial_port *t_serial = nullptr;
    std::unique_ptr<boost::asio::serial_port> t_serial;

    boost::system::error_code t_ec;
    boost::asio::streambuf t_buf4{BUFFER_SIZE};
    uint8_t t_num_lamps = 0;
    bool t_coms_available = true;

    // functions

    
public:     // this things are public


    communications( boost::asio::io_context *io );// constructor
    ~communications();  // destructor

    uint8_t has_hub();
    void write_command();
    void read_async_command( office *the_office );
    void set_coms_not_available(){ t_coms_available = false; }

};

#endif

// boost::asio::serial_port sp{io};
// boost::asio::steady_timer tim{io};
// boost::asio::streambuf buf_1{1}; //read buffer
// boost::asio::streambuf buf_trash; //read buffer
