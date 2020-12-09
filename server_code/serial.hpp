#ifndef SERIAL_HPP
#define SERIAL_HPP

// /*
// This file supports the communications throughout the server
// */

#include <iostream>
#include <string>
#include <boost/asio.hpp>

// Ports
#define RPI_PORT "/dev/ttyACM0" // dmesg
#define MAC_PORT "/dev/tty.usbmodem14601" // ls /dev/tty.*
#define BAUD_RATE 9600//230400
#define ARDUINO_MESSAGE "Arduino"
#define BUFFER_SIZE 4

/*
 * Controls the Serial comunication
 */
class communications
{

private:    // this things are private

    boost::asio::serial_port* t_serial ;

    boost::asio::streambuf t_buf4{BUFFER_SIZE};
    boost::system::error_code t_ec;

    uint8_t t_numLamps = 0;

    // functions
    float bytes2float(uint16_t mostSignificativeBit, uint8_t lessSignificativeBit);

    
public:     // this things are public

    communications( boost::asio::serial_port *s );// constructor
    ~communications();  // destructor

    uint8_t hasHub();
    void write_command();
    //void write_handler();
    //void flush();

};

#endif

// boost::asio::serial_port sp{io};
// boost::asio::steady_timer tim{io};
// boost::asio::streambuf buf_1{1}; //read buffer
// boost::asio::streambuf buf_trash; //read buffer
