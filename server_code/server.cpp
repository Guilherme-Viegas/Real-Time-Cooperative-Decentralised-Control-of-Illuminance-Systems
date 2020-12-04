// Ports
#define RPI_PORT "/dev/ttyACM0"
#define MAC_PORT "/dev/tty.usbmodem14601" // ls /dev/tty.*
#define BAUD_RATE 9600//230400
#define ARDUINO_MESSAGE "Arduino"

#include "util.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
using namespace boost::system;
using namespace boost::asio;
boost::asio::io_context io;
boost::asio::serial_port sp{io};
boost::asio::steady_timer tim{io};
boost::asio::streambuf buf_1{1}; //read buffer
boost::asio::streambuf buf_3{3}; //read buffer
boost::asio::streambuf buf_trash; //read buffer

std::string buffer2String(boost::asio::streambuf &buf)
{
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);
    //std::cout << "this is our string " << line << std::endl;
    return line;
}


int counter = 0;
//forward declaration of write_handler to timer_handler
void write_handler(const error_code &ec, size_t nbytes);
void timer_handler(const error_code &ec)
{
    //timer expired – launch new write operation
    async_write(sp, buffer("RPi"), write_handler);
    std::cout << "Hello RPi\n";
}

void write_handler(const error_code &ec, size_t nbytes)
{
    //tim.expires_after(boost::asio::chrono::seconds{2});
    std::cout << "IS there any connection?\n";
    //tim.async_wait(timer_handler);
}
void read_handler(const error_code &ec, size_t nbytes)
{   
    std::string str_buf = buffer2String( buf_trash );  // string read from the buffer
    if( str_buf.compare(ARDUINO_MESSAGE) == 0 ) // new connection
    {
        std::cout << "New Arduino is connected!\n";
    }

    //ascii2Binary( str_buf[0] );
    //ascii2Binary(read_buf[0]);
    async_read(sp, buf_1, read_handler);
    std::cout << "Hello world!\t" << &buf_1 << std::endl;
}


int main()
{
    boost::system::error_code ec;
    sp.open( MAC_PORT, ec); //connect to port
    if (ec)
        std::cout << "Could not open serial port \n";
    sp.set_option(serial_port_base::baud_rate{BAUD_RATE}, ec);
    //program timer for write operations
    tim.expires_after(boost::asio::chrono::seconds{2});
    tim.async_wait(timer_handler);
    //program chain of read operations
    async_read_until(sp, buf_trash, '\n', read_handler);
    io.run(); //get things rolling
    std::cout << "Last line of the code - serve\n";
}