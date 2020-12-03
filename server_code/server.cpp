// Ports
#define RPI_PORT "/dev/ttyACM0"
#define MAC_PORT "/dev/tty.usbmodem14601" // ls /dev/tty.*
#define BAUD_RATE 9600//230400

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
boost::asio::streambuf read_buf; //read buffer


std::string buffer2String(boost::asio::streambuf &buf)
{
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);
    std::cout << "this is our string " << line << std::endl;
    return line;
}


int counter = 0;
//forward declaration of write_handler to timer_handler
void write_handler(const error_code &ec, size_t nbytes);
void timer_handler(const error_code &ec)
{
    //timer expired – launch new write operation
    std::ostringstream os;
    os << "Counter = " << ++counter << std::endl;
    async_write(sp, buffer(os.str()), write_handler);
}

void write_handler(const error_code &ec, size_t nbytes)
{
    tim.expires_after(boost::asio::chrono::seconds{2});
    tim.async_wait(timer_handler);
}
void read_handler(const error_code &ec, size_t nbytes)
{
    // std::cout << &read_buf;
    // buffer2String(read_buf);
    ascii2Binary( buffer2String( read_buf )[0] );
    //ascii2Binary(read_buf[0]);
    async_read_until(sp, read_buf, "\n", read_handler);
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
    async_read_until(sp, read_buf, "\n", read_handler);
    io.run(); //get things rolling
    std::cout << "Last line of the code - serve\n";
}