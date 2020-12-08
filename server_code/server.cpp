#include "serial.hpp"

int main()
{

    boost::asio::io_context io;
    boost::asio::serial_port s{io};
    
    communications server( &s );

    std::cout << "It was found " << (int)server.hasHub() << " desk"<< ((server.hasHub() > 1) ? "s" : "") << "!\n";

    server.write_command();
    
    io.run();

    return 0;
}

/*
    boost::system::error_code ec;
    sp.open( MAC_PORT, ec); //connect to port
    if (ec)
        std::cout << "Could not open serial port \n";
    sp.set_option(boost::asio::serial_port_base::baud_rate{BAUD_RATE}, ec);

    //program timer for write operations
    tim.expires_after(boost::asio::chrono::seconds{4});
    tim.async_wait(timer_handler);
    //program chain of read operations
    async_read(sp, buf_1, read_handler);
    io.run(); //get things rolling
    std::cout << "Last line of the code - serve\n";
*/