#include "serial.hpp"

#include <unistd.h>

// create the stream to receive the console input
boost::asio::streambuf stm_buff { 1024 };
using ec = boost::system::error_code;
using sd = boost::asio::posix::stream_descriptor;

void start_read_input( sd *stm_desc ) {
    async_read_until( *stm_desc, stm_buff, '\n' ,
        [ stm_desc ](const ec & err, std::size_t len) {
            std::cout << &stm_buff;// << std::endl;
            start_read_input( stm_desc ); }
    );
}

int main()
{

    boost::asio::io_context io;
    boost::asio::serial_port s{io};
    sd stm_desc { io, ::dup(STDIN_FILENO) };
    
    communications server( &s );

    uint8_t numHubs = server.hasHub();
    std::cout << "It was found " << (int)numHubs << " desk"<< ( (numHubs > 1) ? "s" : "") << "!\n";

    server.write_command();
    start_read_input( &stm_desc );
    
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