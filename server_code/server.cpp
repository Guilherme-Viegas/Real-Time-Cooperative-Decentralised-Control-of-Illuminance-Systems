#include "serial.hpp"

#include <unistd.h>

#define EXIT_MESSAGE "exit"

// global variable to control whether the server runs or not
bool stop_server = false;

// create the stream to receive the console input
boost::asio::streambuf stm_buff { 1024 };
using ec = boost::system::error_code;
using sd = boost::asio::posix::stream_descriptor;

void start_read_input( sd *stm_desc ) {
    async_read_until( *stm_desc, stm_buff, '\n' ,
        [ stm_desc ](const ec & err, std::size_t len) {

            int size = stm_buff.size();
            char command[] {};

            stm_buff.sgetn(command , size-1);
            stm_buff.consume(size);

            if( !err && std::string(command).compare( EXIT_MESSAGE ) )
            {   
                start_read_input( stm_desc );
            }
            else
            {
                std::cout << "Async terminal read off\n";
            }
            
        }
    );
}


int main()
{   
    std::signal(SIGINT, [ ](int sig){ std::cout << "\t Querias batinhas com enguias ...\n"; stop_server=true; } );
    
    boost::asio::io_context io;
    sd stm_desc { io, ::dup(STDIN_FILENO) };

    communications the_serial{ &io };

    uint8_t num_lamps = the_serial.has_hub();
    if( num_lamps <= 0 )
    {
        stop_server = true;
        std::cout << "EARLY EXIT WIHT " << num_lamps << "LAMPS" << std::endl;
        return 0;
    }

    office the_office { num_lamps } ;

    the_serial.write_command();
    the_serial.read_until_asynchronous( &the_office, '+' );

    start_read_input( &stm_desc ) ;

    // io.run();
    while( !stop_server ) io.poll_one(); //https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/reference/io_service.html

    return 0;
 
}
