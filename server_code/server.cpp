#include "serial.hpp"
#include "async_server.hpp"

#include <unistd.h>

#define PORT 18700 // https://stackoverflow.com/questions/3855127/find-and-kill-process-locking-port-3000-on-mac

// global variable to control whether the server runs or not
bool stop_server = false;

// create the stream to receive the console input
boost::asio::streambuf stm_buff { 1024 };
using ec = boost::system::error_code;
using sd = boost::asio::posix::stream_descriptor;

int main()
{
    std::signal(SIGINT, [ ](int sig){ std::cout << "\t Querias batinhas com enguias ...\n"; stop_server=true; } );

    boost::asio::io_context io;

    communications the_serial{ &io };

    uint8_t num_lamps = the_serial.has_hub();
    if( num_lamps <= 0 )
    {
        stop_server = true;
        std::cout << "Early exit with" << (int)num_lamps << " lamps" << std::endl;
        //return 0;
    }

    office the_office { num_lamps } ;

    tcp_server server_tcp{ &io, PORT, &the_office };
    udp_server server_udp{ &io, PORT+1, &the_office};

    the_serial.write_command();
    the_serial.read_until_asynchronous( &the_office, '+' );

    // io.run();
    while( !stop_server ) io.poll_one(); //https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/reference/io_service.html

    return 0;

}

// //SYNC_UDP_SERVER
// #include <iostream>
// #include <boost/asio.hpp>
// using namespace boost::asio;
// int main()
// {
//     io_context io;
//     ip::udp::socket server_sock{io, ip::udp::endpoint{ip::udp::v4(), 18700}};
//     std::cout << "Receiving at : " << server_sock.local_endpoint() << std::endl;
//     char buf[1] = {0};
//     for (;;) //ever.
//     {
//         ip::udp::endpoint client_ep;
//     //client endpoint retrieved on receive_from
//     server_sock.receive_from(buffer(buf, 1), client_ep);
//     if (buf[0] != 'q')
//         server_sock.send_to(buffer("Hello World\n"), client_ep);
//     else
//         server_sock.send_to(buffer("q\n"), client_ep);
//     }
// }