#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <unistd.h>

#define BUFFER_SIZE 1024
#define PORT 18700

using namespace boost::asio;

boost::asio::streambuf stm_buff { BUFFER_SIZE };
boost::array<char, BUFFER_SIZE> udp_message { } ;
boost::array<char, BUFFER_SIZE> tcp_message { } ;

void udp_start_read_server(  ip::udp::socket *client  )
{

    client->async_receive( buffer(udp_message, BUFFER_SIZE),
        [ = ]( const boost::system::error_code &err, std::size_t bytes_transferred)
        {   
            if( !err && bytes_transferred )
            {
                std::string command = std::string(udp_message.begin(), udp_message.begin() + bytes_transferred );
                std::cout << command << std::endl;
            }
            // std::cout << "UDP Server - error: " << err << "\tbytes: " << bytes_transferred << std::endl;

            udp_start_read_server( client );
        }
    );
}

void tcp_start_read_server(  ip::tcp::socket *client  )
{
    client->async_receive( buffer(tcp_message, BUFFER_SIZE),
        [ = ]( const boost::system::error_code &err, std::size_t bytes_transferred)
        {
            if( !err && bytes_transferred )
            {
                std::string command = std::string(tcp_message.begin(), tcp_message.begin() + bytes_transferred );
                std::cout << command << std::endl;
            }
                // std::cout << "TCP Server - error: " << err << "\tbytes: " << bytes_transferred << std::endl;
                tcp_start_read_server( client );
        }
    );
    //client->cancel();
}

void udp_start_read_input( boost::asio::posix::stream_descriptor *stm_desc, ip::udp::socket *client ) {


    async_read_until( *stm_desc, stm_buff, '\n' ,
        [ = ](const boost::system::error_code & err, std::size_t bytes_transferred) {
            
            if( !err && bytes_transferred )
            {
                int size = stm_buff.size();
                char command[] {};

                stm_buff.sgetn(command , size-1);
                stm_buff.consume(1);

                if( err ){ std::cout << "Async terminal read off\n"; return; }
                else if( strlen(command) != 3 ){ std::cout << "Usage: <order> <type> <address> -> 3 bytes" << std::endl; }
                else if( (command[0] == 'b' || command[0] == 's') && (command[1] == 'l' || command[1] == 'd') )
                {   
                    client->async_send( buffer(command, strlen(command)),

                        [ = ]( const boost::system::error_code &err, std::size_t len )
                        {
                            // Nice Job :)
                        }
                    );                
                }
                else
                {
                    std::cout << "Command not found. Type 'Comds' to see a list of available commands!\n";
                }
            }
            udp_start_read_input( stm_desc, client );
        }
    );
}

int main()
{
    io_context io;

    ip::tcp::socket tcp_socket(io);
    ip::tcp::endpoint tcp_endpoint( ip::address::from_string("127.0.0.1"), PORT ); // ip::tcp::v4()
    tcp_socket.async_connect(tcp_endpoint, [](const boost::system::error_code& error){} );

    ip::udp::socket udp_socket(io);
    ip::udp::endpoint udp_endpoint( ip::address::from_string("127.0.0.1"), PORT+1 ); // ip::udp::v4()
    udp_socket.async_connect(udp_endpoint, [](const boost::system::error_code& error){} );

    boost::asio::posix::stream_descriptor stm_desc { io, ::dup(STDIN_FILENO) };

    /* Async functions */ 

    // with TCP server
    tcp_start_read_server( &tcp_socket ); 
    
    // with UDP server
    udp_start_read_server( &udp_socket );
    udp_start_read_input( &stm_desc, &udp_socket ) ;
    
    io.run();
    
}


// void connect_handler(const boost::system::error_code& error)
// {
//   if (!error)
//   {
//     // Connect succeeded.
//   }
// }



