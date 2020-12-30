#include "async_server.hpp"

/* --------------------------------------------------------------------------------
   |                                  UDP                                        |
   -------------------------------------------------------------------------------- */

udp_server::udp_server(boost::asio::io_service *io, unsigned short port, office *database):
    t_database(database),
    t_socket(*io, udp::endpoint(udp::v4(), port))
{
start_receive();
}

void udp_server::start_receive()
{
    t_socket.async_receive_from(
        boost::asio::buffer(t_recv_buffer), t_remote_endpoint,
        boost::bind(
            &udp_server::handle_receive,
            this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
        )
    );
}

void udp_server::handle_receive(const boost::system::error_code& error,  size_t bytes_transferred)
{
    std::string command = std::string(t_recv_buffer.begin(), t_recv_buffer.begin()+bytes_transferred);
    char order = 's';
    char type = 'l';
    int address = 0;

    sscanf(command.c_str(), "%c %c %i", &order, &type, &address);

    std::cout << "Received: '" << order << ' ' << type << ' ' << address << "'\t" << " bytes received: " << bytes_transferred << std::endl;

    if (!error || error == boost::asio::error::message_size)
    {   

        std::string header =  std::string(1,order) +  '\t' +  std::string(1,type) + '\t' + std::to_string(address) + '\t';

        if( 1 > address || address > t_database->t_num_lamps ){ send_acknowledgement(false); }
        else if( order == 'b' ){ send_last_minute(header, type, address); }
        else if( order == 's' ){ set_stream( type, address ); }
        start_receive();
    }
}

void udp_server::send_last_minute( std::string header, char type, int address )
{
    std::unique_ptr<float[]> value = t_database->t_lamps_array[ address-1 ]->t_lumminace.get_all();

    for(int i = 0; i < N_POINTS_MINUTE; i++ )
    {       
        std::string response = std::to_string(value[i]) ;

        response = header + response.erase(response.size()-5);

        t_socket.async_send_to( boost::asio::buffer(response.c_str(),response.size() ), t_remote_endpoint,
            [ response ]( const boost::system::error_code &t_ec, std::size_t len ){ 
                std::cout << response << std::endl;
            }
        );
    }
}

void udp_server::set_stream( char type, int address )
{   
    int decision = t_database->set_upd_stream( type, address, &t_socket, &t_remote_endpoint ) ;
    std::cout << "Decision = " << decision << std::endl;
    if( decision == 0 ){ return; }
    else if( decision == 1 ){ send_acknowledgement(true); }
    else{ send_acknowledgement(false); }

}

void udp_server::send_acknowledgement(bool ack_err)
{
    std::string response = (std::string("\t\t\t\t\t\t\t\t")) + (ack_err ? "ack" : "err");

    t_socket.async_send_to( boost::asio::buffer(response.c_str(),response.size() ), t_remote_endpoint,
        [ response ]( const boost::system::error_code &t_ec, std::size_t len ){ 
            std::cout << response << std::endl;
        }
    );
}



/* --------------------------------------------------------------------------------------
   |                                  TCP                                                |
   | https://www.boost.org/doc/libs/1_35_0/doc/html/boost_asio/tutorial/tutdaytime3.html |                                |
   -------------------------------------------------------------------------------------- */

tcp_server::tcp_server(boost::asio::io_service *io, unsigned short port, office *database):
    t_database(database),
    t_io(io),
    t_acceptor(*io, tcp::endpoint(tcp::v4(), port))
{   

    t_database->get_state();
    start_accept();
}

/*
 * Creates a socket and initiates an asynchronous accept operation to wait for a new connection.
 */
void tcp_server::start_accept()
{

    tcp_connection::pointer new_connection { tcp_connection::create( t_io ) };

    t_acceptor.async_accept(
        new_connection->socket(),
            boost::bind(
                &tcp_server::handle_accept,
                this,
                new_connection,
                boost::asio::placeholders::error
                )
            );
}

/*
 *  This functino is called when the asynchronous accept operation initiated by start_accept() finishes. It services the client request, and then calls start_accept() to initiate the next accept operation.
 */
void tcp_server::handle_accept(tcp_connection::pointer new_connection, const boost::system::error_code &error)
{
    if (!error)
    {
        // std::thread new_client( new_connection->start() );
        // new_client.join()
        new_connection->start();
        start_accept();
    }
    else
    {
        std::cout << "Mandar err pq nao criou bem a ligação!!" << std::endl;
    }
    
}