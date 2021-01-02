#ifndef ASYNC_SERVER_HPP
#define ASYNC_SERVER_HPP

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <iostream>
#include <string>
#include <thread>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include "database.hpp"

/* --------------------------------------------------------------------------------
   |                                  UDP                                        |
   -------------------------------------------------------------------------------- */

using boost::asio::ip::udp;

class udp_server
{

private:
    void start_receive();
    void handle_receive(const boost::system::error_code &error, size_t bytes_transferred);
    void send_last_minute( std::string header, char type, int address );
    void set_stream( char type, int address );
    void send_acknowledgement( bool ack_err );

    office *t_database;

    udp::socket t_socket;
    udp::endpoint t_remote_endpoint;
    boost::array<char, 1024> t_recv_buffer;

public:
    udp_server(boost::asio::io_service *io, unsigned short port, office *database);
    ~udp_server(){ t_socket.close(); };
};

/* --------------------------------------------------------------------------------------
   |                                  TCP                                                |
   | https://www.boost.org/doc/libs/1_35_0/doc/html/boost_asio/tutorial/tutdaytime3.html |                                |
   -------------------------------------------------------------------------------------- */

using boost::asio::ip::tcp;

class tcp_connection//: public boost::enable_shared_from_this<tcp_connection>
{

private:

    tcp::socket t_socket;
    office *t_database;

public:

    tcp_connection(boost::asio::io_service *io, office *database);

    tcp::socket &socket(){ return t_socket; }

    boost::array<char, 1024> t_recv_buffer;

    void start_receive();
    void handle_receive(const boost::system::error_code& error,  size_t bytes_transferred);
    void send_acknowledgement( bool ack_err );
};

class tcp_server
{
private:
    void start_accept();

    // void handle_accept(boost::shared_ptr<tcp_connection> new_connection, const boost::system::error_code &error);
    
    office *t_database;

    boost::asio::io_service *t_io;
    tcp::acceptor t_acceptor;

public:
    tcp_server(boost::asio::io_service *io, unsigned short port, office *database);
    ~tcp_server(){ t_acceptor.close(); }
};

#endif
