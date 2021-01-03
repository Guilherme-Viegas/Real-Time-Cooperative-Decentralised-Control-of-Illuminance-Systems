#ifndef ASYNC_SERVER_HPP
#define ASYNC_SERVER_HPP

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <iostream>
#include <string>   // for std::string
#include <sstream> //for std::stringstream 
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include "database.hpp"
#include "serial.hpp"

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
    communications *t_serial;

    std::stringstream t_ss {};

    boost::asio::steady_timer t_timer;

public:

    tcp_connection(boost::asio::io_service *io, office *database, communications *serial);
    ~tcp_connection(){ if(t_socket.is_open()){ t_socket.close(); } }
    
    std::string t_client_address; 
    void new_client(){ t_ss << &t_socket; t_client_address = t_ss.str(); }
    tcp::socket &socket(){ new_client(); return t_socket; }

    boost::array<char, 1024> t_recv_buffer;

    void start_receive();
    void handle_receive(const boost::system::error_code& error,  size_t bytes_transferred);
    void send_acknowledgement( bool ack_err );
    void start_timer();
};

class tcp_server
{
private:
    void start_accept();

    tcp_connection *new_connection;
    office *t_database;
    communications *t_serial;

    boost::asio::io_service *t_io;
    tcp::acceptor t_acceptor;

public:
    tcp_server(boost::asio::io_service *io, unsigned short port, office *database, communications *serial);
    ~tcp_server(){ t_acceptor.close(); delete new_connection; }
};

#endif
