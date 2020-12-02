// #include <string>
// #include <iostream> // input/output e.g.: cin, cout, cerr, clog and w___ (wide)
// #define BOOST_BIND_NO_PLACEHOLDERS // https://stackoverflow.com/questions/13596697/c11-placeholders-with-boost
// #include <boost/bind.hpp>
// #include <boost/shared_ptr.hpp>
// #include <boost/enable_shared_from_this.hpp>
// #include <boost/asio.hpp>
// // https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/tutorial/tutdaytime3.html
// using boost::asio::ip::tcp;


// class tcp_connection : public boost::enable_shared_from_this<tcp_connection>
// {
// public:
//     /* It is used shared_ptr and enable_shared_from_this because it is wanted to keep the
//     tcp_connection object alive as long as there is an operation that refers to it.*/
//     typedef boost::shared_ptr<tcp_connection> pointer;

//     static pointer create(boost::asio::io_context& io)
//     {
//         return pointer(new tcp_connection(io));
//     }

//     tcp::socket& socket()
//     {
//         return socket_;
//     }

//     void start()
//     {
//         message_ = "You have queried a new TCP connection to the server hosted in a Raspberry Pi!";

//         /* It is used boost::asio::async_write(), rather than ip::tcp::socket::async_write_some(),
//         to ensure that the entire block of data is sent.*/
//         boost::asio::async_write(socket_, boost::asio::buffer(message_),
//             boost::bind(&tcp_connection::handle_write, shared_from_this()));
//     }
// private:

//     tcp_connection(boost::asio::io_context& io)
//     : socket_(io)
//     {
//     }

//     void handle_write()
//     {
//     }

//     tcp::socket socket_;
//     std::string message_;
// };

// class tcp_server
// {
// public:
//     tcp_server(boost::asio::io_context& io) : acceptor_(io, tcp::endpoint(tcp::v4(), 13))
//     {
//         start_accept();
//     }
    
// private:
//     /* Creates a socket and initiates an asynchronous accept operation to wait for a new connection. */
//     void start_accept()
//     {
//         tcp_connection::pointer new_connection = tcp_connection::create(acceptor_.get_executor().context());

//         acceptor_.async_accept(new_connection->socket(), boost::bind(&tcp_server::handle_accept, this, new_connection,
//             boost::asio::placeholders::error));
//     }

//     /*  It is called when the asynchronous accept operation initiated by start_accept() finishes.
//         It services the client request, and then calls start_accept() to initiate the next accept operation. */
//     void handle_accept(tcp_connection::pointer new_connection, const boost::system::error_code& error)
//     {
//         if(!error)
//         {
//             new_connection->start();
//         }

//         start_accept();
//     }

//     tcp::acceptor acceptor_;
// };

// int main(int argc, char *argv[])
// {
    
//     // An asynchronous TCP daytime server
//     try
//     {
//         boost::asio::io_context io; // provides I/O services
//         tcp_server server(io);
//         io.run();   // the server will performs asynchronous operations
//     }
//     catch(std::exception& e){
//         std::cerr << e.what() << std::endl;
//     }
//     std::cout << "Close Server" << std::endl;
//     return 0;
// }

#include <iostream>

int main(){
    std::cout << "Hello World v6\n";
    return 0;
}