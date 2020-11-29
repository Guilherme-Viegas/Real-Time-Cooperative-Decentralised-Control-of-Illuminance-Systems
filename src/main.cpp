#include <iostream> // input/output e.g.: cin, cout, cerr, clog and w___ (wide)
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main(int argc, char *argv[]){
    
    // An asynchronous TCP daytime server
    try{
        if(argc != 2){
            std::cerr << "Usage: client <host>" << std::endl;
            return 1;   //TODO
        }

        boost::asio::io_service io; // provides I/O services
        tcp_server server(io);
        io.run();   // the server will performs asynchronous operations
    }
    catch(std::exceptions& e){
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
