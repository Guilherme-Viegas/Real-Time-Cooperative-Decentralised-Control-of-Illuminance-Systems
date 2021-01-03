#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <unistd.h>
#include <fstream>

#define BUFFER_SIZE 1024
#define PORT 18700
#define COMS_MSG "Command not found. Type 'Comds' to see a list of available commands!" << std::endl
#define CLOSE_TCP std::cout << "Close TCP client!" << std::endl
#define CLOSE_UDP std::cout << "Close UDP connection!" << std::endl

using namespace boost::asio;

boost::asio::streambuf stm_buff{BUFFER_SIZE};
boost::array<char, BUFFER_SIZE> tcp_message{};
boost::array<char, BUFFER_SIZE> udp_message{};

// global variables to control whether the client runs or not
bool stop_server = false;
bool tcp_connection = false;
bool udp_connection = false;

// file 
std::ofstream file;

void print_commands()
{
    std::cout << "---------------------------------------------TCP Commands---------------------------------------------" << std::endl; 
    std::cout << "| g l <i>     - get the illuminance at desk with address <i>                                         |" << std::endl;
    std::cout << "| g d <i>     - get the current duty-cycle at luminaire <i>                                          |" << std::endl;
    std::cout << "| g o <i>     - get current occupancy state at desk <i> -  - 0 unoccupied OR 1 occupied              |" << std::endl;
    std::cout << "| o <i> <val> - set current occupancy at desk <i> - 0 unoccupied OR 1 occupied                       |" << std::endl;
    std::cout << "| g O <i>     - get lower bound on illuminance for occupied state at desk                            |" << std::endl;
    std::cout << "| O <i> <val> - set lower bound on illuminance at <val> for occupied state at desk <i>               |" << std::endl;
    std::cout << "| g U <i>     - get lower bound on illumincance for unoccupied state at desk <i>                     |" << std::endl;
    std::cout << "| U <i> <val> - set lower bound on illumincance at <val> for unoccupied sate at desk <i>             |" << std::endl;
    std::cout << "| g L <i>     - get current illumincance lower bound at desk                                         |" << std::endl;
    std::cout << "| g x  <i>    - get current external illuminance at desk <i>                                         |" << std::endl;
    std::cout << "| g r <i>     - get current illuminance control refeence at desk <i>                                 |" << std::endl;
    std::cout << "| g c <i>     - get current energy cost at desk <i>                                                  |" << std::endl;
    std::cout << "| c <i> <val> - set current energy cost at desk <i>                                                  |" << std::endl;
    std::cout << "| g p <i>     - get instantaneous power consumption at desk <i>                                      |" << std::endl;
    std::cout << "| g p T       - get instantaneous power consumption in the whole system                              |" << std::endl;
    std::cout << "| g t <i>     - get elapsed time since last restart                                                  |" << std::endl;
    std::cout << "| g e <i>     - get accumulated energy consumption at desk <i> since the last  system restart        |" << std::endl;
    std::cout << "| g e T       - get total accumulated energy consumption since last system restart                   |" << std::endl;
    std::cout << "| g v <i>     - get accumulated  visibility error at desk <i> since last system restart              |" << std::endl;
    std::cout << "| g v T       - get total visibility error since last system restart                                 |" << std::endl;
    std::cout << "| g f <i>     - get accumulated flicker error at desk <i> since last system restart                  |" << std::endl;
    std::cout << "| g f T       - get total flicker error since last system restart                                    |" << std::endl;
    std::cout << "| r           - restart system                                                                       |" << std::endl;
    std::cout << "|--------------------------------------------UDP Commands--------------------------------------------|" << std::endl;
    std::cout << "| b <x> <i>   - get last minute buffer of variable <x> of desk <i>; NOTE: <x> can be 'l' or 'd'      |" << std::endl;
    std::cout << "| s <x> <i>   - stop stream of real-time variable <x> of desk <i>; NOTE: <x> can be 'l' or 'd'       |" << std::endl;
    std::cout << "| file        - at the end of a command to open and write values from UDP streams                    |" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------------------" << std::endl;

}

void udp_start_read_server(ip::udp::socket *client)
{
    client->async_receive(buffer(udp_message, BUFFER_SIZE),
                          [=](const boost::system::error_code &err, std::size_t bytes_transferred) {
                              if (!err && bytes_transferred)
                              {
                                  std::string command = std::string(udp_message.begin(), udp_message.begin() + bytes_transferred);
                                  std::cout << "[UDP] server: " << command << std::endl;
                                  if(file.is_open()){ file << command.substr(6, command.size()).append("\n"); }
                                  udp_start_read_server(client);
                              }
                              else
                              {
                                  CLOSE_UDP;
                                  udp_connection = false;
                                  client->close();
                              }
                          });
}

void tcp_start_read_server(ip::tcp::socket *client)
{
    client->async_read_some(buffer(tcp_message, BUFFER_SIZE),
                            [=](const boost::system::error_code &err, std::size_t bytes_transferred) {
                                if (!err && bytes_transferred)
                                {
                                    std::string command = std::string(tcp_message.begin(), tcp_message.begin() + bytes_transferred);
                                    std::cout << "[TCP] server: " << command << std::endl;
                                    tcp_start_read_server(client);
                                }
                                else
                                {
                                    CLOSE_TCP;
                                    tcp_connection = false;
                                    client->close();
                                    stop_server = true; // shut down!
                                }
                            });
}

void print_invalid_command()
{
    std::cout << COMS_MSG;

    if (!udp_connection)
    {
        std::cout << "UDP connection not available." << std::endl;
    }
    if (!tcp_connection)
    {
        std::cout << "TCP connection not available." << std::endl;
    }
}

void start_read_input(boost::asio::posix::stream_descriptor *stm_desc, ip::tcp::socket *tcp_client, ip::udp::socket *udp_client)
{

    async_read_until(*stm_desc, stm_buff, '\n',
                     [=](const boost::system::error_code &err, std::size_t bytes_transferred) {
                         if (!err && bytes_transferred)
                         {
                             // init variables
                             char order = '.';
                             char type = '.';
                             unsigned int address = 0;
                             char trash[]{};
                             char input[]{};
                             const char *command{};
                             std::string str_command;
                             int valid_command = 1;
                             float set_value = 0.0;
                             

                             // read input
                             int size = stm_buff.size();
                             stm_buff.sgetn(input, size); 
                             stm_buff.consume(size);
                             std::string str_input = std::string(input);


                             // find the word 'file' 
                             int file_itr = str_input.find("file");
                             if( file_itr != -1 )   // writes in file
                             {  
                                 if(file_itr > 0 ){file_itr--;} // removes the space before it
                                 str_input.erase(file_itr,5);
                                 
                                 if(!file.is_open())
                                 {  
                                    file.open(str_input.substr(0, str_input.size()-1).append(".txt"), std::ofstream::out );
                                 }
                             }
                             // find the word 'close' 

                             else if( str_input.find("close") != -1 )   // writes in file
                             {  
                                 stop_server = true;
                                 return; 
                             }

                             // find the word 'Comds' 
                             else if( str_input.find("Comds") != -1  )   // writes in file
                             {  
                                 print_commands();
                                 start_read_input(stm_desc, tcp_client, udp_client);
                             }

                             
                             str_input.append("\n");    // makes'\n' a delimiter

                             // get order
                             sscanf(str_input.c_str(), "%c %[^\n]", &order, trash);
                             if (!order)
                             {
                                 order = input[0];
                             } // forces to read order

                             // select from posible commands
                             if (err)
                             {
                                 std::cout << "Async terminal read off" << std::endl;
                                 return;
                             } // error occurred

                             // commands for UDP server
                             switch (order)
                             {
                             case 'b': // get last minute buffer of variable <x> of desk <i>
                             case 's': // start/stop stream of real-time variable <x> of desk <i>
                             {   
                                 valid_command = 0; // ignores
                                 sscanf(trash, "%c %u %[^\n]", &type, &address, trash);
                                 if ((type == 'l' || type == 'd') && udp_connection)
                                 {
                                     str_command = std::string(1, order) + std::string(1, type) + std::to_string(address);
                                     command = str_command.c_str();

                                     udp_client->async_send(buffer(command, strlen(command)),
                                                            [=](const boost::system::error_code &err, std::size_t len) {
                                                                std::cout << "[UDP] client: " << str_command << std::endl;
                                                            });
                                 }
                                 else
                                 {
                                     print_invalid_command();
                                 }
                                 break;
                             }
                             // commands for TCP server
                             case 'g': // get commands
                             {
                                 sscanf(trash, "%c %u %[^\n]", &type, &address, trash);
                                 std::cout << "type: " << type << " address: " << address << " trash: " << trash <<  std::endl;
                                 switch (type)
                                 {
                                 case 'e': // get accumulated energy consumption in the system <T> or at desk <i> since the last restart
                                 case 'f': // get flicker error in the system <T> or at desk <i> since the last restart
                                 case 'p': // get instantaneous power consumption in the system <T> or at desk <i>
                                 case 'v': // get visibility error in the system <T> or at desk <i> since the last restart
                                 {
                                     if (trash[strlen(trash) - 1] == 'T' && trash[0] == type && ((strlen(trash) == 2) || (trash[1] == ' ' && strlen(trash) == 3)))
                                     {
                                         address = 0;
                                     }
                                 }
                                 case 'c': // get current cost energy at desk <i>
                                 case 'd': // get current duty cicle at luminance at desk <i>
                                 case 'l': // get current illuminance at luminance at desk <i>
                                 case 'L': // get current illuminance lower bound at desk <i>
                                 case 'O': // get lower bound on illuminance for Occupied state at desk <i>
                                 case 'o': // get current occupancy state at desk <i>
                                 case 'r': // get current illuminance control reference at desk <i>
                                 case 't': // get elapsed time since last restart
                                 case 'U': // get lower bound on illuminance for Unoccupied state at desk <i>
                                 case 'x': // get current external illuminace at desk <i>
                                 {
                                     str_command = std::to_string(address) + std::string(1, order) + std::string(1, type);
                                     break;
                                 }
                                 default:
                                 {
                                     valid_command = -1;
                                     break;
                                 }
                                 }
                                 break;
                             }
                             case 'c': // set current energy cost at desk <x>
                             case 'O': // set lower bound on illuminance for Occupied state at desk <i>
                             case 'o': // set current occupancy state at desk <i>
                             case 'U': // set lower bound on illuminance for Unoccupied state at desk <i>
                             {
                                 std::string str_trash = std::string(trash);
                                 int b_ = str_trash.find(' ');
                                 try
                                 {
                                    address = (int)std::stoi(str_trash.substr(0, b_));
                                    set_value = std::stof(str_trash.substr(b_));
std::cout << "trash: '" << trash << "' Address: '" << address << "' Float Set: '" << set_value << "'" << std::endl;
                                    str_command = std::to_string(address) + std::string(1, order) + 's' + std::to_string(set_value);
                                 }
                                 catch (const std::exception& e)
                                 {
                                     valid_command = -1;
                                 }
                                 break;
                             }
                             case 'r':  // restart system
                             {   
                                 if( strlen(input) == 2 && input[1] == '\n'  )
                                 {
                                     str_command = std::string(1,order);
                                 }
                                 else
                                 {
                                     valid_command = -1;
                                 }
                                 
                                 break;
                             }
                             default:
                             {
                                 valid_command = -1;
                                 break;
                             }
                             }

                             command = str_command.c_str();
                             if (valid_command==1 && tcp_connection)
                             {
                                 tcp_client->async_send(buffer(command, strlen(command)),

                                                        [=](const boost::system::error_code &err, std::size_t len) {
                                                            // Nice Job :)
                                                            std::cout << "[TCP] client: " << str_command << std::endl;
                                                        });
                             }
                             else if (valid_command == -1)
                             {
std::cout << "comando inválido linha 216" << std::endl;
                                 print_invalid_command();
                             }
                             start_read_input(stm_desc, tcp_client, udp_client);
                         }
                     });
}

void find_TCP_server(ip::tcp::socket *tcp_socket, ip::tcp::endpoint *tcp_endpoint)
{
    tcp_socket->async_connect(*tcp_endpoint,
                              [=](const boost::system::error_code &error) {
                                  if (!error)
                                  {
                                      tcp_connection = true;
                                      std::cout << "TCP client is UP" << std::endl;
                                      tcp_start_read_server(tcp_socket);
                                  }
                                  else
                                  {
                                      tcp_socket->close();
                                      find_TCP_server(tcp_socket, tcp_endpoint);
                                  }
                              });
}

void start_UDP_connection(ip::udp::socket *udp_socket, ip::udp::endpoint *udp_endpoint)
{
    udp_socket->async_connect(*udp_endpoint,
                              [=](const boost::system::error_code &error) {
                                  if (!error)
                                  {
                                      udp_connection = true;
                                      std::cout << "UDP connection is UP" << std::endl;
                                      udp_start_read_server(udp_socket);
                                  }
                                  else
                                  {
                                      udp_socket->close();
                                      start_UDP_connection(udp_socket, udp_endpoint);
                                  }
                              });
}



int main()
{
    io_context io;

    // Connects to the endpoints
    // https://www.whatismyip.com
    ip::tcp::socket tcp_socket(io);
    ip::tcp::endpoint tcp_endpoint(ip::address::from_string("127.0.0.1"), PORT); // 46.189.132.92
    find_TCP_server(&tcp_socket, &tcp_endpoint);

    ip::udp::socket udp_socket(io);
    ip::udp::endpoint udp_endpoint(ip::address::from_string("127.0.0.1"), PORT + 1); // 46.189.132.92
    start_UDP_connection(&udp_socket, &udp_endpoint);

    boost::asio::posix::stream_descriptor stm_desc{io, ::dup(STDIN_FILENO)};

    /* Async functions */

    start_read_input(&stm_desc, &tcp_socket, &udp_socket);

    std::signal(SIGINT, [](int sig) { std::cout << "\t Safely close the client \n"; stop_server = true; });

    while (!stop_server)
    {
        io.poll_one();
    }

    if ( tcp_socket.is_open() )
    {
        tcp_socket.close();
        CLOSE_TCP;
    }
    if ( udp_socket.is_open() )
    {
        udp_socket.close();
        CLOSE_UDP;
    }

    if ( file.is_open() )
    {
        file.close();
    }

    return 0;
}

// void connect_handler(const boost::system::error_code& error)
// {
//   if (!error)
//   {
//     // Connect succeeded.
//   }
// }
