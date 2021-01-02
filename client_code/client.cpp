#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <unistd.h>

#define BUFFER_SIZE 1024
#define PORT 18700
#define COMS_MSG "Command not found. Type 'Comds' to see a list of available commands!\n"
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

void udp_start_read_server(ip::udp::socket *client)
{
    client->async_receive(buffer(udp_message, BUFFER_SIZE),
                          [=](const boost::system::error_code &err, std::size_t bytes_transferred) {
                              if (!err && bytes_transferred)
                              {
                                  std::string command = std::string(udp_message.begin(), udp_message.begin() + bytes_transferred);
                                  std::cout << "[UDP] server: " << command << std::endl;
                                  udp_start_read_server(client);
                              }
                              else
                              {
                                  CLOSE_UDP;
                                  ;
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
        std::cout << "UDP connection not available.";
    }
    if (!tcp_connection)
    {
        std::cout << "TCP connection not available.";
    }
}

void start_read_input(boost::asio::posix::stream_descriptor *stm_desc, ip::tcp::socket *tcp_client, ip::udp::socket *udp_client)
{

    async_read_until(*stm_desc, stm_buff, '\n',
                     [=](const boost::system::error_code &err, std::size_t bytes_transferred) {
                         if (!err && bytes_transferred)
                         {
                             // init variables
                             int size = stm_buff.size();

                             char order = '.';
                             char type = '.';
                             unsigned int address = 0;
                             char trash[]{};
                             char input[]{};
                             const char *command{};
                             std::string str_command;
                             bool valid_command = true;
                             float set_value = 0.0;

                             // read input
                             stm_buff.sgetn(input, size); //size-1
                             stm_buff.consume(size);

                             // get order
                             sscanf(input, "%c %[^\n]", &order, trash);
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
                             //else if( strlen(command) != 3 ){ std::cout << "Usage: <order> <type> <address> -> 3 bytes, instead of " << strlen(command) << std::endl ; }

                             // commands for UDP server
                             switch (order)
                             {
                             case 'b': // get last minute buffer of variable <x> of desk <i>
                             case 's': // start/stop stream of real-time variable <x> of desk <i>
                             {
                                 sscanf(trash, "%c %u %[^\n]", &type, &address, trash);
                                 if ((type == 'l' || type == 'd'))
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
                                     valid_command = false;
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
                                 address = (int)std::stoi(str_trash.substr(0, b_));
                                 set_value = std::stof(str_trash.substr(b_));

                                 std::cout << "trash: '" << trash << "' Address: '" << address << "' Float Set: '" << set_value << "'" << std::endl;
                                 str_command = std::to_string(address) + std::string(1, order) + 's' + std::to_string(set_value);
                                 break;
                             }
                             default:
                             {
                                 valid_command = false;
                                 break;
                             }
                             }

                             command = str_command.c_str();
                             if (valid_command)
                             {
                                 tcp_client->async_send(buffer(command, strlen(command)),

                                                        [=](const boost::system::error_code &err, std::size_t len) {
                                                            // Nice Job :)
                                                            std::cout << "[TCP] client: " << str_command << std::endl;
                                                        });
                             }
                             else
                             {
                                 std::cout << "aqui?" << std::endl;
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

    if (tcp_socket.is_open())
    {
        tcp_socket.close();
        CLOSE_TCP;
    }
    if (udp_socket.is_open())
    {
        udp_socket.close();
        CLOSE_UDP;
    }
}

// void connect_handler(const boost::system::error_code& error)
// {
//   if (!error)
//   {
//     // Connect succeeded.
//   }
// }
