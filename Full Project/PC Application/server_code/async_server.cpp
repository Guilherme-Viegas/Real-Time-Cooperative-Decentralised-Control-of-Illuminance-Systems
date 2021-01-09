#include "async_server.hpp"

/* --------------------------------------------------------------------------------
   |                                  UDP                                        |
   -------------------------------------------------------------------------------- */

udp_server::udp_server(boost::asio::io_service *io, unsigned short port, office *database) : t_database(database),
                                                                                             t_socket(*io, udp::endpoint(udp::v4(), port))
{
    std::cout << "UDP server is open!" << std::endl;
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
            boost::asio::placeholders::bytes_transferred));
}

void udp_server::handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
{
    if (!error && bytes_transferred)
    {
        std::string command = std::string(t_recv_buffer.begin(), t_recv_buffer.begin() + bytes_transferred);
        char order = 'u';
        char type = 'u';
        int address = 0;

        sscanf(command.c_str(), "%c %c %d", &order, &type, &address);

        std::cout << "Received: '" << order << ' ' << type << ' ' << address << "'\t"
                  << " bytes received: " << bytes_transferred << std::endl;

        std::string header = std::string(1, order) + '\t' + std::string(1, type) + '\t' + std::to_string(address) + '\t';

        if (1 > address || address > t_database->get_num_lamps())
        {
            std::string response = "The number of Total desks connected in the network is: " + std::to_string(t_database->get_num_lamps());

            t_socket.async_send_to(boost::asio::buffer(response.c_str(), response.size()), t_remote_endpoint,
                                   [response](const boost::system::error_code &t_ec, std::size_t len) {
                                       std::cout << response << std::endl;
                                   });
        }
        else if (order == 'b') // get last minute buffer of variable <x> of desk <i>; NOTE: <x> can be 'l' or 'd'
        {
            send_last_minute(header, type, address, t_remote_endpoint);
        }
        else if (order == 's') // stop stream of real-time variable <x> of desk <i>; NOTE: <x> can be 'l' or 'd'
        {
            set_stream(type, address, t_remote_endpoint);
        }
    }
    start_receive();
}

void udp_server::send_last_minute(std::string header, char type, int address, udp::endpoint remote_endpoint)
{
    std::unique_ptr<float[]> value = t_database->t_lamps_array[address - 1]->t_luminance.get_all();

    for (int i = 0; i < N_POINTS_MINUTE; i++)
    {
        std::string response = std::to_string(value[i]);

        response = header + response.erase(response.size() - 5);

        t_socket.async_send_to(boost::asio::buffer(response.c_str(), response.size()), remote_endpoint,
                               [response](const boost::system::error_code &t_ec, std::size_t len) {
                                   std::cout << response << std::endl;
                               });
    }
}

void udp_server::set_stream(char type, int address, udp::endpoint remote_endpoint)
{
    int decision = t_database->set_upd_stream(type, address, &t_socket, remote_endpoint);
    std::cout << "Decision = " << decision << std::endl;
    if (decision == 0)
    {
        return;
    }
    else if (decision == 1)
    {
        send_acknowledgement(true);
    }
    else
    {
        send_acknowledgement(false);
    }
}

void udp_server::send_acknowledgement(bool ack_err)
{
    std::string response = (std::string("\t\t\t\t\t\t\t\t")) + (ack_err ? "ack" : "err");

    t_socket.async_send_to(boost::asio::buffer(response.c_str(), response.size()), t_remote_endpoint,
                           [response](const boost::system::error_code &t_ec, std::size_t len) {
                               std::cout << response << std::endl;
                           });
}

/* --------------------------------------------------------------------------------------
   |                                  TCP                                                |
   | https://www.boost.org/doc/libs/1_35_0/doc/html/boost_asio/tutorial/tutdaytime3.html |                                |
   -------------------------------------------------------------------------------------- */

tcp_server::tcp_server(boost::asio::io_service *io, unsigned short port, office *database, communications *serial) : t_database(database),
                                                                                                                     t_serial(serial),
                                                                                                                     t_io(io),
                                                                                                                     t_acceptor(*io, tcp::endpoint(tcp::v4(), port))
{
    start_accept();
}

tcp_server::~tcp_server()
{
    t_acceptor.close();

    // delete all clients backwards
    std::vector<int>::size_type sz = connections.size();
    for (unsigned int i = 0; i < sz; i++)
    {
        delete connections.at(i);
    }
}
/*
 * Creates a socket and initiates an asynchronous accept operation to wait for a new connection.
 */
void tcp_server::start_accept()
{
    new_connection = new tcp_connection{t_io, t_database, t_serial};
    connections.push_back(new_connection); // save clients in the list

    t_acceptor.async_accept(new_connection->socket(),
                            [this](const boost::system::error_code &err) {
                                if (!err)
                                {
                                    std::cout << "New TCP client! " << new_connection->t_client_address << std::endl;
                                    new_connection->start_timer();   // starts timer
                                    new_connection->start_receive(); // start receive instructions
                                }
                                else
                                {
                                    // delete new_connection;
                                }
                                start_accept();
                            } //end async_accept lambda arg
    );                        //end async_accept cal
}

/*
 *  This functino is called when the asynchronous accept operation initiated by start_accept() finishes. It services the client request, and then calls start_accept() to initiate the next accept operation.
 */

tcp_connection::tcp_connection(boost::asio::io_service *io, office *database, communications *serial) : t_socket(*io), t_database(database), t_serial(serial), t_timer(*io)
{
}

/*
 * Receives a new TCP client command
 */
void tcp_connection::start_receive()
{
    t_socket.async_read_some(
        boost::asio::buffer(t_recv_buffer),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            if (!error && bytes_transferred)
            {
                handle_receive(error, bytes_transferred);
            }
            else
            {
                std::cout << "TCP client has left. " << this << std::endl;
                t_socket.close();
            }
        });
}

/*
 * Process the command
 */
void tcp_connection::handle_receive(const boost::system::error_code &error, size_t bytes_transferred)
{
    if (!error && bytes_transferred)
    {
        std::string command = std::string(t_recv_buffer.begin(), t_recv_buffer.begin() + bytes_transferred);

        char order = 't';
        char type = 't';
        int address = -1;
        float value = 0.0;

        sscanf(command.c_str(), "%d %c %c %f", &address, &order, &type, &value);

        std::cout << "Received: '" << order << ' ' << type << ' ' << address << ' ' << value << "\t"
                  << " bytes received: " << bytes_transferred << ' ' << "command: " << command << std::endl;

        std::string response = std::string(1, type) + '\t' + std::to_string(address) + '\t';
        int valid_response = 1; // 1: True , -1 : Fale, 0: do nothing

        if (command.compare("r") == 0)
        {
            t_serial->write_command("+rrrr");
            t_database->t_clients_address.push_back(t_client_address); // appends the clients address
            t_database->t_clients_command.push_back("A00");            // appends the new command --- to be equal as the order instructions
            t_database->t_acknowledge.push_back(-2);
            valid_response = 0;
        }
        else if (0 > address || address > t_database->get_num_lamps())
        {
            valid_response = -1;
        }
        else
        {
            switch (order)
            {
            case 'g':
            {
                switch (type)
                {
                case 'e': // get accumulated energy consumption in the system <T> or at desk <i> since the last restart
                {
                    response += std::to_string(address == 0 ? t_database->get_accumulated_energy_consumption() : t_database->t_lamps_array[address - 1]->get_accumulated_energy_consumption_at_desk());
                    break;
                }
                case 'f': // get flicker error in the system <T> or at desk <i> since the last restart
                {
                    response += std::to_string(address == 0 ? t_database->get_accumulated_flicker_error() : t_database->t_lamps_array[address - 1]->get_accumulated_flicker_error_at_desk());
                    break;
                }
                case 'p': // get instantaneous power consumption in the system <T> or at desk <i>
                {
                    response += std::to_string(address == 0 ? t_database->get_instant_power() : t_database->t_lamps_array[address - 1]->get_instant_power_at_desk());
                    break;
                }
                case 'v': // get visibility error in the system <T> or at desk <i> since the last restart
                {
                    response += std::to_string(address == 0 ? t_database->get_accumulated_visibility_error() : t_database->t_lamps_array[address - 1]->get_accumulated_visibility_error_at_desk());
                    break;
                }
                case 'c': // get current cost energy at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->t_lamps_array[address - 1]->get_nominal_power());
                    }
                    break;
                }
                case 'd': // get current duty cicle at luminance at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->t_lamps_array[address - 1]->t_duty_cicle.get_newest());
                    }
                    break;
                }
                case 'l': // get current illuminance at luminance at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->t_lamps_array[address - 1]->t_luminance.get_newest());
                    }
                    break;
                }
                case 'L': // get current illuminance lower bound at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->t_lamps_array[address - 1]->get_state() == false ? t_database->t_lamps_array[address - 1]->get_unoccupied_value() : t_database->t_lamps_array[address - 1]->get_occupied_value());
                    }
                    break;
                }
                case 'O': // get lower bound on illuminance for Occupied state at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->t_lamps_array[address - 1]->get_occupied_value());
                    }
                    break;
                }
                case 'o': // get current occupancy state at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->t_lamps_array[address - 1]->get_state());
                    }
                    break;
                }
                case 't': // get elapsed time since last restart
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->get_elapesd_time_since_last_restart());
                    }
                    break;
                }
                case 'U': // get lower bound on illuminance for Unoccupied state at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        response += std::to_string(t_database->t_lamps_array[address - 1]->get_unoccupied_value());
                    }
                    break;
                }
                // direct to arduino
                case 'r': // get current illuminance control reference at desk <i>
                case 'x': // get current external illuminace at desk <i>
                {
                    if (address == 0)
                    {
                        valid_response = -1;
                    }
                    else
                    {
                        std::string client_msg = std::string(1, type) + std::to_string(address);
                        // command already in stack
                        if (std::find(t_database->t_clients_command.begin(), t_database->t_clients_command.end(), client_msg) != t_database->t_clients_command.end())
                        {
                            send_acknowledgement(false);
                        }
                        else
                        {
                            valid_response = 0;
                            std::string to_arduino = '+' + client_msg + "**";
                            t_serial->write_command(to_arduino);
                            t_database->t_clients_address.push_back(t_client_address);
                            t_database->t_clients_command.push_back(client_msg); // appends the new command
                            t_database->t_acknowledge.push_back(-2);
                        }
                    }
                    break;
                }
                default:
                {
                    valid_response = -1;
                    break;
                }
                }
                break;
            }
            case 'c': // set current energy cost at desk <x>
            {
                if (address == 0)
                {
                    valid_response = -1;
                }
                else if (value >= 0) // acceptable value
                {
                    valid_response = 2; // do nothing
                }
                else
                {
                    valid_response = 0; // do nothing
                    send_acknowledgement(false);
                }
                break;
            }
            case 'O': // set lower bound on illuminance for Occupied state at desk <i>
            {
                if (address == 0)
                {
                    valid_response = -1;
                }
                else if (value >= 0 && value > t_database->t_lamps_array[address - 1]->get_unoccupied_value()) // acceptable value
                {
                    valid_response = 2; // do nothing
                }
                else
                {
                    valid_response = 0; // do nothing
                    send_acknowledgement(false);
                }
                break;
            }
            case 'o': // set current occupancy state at desk <i>
            {
                if (address == 0)
                {
                    valid_response = -1;
                }
                else if (value >= 0) // acceptable value
                {
                    value = value < 0.05 ? (int)0 : (int)1;
                    valid_response = 2; // do nothing
                }
                else
                {
                    valid_response = 0; // do nothing
                    send_acknowledgement(false);
                }
                break;
            }
            case 'U': // set lower bound on illuminance for Unoccupied state at desk <i>
            {
                if (address == 0)
                {
                    valid_response = -1;
                }
                else if (value >= 0 && value < t_database->t_lamps_array[address - 1]->get_occupied_value()) // acceptable value
                {
                    valid_response = 2; // do nothing
                }
                else
                {
                    valid_response = 0; // do nothing
                    send_acknowledgement(false);
                }
                break;
            }
            default:
            {
                valid_response = -1;
                break;
            }
            }
        }
        if (valid_response == -1) // invalid command
        {
            response = "The number of Total desks connected in the network is: " + std::to_string(t_database->get_num_lamps());
            valid_response = 1;
        }

        if (valid_response == 1) // there is a response to send
        {
            send_string(response);
        }
        else if (valid_response == 2) // get command
        {
            int int_value = round(value * 10);
            std::string client_msg = std::string(1, order) + std::to_string(address) + std::to_string(int_value); // in this case the var order is the type once it is a set command
            std::cout << t_client_address << "\t" << client_msg << std::endl;

            // command already in stack
            if (std::find(t_database->t_clients_command.begin(), t_database->t_clients_command.end(), client_msg) != t_database->t_clients_command.end())
            {
                send_acknowledgement(false);
            }
            else
            {
                u_int8_t val[2]{};                                                                                                                            // 2 bytes with float value
                t_database->float_2_bytes(value, val);                                                                                                        // converts the float to 12 decimal bit and 4 floats
                std::string to_arduino = '+' + std::string(1, order) + std::to_string(address) + std::string(1, (char)val[1]) + std::string(1, (char)val[0]); // msg to be sent
                t_serial->write_command(to_arduino);                                                                                                          // sent message
                t_database->t_clients_address.push_back(t_client_address);                                                                                    // appends the clients address
                t_database->t_clients_command.push_back(client_msg);                                                                                          // appends the new command
                t_database->t_acknowledge.push_back(-2);                                                                                                       // appends the arduino response
            }
        }

        start_receive();
    }
}

/*
 * Sends wheater the information was accepted 'ack' or not 'err'
 */
void tcp_connection::send_acknowledgement(bool ack_err)
{
    std::string response = (std::string("\t\t\t\t\t\t\t\t")) + (ack_err ? "ack" : "err");

    t_socket.async_send(boost::asio::buffer(response.c_str(), response.size()),
                        [response](const boost::system::error_code &t_ec, std::size_t len) {
                            std::cout << response << std::endl;
                        });
}

/*
 * Sends string
 */
void tcp_connection::send_string(std::string s)
{
    s += '\n';
    t_socket.async_send(boost::asio::buffer(s.c_str(), s.size()),
                        [s](const boost::system::error_code &t_ec, std::size_t len) {
                            std::cout << s << std::endl;
                        });
}

/*
 * Checks asynchronous if the database as a new information valid or not
 */
void tcp_connection::start_timer()
{
    t_timer.expires_after(boost::asio::chrono::milliseconds{1500}); // it takes in avergare 1s and 500ms
    t_timer.async_wait([this](const boost::system::error_code &t_ec) {
        bool last_call = false;
        if (t_ec || !t_socket.is_open())
        {
            last_call = true; // erase all commands of that client
        }

        // delete index from the last when the client is done
        std::vector<int>::size_type sz = t_database->t_clients_address.size();
        for (int clt = sz - 1; clt >= 0; clt--)
        {
            std::cout << "Queue of waiting for acknowledge instructions.\nNumber of pendent instructions:\t" << sz << std::endl;
            std::cout << "Client's id:\t" << t_client_address << "\t message :\t" << t_database->t_clients_command.at(clt) << std::endl;

            if (!t_database->t_clients_address.at(clt).compare(t_client_address)) // client has a pendent process
            {
                std::cout << "Waiting message value:" << t_database->t_acknowledge.at(clt) << std::endl;

                if ((t_database->t_acknowledge.at(clt) == -2) && !last_call)
                {
                    continue;
                }
                // 'ack' to be sent
                if (!last_call)
                {
                    if (((char)t_database->t_clients_command.at(clt)[0] == 'x') || ((char)t_database->t_clients_command.at(clt)[0] == 'r')) // get command
                    {
                        std::string response = std::string(1, (char)t_database->t_clients_command.at(clt)[1]) + '\t' +
                                               std::string(1, (char)t_database->t_clients_command.at(clt)[0]) + '\t' +
                                               std::to_string(t_database->t_acknowledge.at(clt) / 10.0);
                        send_string(response);
                    }

                    else // set commands wainting for acknoledge
                    {
                        send_acknowledgement(t_database->t_acknowledge.at(clt) == 1 ? true : false);
                    }
                }
                // erase commands
                std::cout << "The command \t " << t_database->t_clients_command.at(clt)
                          << "\t poped out to \t" << t_database->t_clients_address.at(clt)
                          << "\t with the value \t" << t_database->t_acknowledge.at(clt) << std::endl;

                t_database->t_acknowledge.erase(t_database->t_acknowledge.begin() + clt);
                t_database->t_clients_command.erase(t_database->t_clients_command.begin() + clt); // if the clients wants to know the command, print this before send teh acknowledge
                t_database->t_clients_address.erase(t_database->t_clients_address.begin() + clt);
            }
        }
        if (!last_call)
        {
            start_timer();
        }
    });
}