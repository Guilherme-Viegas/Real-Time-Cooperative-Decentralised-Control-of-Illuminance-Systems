#include "serial.hpp"

// communications::communications( boost::asio::serial_port* s)
communications::communications(boost::asio::io_context *io)
{
    if (DEBUG)
        std::cout << "This is the initial message of the Serial communication :)\n"; // welcome message

    t_serial = std::unique_ptr<boost::asio::serial_port>(new boost::asio::serial_port{*io});

    t_serial->open(RPI_PORT, t_ec); //connect to port

    if (t_ec) // problems with serial
    {
        if (DEBUG)
            std::cout << t_ec << " Could not open serial port \n";
        set_coms_not_available();
        set_coms_not_available();
        return;
    }

    t_serial->set_option(boost::asio::serial_port_base::baud_rate{BAUD_RATE}, t_ec);
}

communications::~communications()
{
    if (DEBUG)
        std::cout << "This is the final message of the Serial communication :(\n";
    boost::asio::write(*t_serial, boost::asio::buffer("+RPiE"), t_ec);
    t_serial->close();
}

/*
*   Connects to a Arduino and returns the number of desks
*/
uint8_t communications::has_hub()
{
    if (!t_coms_available)
        return 0;
    // std::cout << "Waiting for the arduino's delay ...\n";
    // sleep(2);
    // https://stackoverflow.com/questions/39517133/write-some-vs-write-boost-asio - "Since you're only sending a little data, you don't save much time by returning before all the data's sent.(write_some)"
    boost::asio::write(*t_serial, boost::asio::buffer("+RPiG"), t_ec);

    char trash;
    do
    {
        boost::asio::read(*t_serial, t_buf, t_ec);
        trash = t_buf.sgetc();
        t_buf.consume(1);
    } while (trash != '+');

    boost::asio::read(*t_serial, t_buf_command, t_ec);
    char command1[BUFFER_SIZE_COMMAND]{};
    t_buf_command.sgetn(command1, BUFFER_SIZE_COMMAND);

    if (DEBUG)
        std::cout << command1[0] << (int)(uint8_t)command1[1] << command1[2] << command1[3] << "\t" << t_ec << std::endl;
    // std::cout << (int)(uint8_t)ch[0] << " " << (int)(uint8_t)ch[1] << " " << (int)(uint8_t)ch[2] << " " << (int)(uint8_t)ch[3] << "\t"<< t_ec << std::endl;

    int num_lamps = 0;
    if ((command1[0] == 'A') && (command1[2] == ':') && (command1[3] == ')'))
    {
        num_lamps = (int)(uint8_t)command1[1];
    }
    else
    {
        num_lamps = has_hub();
    }

    if (DEBUG)
        std::cout << "It was found " << num_lamps << " desk" << ((num_lamps != 1) ? "s" : "") << "!\n";
    return num_lamps;
}

/*
*   Sends message to Arduino through serial
*/
void communications::write_command(std::string command)
{
    boost::asio::write(*t_serial, boost::asio::buffer(command), t_ec);
}

/*
*   Reads Arduino's commands asyncronously
*/
void communications::read_async_command(office *the_office)
{ // https://web.fe.up.pt/~ee96100/projecto/Tabela%20ascii.htm
    if (!t_coms_available)
        return;

    async_read(*t_serial, t_buf_command,
               [this, the_office](const boost::system::error_code &t_ec, std::size_t len) {
                   // creats command variable
                   char command1[BUFFER_SIZE_COMMAND]{};
                   t_buf_command.sgetn(command1, BUFFER_SIZE_COMMAND);

                   if (command1[0] == 's') // stream
                   {
                       char command2[BUFFER_SIZE_STREAM - BUFFER_SIZE_COMMAND]{};
                       boost::asio::streambuf aux_{BUFFER_SIZE_STREAM - BUFFER_SIZE_COMMAND};
                       boost::asio::read(*t_serial, aux_, this->t_ec);
                       aux_.sgetn(command2, BUFFER_SIZE_STREAM - BUFFER_SIZE_COMMAND);

                       char command[BUFFER_SIZE_STREAM]{};

                       for (int i = 0; i < BUFFER_SIZE_COMMAND; i++)
                       {
                           command[i] = command1[i];
                       }
                       for (int i = 0; i < BUFFER_SIZE_STREAM - BUFFER_SIZE_COMMAND; i++)
                       {
                           command[i + BUFFER_SIZE_COMMAND] = command2[i];
                       }

                       if (!t_ec)
                       {
                           the_office->updates_database(command, BUFFER_SIZE_STREAM);
                           read_until_asynchronous(the_office, '+');
                       }
                   }
                   else // normal command
                   {
                       if (!t_ec)
                       {
                           the_office->updates_database(command1, BUFFER_SIZE_COMMAND);
                           read_until_asynchronous(the_office, '+');
                       }
                   }
               });
}

/*
*  Truly implementation of read until delimiter
*/
void communications::read_until_asynchronous(office *the_office, char delimiter)
{
    if (!t_coms_available)
        return;

    boost::asio::async_read(*t_serial, t_buf,
                            [this, the_office, delimiter](const boost::system::error_code &t_ec, std::size_t len) {
                                char trash;
                                trash = t_buf.sgetc();
                                t_buf.consume(1);

                                trash == delimiter ? read_async_command(the_office) : read_until_asynchronous(the_office, delimiter);
                            });
}