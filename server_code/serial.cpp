#include "serial.hpp"



// communications::communications( boost::asio::serial_port *s)
communications::communications( boost::asio::io_context *io )
{
    if(DEBUG) std::cout << "This is the initial message of the Serial communication :)\n"; // welcome message

    t_serial = std::unique_ptr<boost::asio::serial_port> ( new boost::asio::serial_port {*io} );

    t_serial->open( MAC_PORT, t_ec); //connect to port

    if(t_ec)   // problems with serial
    {
        if(DEBUG) std::cout << t_ec << " Could not open serial port \n";
        set_coms_not_available();
        return;
    }  
        
    t_serial->set_option(boost::asio::serial_port_base::baud_rate{BAUD_RATE}, t_ec);
}


communications::~communications()
{   
    if(DEBUG) std::cout << "This is the final message of the Serial communication :(\n";
    boost::asio::write( *t_serial, boost::asio::buffer("+RPiE"), t_ec );
    t_serial->close();
}

/*
*   Connects to a Arduino and returns the number of desks
*/
uint8_t communications::has_hub()
{   
    // std::cout << "Waiting for the arduino's delay ...\n";
    // sleep(2);
    // https://stackoverflow.com/questions/39517133/write-some-vs-write-boost-asio - "Since you're only sending a little data, you don't save much time by returning before all the data's sent.(write_some)"
    boost::asio::write( *t_serial, boost::asio::buffer("+RPiG"), t_ec );
    // boost::asio::read( *t_serial, t_buf4, t_ec );   // expected "A<>:)"

    // gets answer
    char tmp[1] {};

    do  // waits for the first 'A' (sent by an Arduino, if not it was just luck)
    {   
        boost::asio::read( *t_serial, t_buf4, t_ec );
        t_buf4.sgetn(tmp, 1);
    } while ( std::strncmp(tmp, "A", 1) != 0x0 );

    char ch[BUFFER_SIZE] {};
    t_buf4.sgetn(ch, BUFFER_SIZE-1);

    for(int i = BUFFER_SIZE-1; i>0; i--)
    {
        ch[i] = ch[i-1];
    }
    ch[0] = tmp[0];

    if(DEBUG) std::cout << ch[0] << (int)(uint8_t)ch[1] << ch[2] << ch[3] << "\t"<< t_ec << std::endl;
    // std::cout << (int)(uint8_t)ch[0] << " " << (int)(uint8_t)ch[1] << " " << (int)(uint8_t)ch[2] << " " << (int)(uint8_t)ch[3] << "\t"<< t_ec << std::endl;


    if( ch[0] == 'A' && ch[2] == ':' && ch[3] == ')')
    {
        t_num_lamps =  (uint8_t)ch[1];
    }

    if(DEBUG) std::cout << "It was found " << (int)t_num_lamps << " desk"<< ( (t_num_lamps != 1) ? "s" : "") << "!\n";
    return t_num_lamps;
}

/*
*   Ask to the Arduino the initial states
*/
void communications::write_command()
{
    boost::asio::write( *t_serial, boost::asio::buffer("+RPiS"), t_ec );
    // boost::asio::read( *t_serial, t_buf4, t_ec );
              
}

/*
*   Reads Arduino's commands
*/
void communications::read_async_command( office *the_office )
{   // https://web.fe.up.pt/~ee96100/projecto/Tabela%20ascii.htm
    if(!t_coms_available) return;

    async_read( *t_serial, t_buf4, [ this, the_office ]( const boost::system::error_code &t_ec, std::size_t len)
        {  
            char ch[BUFFER_SIZE] {};
            t_buf4.sgetn(ch, BUFFER_SIZE);

            if(ch[0] == 's') // stream
            {
                char ch2[BUFFER_SIZE] { };

                boost::asio::streambuf aux_{2};
                boost::asio::read( *t_serial, aux_, this->t_ec );
                aux_.sgetn(ch2, BUFFER_SIZE);

                char command[BUFFER_SIZE+2] {};

                for(int i = 0; i < BUFFER_SIZE; i++)
                {
                    command[i] = ch[i];
                }
                for(int i = 0; i < 2; i++)
                {
                    command[i+BUFFER_SIZE] = ch2[i];
                }

                if(!t_ec)
                {
                    // the_office->get_state();
                    the_office->updates_database( command, BUFFER_SIZE+2 );
                    read_async_command( the_office );
                }
            }
            else    // normal command
            {
                if(!t_ec)
                {
                    // the_office->get_state();
                    the_office->updates_database( ch, BUFFER_SIZE );
                    read_async_command( the_office );
                }
            }



        }
    );
}