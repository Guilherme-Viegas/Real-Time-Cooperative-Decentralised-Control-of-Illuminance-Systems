#include "serial.hpp"



/*
boost::asio::io_context io;
boost::asio::serial_port sp{io};

boost::asio::streambuf buf_1{1}; //read buffer
boost::asio::streambuf buf_trash; //read buffer


//forward declaration of write_handler to timer_handler
void write_handler(const boost::system::error_code &ec, size_t nbytes);
void timer_handler(const boost::system::error_code &ec)
{
    //timer expired – launch new write operation
    boost::asio::async_write(sp, boost::asio::buffer("RPi"), write_handler);
    std::cout << "Hello RPi\n";
}

void write_handler(const boost::system::error_code &ec, size_t nbytes)
{
    //tim.expires_after(boost::asio::chrono::seconds{2});
    std::cout << "IS there any connection?\n";
    //tim.async_wait(timer_handler);
}

void read_handler(const boost::system::error_code &ec, size_t nbytes)
{
    char ch = buf_1.sgetc();
    buf_1.consume(1);
    std::cout << (int)(uint8_t)ch <<  std::endl;
    //ascii2Binary(ch);
    async_read(sp, buf_1, read_handler);
}
*/

communications::communications( boost::asio::serial_port *s)
{
    std::cout << "This is the initial message of the Serial communication :)\n"; // welcome message

    t_serial = s;
    t_serial->open( MAC_PORT, t_ec); //connect to port

    if(t_ec)   // problems with serial
    {
        std::cout << "Could not open serial port \n";
        return -1;
    }  
        
    t_serial->set_option(boost::asio::serial_port_base::baud_rate{BAUD_RATE}, t_ec);
}


communications::~ communications()
{   
    std::cout << "This is the final message of the Serial communication :(\n";
    t_serial->close();
}

/*
*   Connects to a Arduino and returns the number of desks
*/
uint8_t communications::hasHub()
{   
    // std::cout << "Waiting for the arduino's delay ...\n";
    sleep(2);
    // https://stackoverflow.com/questions/39517133/write-some-vs-write-boost-asio - "Since you're only sending a little data, you don't save much time by returning before all the data's sent.(write_some)"
    boost::asio::write( *t_serial, boost::asio::buffer("RPi"), t_ec );
    boost::asio::read( *t_serial, t_buf4, t_ec );   // expected "A<>:)"

    // gets answer
    char ch[BUFFER_SIZE] {};
    t_buf4.sgetn(ch, BUFFER_SIZE);

    //std::cout << ch[0] << (int)(uint8_t)ch[1] << ch[2] << ch[3] << "\t"<< t_ec << std::endl;

    if( ch[0] == 'A' && ch[2] == ':' && ch[3] == ')')
    {
        t_numLamps =  (uint8_t)ch[1];
    }
    return t_numLamps;
}

void communications::write_command()
{
    boost::asio::write(*t_serial, boost::asio::buffer("gli"), t_ec );
    boost::asio::read( *t_serial, t_buf4, t_ec );

    char ch[BUFFER_SIZE];
    t_buf4.sgetn(ch, BUFFER_SIZE);

    float num = bytes2float( (uint8_t)ch[2], (uint8_t)ch[3] );

    std::cout << (int)(uint8_t)ch[0] << " "
              << (int)(uint8_t)ch[1] << "\t -> \t"
              << bytes2float( (uint8_t)ch[0], (uint8_t)ch[1] ) << std::endl
              << (int)(uint8_t)ch[2] << " "
              << (int)(uint8_t)ch[3] << "\t -> \t"
              << num << std::endl;
              
}

float communications::bytes2float(uint16_t mostSignificativeBit, uint8_t lessSignificativeBit)
{
    float decimalNumber = lessSignificativeBit & 0xF;   // gets 4 less significatives bits
    float integerNumber = (mostSignificativeBit << 4) + ((lessSignificativeBit & 0xF0) >> 4);   // gets 12 most significatives bits

    if(decimalNumber == 15 )    // invalid read detected
    {
        std::cout << "INVALID NUMBER - number must be positive\t";
        decimalNumber = 0;
    }
    
    //std::cout << "integer " << integerNumber << " decimal " <<decimalNumber << std::endl;
    return integerNumber + decimalNumber*0.1;
}