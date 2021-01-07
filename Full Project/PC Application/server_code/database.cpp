#include "database.hpp"

office::office( uint8_t num_lamps ) : t_num_lamps(num_lamps)
{   
    if(DEBUG) std::cout << "Welcome to the Office!: " << this << std::endl;    // greeting

    t_lamps_array = new lamp* [t_num_lamps];    // creats an array of lamps and return he array of poiters to lamps

    for( int l=0; l<t_num_lamps; l++)    // for each lamp will create one struct 
    {
        t_lamps_array[l] = new lamp { l+1 };   // stores the address of the new lamp in the array of pointers to lamp object  
    }
}

office::~office()
{   
    std::lock_guard<std::mutex> lock(t_mutex);
    if(DEBUG) std::cout << "\nExits the office, see you later aligator! "  << this << std::endl ; // goodbye message
    for( int l=0; l<t_num_lamps; l++) { delete t_lamps_array[l]; }    // free the memory of each lamp
    delete[] t_lamps_array;  // free the memory of the array of teh lamps' address
}

/*
*   Writes new values on database
*/
void office::updates_database( char command[], uint8_t size )
{   
    std::lock_guard<std::mutex> lock(t_mutex);
    
    float value = 0.0;
    //static bool first = true;

    char type = command[0];
    int address = (int)(uint8_t)command[1];    // make sure it converts well
    int set_command = 0;    // commands that are set by the client
    //bool error = (int)(uint8_t)( 0x0F & command[3]) == 15;
    bool error = (0x8 & command[2]);

    switch (type)
    {
    case 'A':   // restart
    {   
        set_command = -1;
        if(command[2] == ':' && command[3] == ')')
        {   
            restart_it_all( address );  // constains the number of lamps
            address = 0;
            value = 0;
            set_command = 1;
        }
        
        break;
    }
    case 't':   //  get elapsed time since last restart
    {   
        t_time_since_last_restart = (float)(address<<12) + bytes_2_float(command[2], command[3]);
        if(DEBUG) std::cout << "Time since last restart: " << t_time_since_last_restart << " segundos.\n";
        break;
    }
    case 'o':   // set current occupancy state at desk <i> - send this before case 'O' during the arduino setup because it will use one of its initial values as checkpoint
    {   
        value = bytes_2_float(command[2], command[3]);
        if( error )//(int)value*100
        {       
            value = value - 0x80;   // subtracting the error
            set_command = -1;
            break;
        }
        else if( t_lamps_array[address-1]->get_occupied_value() != -1.0 )   // not to print in the first time
        {
            set_command = 1;
        }
        // Updates the value in the dataset
        t_lamps_array[address-1]->set_state((bool)(int)value);

        if(DEBUG) std::cout << "Desk[" << address << "]\tThe state was step to: " << ( t_lamps_array[address-1]->get_state() ? "occupied" : "unoccupied") << "\n";
        break;
    }
    case 'O':   // set lower bound on illuminance for Occupied state at desk <i>
    {
        value = bytes_2_float(command[2], command[3]);
        if( error )
        {   
            value = value - 0x80;   // subtracting the error
            set_command = -1;
            break;
        }
        else if( t_lamps_array[address-1]->get_occupied_value() != -1.0 )
        {   
            set_command = 1;
        }
        // Updates the value in the dataset
        t_lamps_array[address-1]->set_occupied_value( value );
    
        if(DEBUG) std::cout << "Desk[" << address << "]\tThe occupied value is " << t_lamps_array[address-1]->get_occupied_value() << "\n";
        break;
    }
    case 'U':   // set lower bound on illuminance for Unoccupied state at desk <i>
    {
        value = bytes_2_float(command[2], command[3]);
        if( error )
        {
            value = value - 0x80;   // subtracting the error
            set_command = -1;
            break;
        }
        else if( t_lamps_array[address-1]->get_unoccupied_value() != -2.0 )
        {
            set_command = 1;
        }
        // Updates the value in the dataset
        t_lamps_array[address-1]->set_unoccupied_value( value );

        if(DEBUG) std::cout << "Desk[" << address << "]\tThe unoccupied value is " << t_lamps_array[address-1]->get_unoccupied_value() << "\n";
        break;
    }
    case 's':   // stop stream of real-time variable <x> of desk <i>; NOTE: <x> can be 'l' or 'd'
    {   
        set_command = false;
        float luminance = bytes_2_float(command[2], command[3]);
        t_lamps_array[address-1]->t_luminance.insert_newest( luminance );

        float duty_cicle = bytes_2_float(command[4], command[5])/100.0;
        t_lamps_array[address-1]->t_duty_cicle.insert_newest( duty_cicle );

        t_lamps_array[address-1]->compute_performance_metrics_at_desk( luminance, duty_cicle );

        // updates time since last system restart when the information about the first one is recived
        if( (address-1) == 0 ){ t_time_since_last_restart += SAMPLE_TIME_MILIS*std::pow(10,-3); }

        // streams
        if(t_stream && ( t_stream_address == address ) ){ udp_stream( (t_stream_type == 'l') ? luminance : duty_cicle);  }

       break;
    }
    case 'c': // set current energy cost at desk <x>
    {
        value = bytes_2_float(command[2], command[3]);
        if( error )
        {   
            value = value - 0x80;   // subtracting the error
            set_command = -1;
            break;
        }
        else if( t_lamps_array[address-1]->get_nominal_power() != -1.0 )
        {
            set_command = 1;
        }
        // Updates the value in the dataset
        t_lamps_array[address-1]->set_nominal_power( value );
    
        if(DEBUG) std::cout << "Desk[" << address << "]\tThe cost value is " << t_lamps_array[address-1]->get_nominal_power() << "\n";
        break;
    }
    case 'x':
    case 'r':
    {
        set_command = 0; // speacial case
        std::string client_msg =  std::string(1, type) + std::to_string(address);
        std::vector<int>::size_type sz = t_clients_address.size();
        value = bytes_2_float(command[2], command[3]);

        for(unsigned int clt = 0; clt < sz ; clt++)
        {
            if ( ! t_clients_command.at(clt).compare(client_msg) )   // return status
            {   
                std::cout << "Pop command value " << set_command << std::endl;
                t_acknowledge.at(clt) = 10*value;
            }
        }
        break;
    }
    default:
        std::cout << "Default at switch " << address << std::endl;
        break;
    }

    if( set_command != 0 )   // happens when the arduino return same value that was set by the client 
    {   
        int int_value = value * 10;
        std::string client_msg = std::string(1, type) + std::to_string(address) + std::to_string(int_value);

        std::vector<int>::size_type sz = t_clients_address.size();
        for(unsigned int clt = 0; clt < sz ; clt++)
        {
            if ( ! t_clients_command.at(clt).compare(client_msg) )   // return status
            {   
                std::cout << "Pop command ack/err " << set_command << std::endl;
                t_acknowledge.at(clt) = set_command;
            }
        }
    }
}

/*
*   Converts 2 bytes that contain 12 bit to integer and 4 bit to decimal part into a float number
*/
float office::bytes_2_float(uint8_t most_significative_bit, uint8_t less_significative_bit) const
{
    float decimal_number = less_significative_bit & 0xF;   // gets 4 less significatives bits
    float integer_number = (most_significative_bit << 4) + ((less_significative_bit & 0xF0) >> 4);   // gets 12 most significatives bits

    if(decimal_number == 15.0 )    // invalid read detected
    {
        if(DEBUG) std::cout << "INVALID NUMBER - number must be positive\t";
        return decimal_number*0.01;
    }
    
    // std::cout << "integer " << integer_number << " decimal " <<decimal_number << std::endl;
    return integer_number + decimal_number*0.1;
}

/*
 * This function is used to represent a float number in 2 bytes.
 * 
 * The 12 most significatives bits represents the integer part with resolution 0:4095
 * The 4  less significatives bits represents the decimal part with resolution 0:9
 * 
 * Sends '*' when there is nothing to be said, which corresnponds to 10 in the 4 less significative bits
 */
void office::float_2_bytes(float fnum, u_int8_t bytes[2]) const
{   
    // the maxium admissive value is 4095.94(9)
    fnum = fnum < pow(2,12)-0.05 ? fnum : pow(2,12)-1+0.9;  // 2^12-1 == 4095, 12 bits representation + 4bits to decimal representation(0.1 -> 0.9)
  
    uint16_t output = fnum < 0; // flag that represents if fnum is negative
    uint16_t inum = fnum;  // integer part
    fnum = fnum-inum; // floating part
    uint8_t dnum = round(10*fnum);  // integer decimal part

    // increments one when the decimal part rounds up
    if(dnum == 10){
        dnum = 0;
        inum++;
    }
    
    inum = inum<<4; // the integer will be represent in the first 1,5 bits
    output = output ? 15: inum + dnum; // send 15 when fnum is negative, which should not be possible
    
    bytes[1] = (uint8_t) (output>>8); // wirte second byte - DEBUG: Serial.println((byte) (output>>8))
    bytes[0] = (uint8_t) output;  // write first byte - DEBUG: Serial.println((byte) output);
}

/*
 * Computes the total accumulated energy since last system restart
 */
float office::get_accumulated_energy_consumption()
{   
    std::lock_guard<std::mutex> lock(t_mutex);

    float energy = 0.0;
    for(int i = 0; i < t_num_lamps; i ++)
    {
        energy += t_lamps_array[i]->get_accumulated_energy_consumption_at_desk();
    }
    return energy;
}

/*
 * Computes the instataneous total power consumption in the system
 */
float office::get_instant_power()
{   
    std::lock_guard<std::mutex> lock(t_mutex);
    
    float power = 0.0;
    for(int i = 0; i < t_num_lamps; i ++)
    {
        power += t_lamps_array[i]->get_instant_power_at_desk();
    }
    return power;
}

/*
 * Computes the total accumulated visibility error since last system restart
 */
float office::get_accumulated_visibility_error()
{   
    std::lock_guard<std::mutex> lock(t_mutex);
    
    float visibility = 0.0;
    for(int i = 0; i < t_num_lamps; i ++)
    {
        visibility += t_lamps_array[i]->get_accumulated_visibility_error_at_desk();
    }
    return visibility;
}

/*
 * Computes the total accumulated flicker error since last system restart
 */
float office::get_accumulated_flicker_error()
{   
    std::lock_guard<std::mutex> lock(t_mutex);

    float flicker = 0.0;
    for(int i = 0; i < t_num_lamps; i ++)
    {
        flicker += t_lamps_array[i]->get_accumulated_flicker_error_at_desk();
    }
    return flicker;
}

/*
 * Controls stream parameters
 * return whereas the stream has started 0, if it was stopped correctly 1 or not wrong command -1
 */
int office::set_upd_stream( char type, int address, boost::asio::ip::udp::socket* socket, boost::asio::ip::udp::endpoint* endpoint)
{   
    std::lock_guard<std::mutex> lock(t_mutex);

    if(t_stream)   // command to stop stream
    {   
        if( t_stream_type == type && t_stream_address == address )
        {
            t_stream = false;
            return 1;    // stoped stream successfully
        }
        else
        {
             return -1;    // stream was not stoped
        }
    }

    // updates data and starts stream;
    t_socket = socket;
    t_endpoint = endpoint;
    t_stream_type = type;
    t_stream_address = address;
    t_stream = true;
    return 0;
}
/*
 * Sends real time data to UDP client
 */
 void office::udp_stream( float value )
 {  
    std::string str_value =  std::to_string( value );
    std::string str_time =  std::to_string( t_time_since_last_restart );

    std::string response =  std::string(1,'s') +  '\t' +  std::string(1,t_stream_type) + '\t'
                        + std::to_string(t_stream_address) + '\t' + str_value.erase(str_value.size()-5)
                        + '\t' + str_time.erase(str_time.size()-4);

    t_socket->async_send_to( boost::asio::buffer(response.c_str(),response.size() ), (*t_endpoint),
        [ response ]( const boost::system::error_code &t_ec, std::size_t len ){ 
            //std::cout << response << std::endl;
            // Nice Job :)
        }
    );
 }

int office::get_num_lamps()
{
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_num_lamps;
}

void office::restart_it_all( int lamps )
{   

    // deletes previous nodes
    for(int i = 0; i<t_num_lamps; i++)
    {   
        delete t_lamps_array[i];
    }
    delete[] t_lamps_array;


    // clears office class (this)
    t_time_since_last_restart = 0.0;
    t_num_lamps = lamps; 
    t_stream = false;
    t_stream_type = ' ';
    t_stream_address = 0;
    
    std::vector<int>::size_type sz = t_clients_address.size();
    for (int clt = sz - 1; clt >= 0; clt--)
    {
        if( t_clients_command.at(clt).compare("A00") ) // if message is to restart does not restart
        {
            t_clients_address.erase( t_clients_address.begin() + clt );
            t_clients_command.erase( t_clients_command.begin() + clt );
            t_acknowledge.erase( t_acknowledge.begin() + clt );
        }
    }

    t_lamps_array = new lamp* [t_num_lamps];    // creats an array of lamps and return he array of poiters to lamps

    // creates new nodes
    for( int l=0; l<t_num_lamps; l++)    // for each lamp will create one struct 
    {
        t_lamps_array[l] = new lamp { l+1 };   // stores the address of the new lamp in the array of pointers to lamp object  
    }

}

/* --------------------------------------------------------------------------------
   |                                  Lamp                                        |
   -------------------------------------------------------------------------------- */

lamp::lamp( int address ) : t_address( (uint8_t)address ) // stores personal address of CAN BUS
{
    if(DEBUG) std::cout << "I am a lamp at the address " << (int)t_address << " ;)\n";    // greeting
}

lamp::~lamp()
{   
    std::lock_guard<std::mutex> lock(t_mutex);
    if(DEBUG) std::cout << "Ups... seems that one lamp is not available anymore.\n";    // goodbye message
}

/*
 *  Computes Performence metrcis such as energy, power, flicker, visibility
 */
void lamp::compute_performance_metrics_at_desk( float new_luminance, float new_duty_cicle )
{   
    std::lock_guard<std::mutex> lock(t_mutex);
    // Computes Instante Power
    t_instant_power = t_nominal_power*new_duty_cicle;

    // Computes accumulated energy consumption
    t_accumulated_energy_consumption += t_nominal_power*t_duty_cicle_prev*SAMPLE_TIME_MILIS*std::pow(10,-3);

    // Computes accumulated visibility error
    t_n_samples++;
    double Reference = t_state ? t_occupied_value : t_unoccupied_value;
    t_accumulated_flicker_error  = ( (t_n_samples-1)*t_accumulated_flicker_error + std::max(0.0, Reference-new_luminance) ) / t_n_samples;

    // Computes accumulated flicker error
    float flicker = ( ((new_luminance-t_luminance_prev_1)*(t_luminance_prev_1-t_luminance_prev_2)) < 0 ) ? 
                                                ((new_luminance-t_luminance_prev_1)+(t_luminance_prev_1-t_luminance_prev_2)) : 0;
    t_accumulated_flicker_error += flicker;

    // Updates new_values
    t_luminance_prev_2 = t_luminance_prev_1;
    t_luminance_prev_1 = new_luminance;
    t_duty_cicle_prev = new_duty_cicle;
}

float lamp::get_accumulated_energy_consumption_at_desk()
{ 
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_accumulated_energy_consumption;
}

float lamp::get_instant_power_at_desk()
{
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_instant_power;
}

float lamp::get_accumulated_visibility_error_at_desk()
{
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_accumulated_visibility_error;
}

float lamp::get_accumulated_flicker_error_at_desk()
{
    std::lock_guard<std::mutex> lock(t_mutex); 
    return t_accumulated_flicker_error;
}

 void lamp::set_state( bool state )
 {
     std::lock_guard<std::mutex> lock(t_mutex);
     t_state = state;
 }

 bool lamp::get_state()
 {
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_state;
 }

 void lamp::set_occupied_value( float value )
 {
    std::lock_guard<std::mutex> lock(t_mutex);
    t_occupied_value = value;
 }

float lamp::get_occupied_value()
{
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_occupied_value;
}

 void lamp::set_unoccupied_value( float value )
 {
    std::lock_guard<std::mutex> lock(t_mutex);
    t_unoccupied_value = value;
 }

float lamp::get_unoccupied_value()
{
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_unoccupied_value;
}

void lamp::set_nominal_power( float value )
 {
    std::lock_guard<std::mutex> lock(t_mutex);
    t_nominal_power = value;
 }

float lamp::get_nominal_power()
{
    std::lock_guard<std::mutex> lock(t_mutex);
    return t_nominal_power;
}
