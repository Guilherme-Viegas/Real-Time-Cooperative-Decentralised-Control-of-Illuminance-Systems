#include "database.hpp"


office::office( uint8_t num_lamps ) : t_num_lamps(num_lamps)
{   
    if(DEBUG) std::cout << "Welcome to the Office!\n";    // greeting

    t_lamps_array = new lamp* [t_num_lamps];    // creats an array of lamps and return he array of poiters to lamps

    for( int l=0; l<t_num_lamps; l++)    // for each lamp will create one struct 
    {
        t_lamps_array[l] = new lamp { l+1 };   // stores the address of the new lamp in the array of pointers to lamp object  
    }
}

office::~office()
{   
    if(DEBUG) std::cout << "\nExits the office, see you later aligator!\n"; // goodbye message
    for( int l=0; l<t_num_lamps; l++) { delete t_lamps_array[l]; }    // free the memory of each lamp
    delete[] t_lamps_array;  // free the memory of the array of teh lamps' address
}


/*
*   Writes new values on database
*/
void office::updates_database( char command[], uint8_t size )
{   
    float value = 0.0;
    //static bool first = true;

    char order = command[0];
    int address = (int)(uint8_t)command[1];

    switch (order)
    {
    case 't':
    {
        t_time_since_restart = (float)(address<<12) + bytes_2_float(command[2], command[3]);
        if(DEBUG) std::cout << "Time since last restart: " << t_time_since_restart << " segundos.\n";
        break;
    }
    case 'o':
    {
        t_lamps_array[address-1]->t_state = command[2];   // check if command[3] == '*'

        if(DEBUG) std::cout << "Desk[" << address << "]\tThe state was step to: " << ( t_lamps_array[address-1]->t_state ? "occupied" : "unoccupied") << "\n";
        break;
    }
    case 'O':
    {
        value = bytes_2_float(command[2], command[3]);
        if( value == 0.15 )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        else if( t_lamps_array[address-1]->t_occupied_value != -1.0 )
        {
            std::cout << "[CLIENT]\t>>\tack\n";
        }
        // Updates the value in the dataset
        t_lamps_array[address-1]->t_occupied_value = value;
    
        if(DEBUG) std::cout << "Desk[" << address << "]\tThe occupied value is " << t_lamps_array[address-1]->t_occupied_value << "\n";
        break;
    }
    case 'U':
    {
        value = bytes_2_float(command[2], command[3]);
        if( value == 0.15 )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        else if( t_lamps_array[address-1]->t_unoccupied_value != -2.0 )
        {
            std::cout << "[CLIENT]\t>>\tack\n";
        }
        // Updates the value in the dataset
        t_lamps_array[address-1]->t_unoccupied_value = value;

        if(DEBUG) std::cout << "Desk[" << address << "]\tThe unoccupied value is " << t_lamps_array[address-1]->t_unoccupied_value << "\n";
        break;
    }
    case 's':
    {
        float luminance = bytes_2_float(command[2], command[3]);
        t_lamps_array[address-1]->t_lumminace.insert_newest( luminance );

        float duty_cicle = bytes_2_float(command[4], command[5])/100.0;
        t_lamps_array[address-1]->t_duty_cicle.insert_newest( duty_cicle );

        t_lamps_array[address-1]->compute_performance_metrics_at_desk( luminance, duty_cicle );

        // updates time since last system restart when the information about the first one is recived
        if( (address-1) == 0 ){ t_time_since_restart += SAMPLE_TIME_MILIS*std::pow(10,-3); }

        // streams
        if(t_stream && ( t_stream_address == address ) ){ udp_stream( (t_stream_type == 'l') ? luminance : duty_cicle);  }

       break;
    }
    case 'c':
    {
        value = bytes_2_float(command[2], command[3]);
        if( value == 0.15 )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        else if( t_lamps_array[address-1]->t_nominal_power != -1.0 )
        {
            std::cout << "[CLIENT]\t>>\tack\n";
        }
        // Updates the value in the dataset
        t_lamps_array[address-1]->t_nominal_power = value;
    
        if(DEBUG) std::cout << "Desk[" << address << "]\tThe cost value is " << t_lamps_array[address-1]->t_nominal_power << "\n";
        break;
    }
    default:
        //std::cout << "Default at switch " << (int)(uint8_t)desk << std::endl;
        break;
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
 * Computes the total accumulated energy since last system restart
 */
float office::get_accumulated_energy_consumption()
{   
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
int office::set_upd_stream( char type, int address, boost::asio::ip::udp::socket *socket, boost::asio::ip::udp::endpoint *endpoint)
{
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
    std::string str_time =  std::to_string( t_time_since_restart );

    //std::to_string( t_time_since_restart)

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

/* --------------------------------------------------------------------------------
   |                                  Lamp                                        |
   -------------------------------------------------------------------------------- */

lamp::lamp( int address ) : t_address( (uint8_t)address ) // stores personal address of CAN BUS
{
    if(DEBUG) std::cout << "I am a lamp at the address " << (int)t_address << " ;)\n";    // greeting
}

lamp::~lamp()
{
    if(DEBUG) std::cout << "Ups... seems that the lamp at address " << (int)t_address << " is not available anymore.\n";    // goodbye message
}

/*
 *  Computes Performence metrcis such as energy, power, flicker, visibility
 */
void lamp::compute_performance_metrics_at_desk( float new_luminance, float new_duty_cicle )
{
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