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

    switch (command[0])
    {
    case 't':
    {
        t_time_since_restart = (double)(command[1]<<12) + bytes_2_float(command[2], command[3]);
        if(DEBUG) std::cout << "Time since last restart: " << t_time_since_restart << " segundos.\n";
        break;
    }
    case 'o':
    {
        t_lamps_array[command[1]-1]->t_state = command[2];   // check if command[3] == '*'

        if(DEBUG) std::cout << "Desk[" << (int)(uint8_t)command[1] << "]\tThe state was step to: " << ( t_lamps_array[command[1]-1]->t_state ? "occupied" : "unoccupied") << "\n";
        break;
    }
    case 'O':
    {
        value = bytes_2_float(command[2], command[3]);
        if( value < t_lamps_array[command[1]-1]->t_unoccupied_value )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        if( t_lamps_array[command[1]-1]->t_unoccupied_value >= 0 ){ std::cout << "[CLIENT]\t>>\tack\n"; }
        // Updates the value in the dataset
        t_lamps_array[command[1]-1]->t_occupied_value = value;
    
        if(DEBUG) std::cout << "Desk[" << (int)(uint8_t)command[1] << "]\tThe occupied value is " << t_lamps_array[command[1]-1]->t_occupied_value << "\n";
        break;
    }
    case 'U':
    {
        value = bytes_2_float(command[2], command[3]);
        if( value > t_lamps_array[command[1]-1]->t_occupied_value )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        if( t_lamps_array[command[1]-1]->t_unoccupied_value >= 0 ){ std::cout << "[CLIENT]\t>>\tack\n"; }
        // Updates the value in the dataset
        t_lamps_array[command[1]-1]->t_unoccupied_value = value;

        if(DEBUG) std::cout << "Desk[" << (int)(uint8_t)command[1] << "]\tThe unoccupied value is " << t_lamps_array[command[1]-1]->t_unoccupied_value << "\n";
        break;
    }
    case 's':
    {
        float luminance = bytes_2_float(command[2], command[3]);
        t_lamps_array[command[1]-1]->t_lumminace.insert_newest( luminance );
        std::cout << "id: " << (int)(uint8_t)command[1] << "\t\tlux: " << (t_lamps_array[command[1]-1]->t_lumminace).get_newest();

        float duty_cicle = bytes_2_float(command[4], command[5]);
        t_lamps_array[command[1]-1]->t_duty_cicle.insert_newest( duty_cicle );

        std::cout << "\tdc: " << t_lamps_array[command[1]-1]->t_duty_cicle.get_newest() << std::endl;

        t_lamps_array[command[1]-1]->compute_performance_metrics_at_desk( luminance, duty_cicle );

        // updates time since last system restart when the information about the first one is recived
        if( (command[1]-1) == 0 ){ t_time_since_restart += SAMPLE_TIME_MILIS*std::pow(10,-3); }

        // ***********TESTE para ver se ele imprime o last minute***********
        //  std::thread last_minute(
        //     [&](){
        //         if( t_lamps_array[command[1]-1]->t_lumminace.is_full() && first ){                      
        //             std::unique_ptr<float[]> array = t_lamps_array[command[1]-1]->t_lumminace.get_all();
        //             for(int i = 0; i < N_POINTS_MINUTE; i++ )
        //             {
        //                 std::cout << "Estou a imprimir o array last minute: " << array[i] << std::endl;
        //             }
        //             std::cout << "Báu cheio!! :)\n";    
        //             first = false;
        //         }
        //     }
        // );
        // last_minute.join();  
         // ***********TESTE***********
       
       break;
    }
    case 'c':
    {
        value = bytes_2_float(command[2], command[3]);
        if( value == 0 )
        {
            std::cout << "[CLIENT]\t>>\terr\n";
            break;
        }
        if( t_lamps_array[command[1]-1]->t_cost >= 0 ){ std::cout << "[CLIENT]\t>>\tack\n"; }
        // Updates the value in the dataset
        t_lamps_array[command[1]-1]->t_cost = value;
    
        if(DEBUG) std::cout << "Desk[" << (int)(uint8_t)command[1] << "]\tThe cost value is " << t_lamps_array[command[1]-1]->t_cost << "\n";
        break;
    }
    default:
        //std::cout << "Default at switch " << (int)(uint8_t)command[0] << std::endl;
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

    if(decimal_number == 15 )    // invalid read detected
    {
        if(DEBUG) std::cout << "INVALID NUMBER - number must be positive\t";
        decimal_number = 0;
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