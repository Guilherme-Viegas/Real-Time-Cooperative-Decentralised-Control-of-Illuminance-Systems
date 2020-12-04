#include "util.hpp"


/**
 * Compute the binary number from a deciaml number
 * 
 * @return bitset
 */
std::bitset<ASCII> ascii2Binary(char ch)
{
    std::bitset<ASCII> bin;

    std::cout << (int)ch << "\n";

    for( int i=ASCII-1; i>=0; i-- )
    {  
        bin[i] = ch%2;  // evaluating the minor bit
        ch = ch/2;  // shift right
    }

    // for( int i=0; i < ASCII; i++ )
    // {   
    //     std::cout << bin[i];    // prints ASCII 
    // }
    // std::cout << std::endl;

    return bin;
}


