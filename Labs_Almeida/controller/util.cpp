#include "util.h"

/*
 * Normalize PWM between 0 and 255
 *
 *@return PWM
 */
 
float boundPWM(float u){

    u = u < 0 ? 0 : u;
    u = u > MAX_DIGITAL ? MAX_DIGITAL : u;
    return u;
}
