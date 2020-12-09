#ifndef HUB_HPP
#define HUB_HPP

#include <Math.h>
#include "Arduino.h"
#define BUFFER_SIZE 5 // number of char to read plus \0

void float2bytes(float fnum);
void greeting(int numLamps);
void hub();


#endif
