#include "phoenix.hpp"


#include <string.h>
#include <iostream> // input/output e.g.: cin, cout, cerr, clog and w___ (wide)
#include <fstream>

std::fstream serialArduino;

int ave() {

    std::string arduinoPort = "/dev/ttyACM0";
    serialArduino.open(arduinoPort, std::ios::in|std::ios::out|std::ios::binary);   // opens file for reading (in) adn writing (out), the binary makes sure the data is read or written without translating new line characters
    bool exitPrompt = false;
    // bool validCommand = false;

    // If the file could not be opened
    if(!serialArduino){
        std::cerr << "Unable to open serial device: " << arduinoPort << std::endl;
        exitPrompt = true;
    }
    while(!exitPrompt){
        /* code */
        break;
    }
    
    serialArduino.close();
    return 0;
}