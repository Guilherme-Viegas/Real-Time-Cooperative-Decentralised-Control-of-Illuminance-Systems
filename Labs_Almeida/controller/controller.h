#ifndef CONTROLLER_H
#define CONTROLLER

#include "ldr_controller.h"
#include "led.h"

#include "Arduino.h"
#include <EEPROM.h> // https://www.arduino.cc/en/Tutorial/LibraryExamples/EEPROMWrite

class ControllerPid{

  private:
    //volatile unsigned long t_currentTime, t_previousTime;
    float t_sampleTime = 10;  //[ms] 100Hz
    float t_uInt = 0; // integral error
    boolean t_integralReset = true; // init integral
    float t_uff = 0, t_ufb = 0; // feedfoward and feedback signal before multiplied by constant

    // flags
    boolean t_deadZone = true; // silent the error when it is small
    boolean t_antiWindUp = true; // prevent integral windup
    boolean t_feedfoward = true;  // feedfoward 
    boolean t_feedback = true;  // feedback
    
    float t_reference=0; // lux reference
    float t_lastReference=0; // last reference of lux
    float t_lastError=0;  // prioir error
    
    float t_kp=0, t_ki=0;  // gains
    unsigned long t_to=0; // time of new uff

    byte t_ledPin = 0; // led pin
    int t_ldrPin = 0; // ldr pin

    static const short t_meanSize = 10; // N last output saves
    float t_output[t_meanSize] = { }; // [ N-1 ] - stores N ouput values
    float t_sum = 0; //  The sum in the last position
    unsigned short t_counter = 0; // output counter

  public:
    ControllerPid( byte led, int ldr );  // constructor

    LDR ldr = t_ldrPin;
    LED_ led = t_ledPin;

    void setReferenceLux( float reference );
    void setUff( float uff );
    float getU();
    unsigned long get_to();
    void computeFeedbackGain(  float output );
    int getLdrPin();
    byte getLedPin();
    float simulator(boolean print);
    void output();
    boolean has_feedback(){ return t_feedback; }

};

#endif
