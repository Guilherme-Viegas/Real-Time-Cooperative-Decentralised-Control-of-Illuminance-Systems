#ifndef CONTROLLER_H
#define CONTROLLER

#include "ldr_controller.h"
#include "led.h"
#include "util.h"

#include "Arduino.h"
#include <EEPROM.h> // https://www.arduino.cc/en/Tutorial/LibraryExamples/EEPROMWrite


class ControllerPid{

  private:
    //volatile unsigned long t_currentTime, t_previousTime;
    float t_sampleTime = 10;  //[ms] 100Hz
    float t_lastError, t_uint, t_uder; // errors
    boolean t_integral_reset = true; // init integral
    byte t_uff; // feedfoward signal
    float t_ufb; // feedback signal
    float t_reference=0; // lux reference
    float t_kp=0, t_ki=0, t_kd=0;  // gains
    unsigned long t_to=0; // time of new uff

    byte t_ledPin = 0; // led pin
    int t_ldrPin = 0; // ldr pin


  public:
    ControllerPid( byte led, int ldr );  // constructor

    LDR ldr = t_ldrPin;
    LED_ led = t_ledPin;

    void setReferenceLux( float reference );
    void setUff( byte uff );
    byte getU();
    unsigned long get_to();
    void computeFeedbackGain( float output );
    int getLdrPin();
    byte getLedPin();
    void simulator(boolean end);
    void output();

};

#endif
