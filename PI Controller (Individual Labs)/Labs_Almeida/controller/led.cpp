#include "led.h"

// ---------------------------------------------Led Class---------------------------------------------

Led::Led(){
  pinMode(t_pin, OUTPUT);
  // Serial.println("You created a new LED object locate at pin" + String(t_pin) + "!");
}

/*
 * Sets the brightness
 *
 * @param PWM
 * @param simulator
 */
void Led::setBrightness( float v ){ analogWrite( t_pin, (byte) v ); }

/*
 * Sets the brightness
 *
 * @return output voltage
 */
float Led::getBrightness(){  return analogRead( t_pin ) * VCC/MAX_DIGITAL; } // output (volt)



// ---------------------------------------------Auxiliar Functions---------------------------------------------
