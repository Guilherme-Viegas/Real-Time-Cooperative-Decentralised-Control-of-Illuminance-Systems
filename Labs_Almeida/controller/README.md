# PI controller

The code follows a syntax of [CamelCase](https://pt.wikipedia.org/wiki/CamelCase) and also the classes are encapsulated throughout the files.

## Files description
This folder contains the Controller PID, and to run it, it should be connected one LDR between VCC and an analog pin and one led between a digital pin and ground, and then speacify it: ```ControllerPid pid(DIGITAL_PIN, ANALOG_PIN);```.

Containing of each file:
  * [controller.ino](./controller.ino) - main file where in the setup() function classes as Taus, Led and Ldr are initializated as well as the gain computation. In the void loop(), it can be defined a new reference in Lux (please note that there are limits of Lux, which the program fulfils).
  * [controller.cpp](./controller.cpp) and [controller.h](./controller.h) - file that contains the operations related with the Controller: feedback and feedfoward.
  * [ldr_controller.cpp](./ldr_controller.cpp) and [ldr_controller.h](./ldr_controller.h) - file containing both types Tau and Ldr, as well as its functions.
  * [led.cpp](./led.cpp) and [led.h](./led.h) - this file contains the class LED where it is stored the information related with it which allow the controller to change led intensity.
  * [util.cpp](./util.cpp) and [util.h](./util.h) - it contains functions that can be use allover the code.
  
## Control flow

The controller is divided in two parts, the setup and the loop.

### setup()

The Led and Ldr are declared in Pins 3 and A0, respectively, as well as both Taus. To compute the gain (Pwm to Lux) and offset (room light with led turned off) the program does one step up at each time and then down (total of 510steps). In each step it waits 50ms to the led establish, so it will take at least 510\*50e-3 = 25,5 seconds. 

### loop()

