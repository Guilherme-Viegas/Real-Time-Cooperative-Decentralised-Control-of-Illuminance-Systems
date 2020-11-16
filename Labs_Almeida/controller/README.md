# PI controller

## Files description
This folder contains the Controller PID, and to run it, it should be connected one LDR between VCC and an analog pin and one led between a digital pin and ground, and then speacify it: ```ControllerPid pid(DIGITAL_PIN, ANALOG_PIN);```.

Containing of each file:
  * [controller.ino](./controller.ino) - main file where in the setup() function classes as Taus, Led and Ldr are initializated as well as the gain computation. In the void loop(), it can be defined a new reference in Lux (please note that there are limits of Lux, which the program fulfils).
  * [controller.cpp](./controller.cpp) and [controller.h](./controller.h) - file that contains the operations related with the Controller: feedback and feedfoward.
  * [ldr_controller.cpp](./ldr_controller.cpp) and [ldr_controller.h](./ldr_controller.h) - file containing both types Tau and Ldr, as well as its functions.
  * [led.cpp](./led.cpp) and [led.h](./led.h) - this file contains the class LED where it is stored the information related with it which allow the controller to change led intensity.
  * [util.cpp](./util.cpp) and [util.h](./util.h) - it contains functions that can be use allover the code.
  
## Control flow
