This folder contains the Controller PID, and to run it connect one LDR between VCC and an analog pin and one led between a digital pin and ground, and speacify it: ```ControllerPid pid(3, A0); // led and ldr pin```.

Containing of each file:
  * [controller.ino](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/controller.ino) - main file where in the setup function,  classes as Taus, Led and Ldr are initializated as well as the gain computation. In the void loop, it can be defined a new reference (please note that there are limits of Lux, which the program fulfil).
  * [controller.cpp](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/controller.cpp) and * [controller.h](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/controller.h) - 
  * [ldr_controller.cpp](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/ldr_controller.cpp) and * [controller.ino](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/ldr_controller.h) - 
  * [led.cpp](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/led.cpp) and * [controller.h](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/led.h) - 
  * [util.cpp](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/util.cpp) and * [controller.ino](https://github.com/Guilherme-Viegas/SCTDR/new/master/Labs_Almeida/controller/util.h) - 
