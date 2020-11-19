# PI controller

The code follows a syntax of CamelCase and also the classes are encapsulated throughout the files.

## Files description
This folder contains the Controller PID, and to run it, it should be connected one LDR between VCC and an analog pin and one led between a PWM pin and ground, and then specify it: ```ControllerPid pid(DIGITAL_PIN, ANALOG_PIN);```.

Containing of each file:
  * [controller.ino](./controller.ino) - main file where in the setup() function classes as Taus, Led and Ldr are initialized as well as the gain computation. In the void loop(), it can be defined a new reference in Lux (please note that there are limits of Lux, which the program fulfils).
  * [controller.cpp](./controller.cpp) and [controller.h](./controller.h) - file that contains the operations related with the Controller: feedback and feedfoward.
  * [ldr_controller.cpp](./ldr_controller.cpp) and [ldr_controller.h](./ldr_controller.h) - file containing both types Tau and Ldr, as well as its functions.
  * [led.cpp](./led.cpp) and [led.h](./led.h) - this file contains the class LED where it is stored the information related with it which allows the controller to change led intensity.
  * [util.cpp](./util.cpp) and [util.h](./util.h) - it contains functions that can be use allover the code.

## Control flow

The controller is divided in two parts, the setup and the loop.

### setup()

The Led and Ldr are declared in Pins 3 and A0, respectively, as well as both Tau functions. To compute the gain (Pwm to Lux) and offset (room light with led turned off) the program does one step up at each time and then down (total of 510steps). In each step it waits 50ms to the led establish, so it will take at least 510\*50e-3 = 25,5 seconds.

### loop()

The reference is setted in Lux and the program bounds the illuminance value, so the bright can be assure by the arduino from the pwm range of 0 to 255, on the led pin.
In order to have a smooth controller, the goal is to follow the theoretical response. This response is described by a first order system equation, given by ![v_{sim} = v_n - (v_n - v_{n-1})\*exp(\frac{t_n - t_{n-1}}{tau_{up/down}(n)}) [V]](https://latex.codecogs.com/svg.latex?v_{sim}%20=%20v_n%20-%20(v_n%20-%20v_{n-1})\*e^{\frac{t_n%20-%20t_{n-1}}{tau_{up/down}(n)}}%20[V]). The relation between Lux and volts is given by ![R_2 = 10^{m \cdot log_{10}(Lux) + b}  \quad \land \quad v_o = V_{cc}\frac{R_1}{R_1 + R_2}](https://latex.codecogs.com/svg.latex?R_2%20=%2010^{m%20\cdot%20log_{10}(Lux)%20+%20b}%20%20\quad%20\land%20\quad%20v_o%20=%20V_{cc}\frac{R_1}{R_1%20+%20R_2}).

In order to have a quick response to the setted reference it is computed if ´´´ t_feedfoward == true ´´´, the volts correspondent of Lux reference, and it is saved in the variable t_uff;

The feedback goal is to cancel the internal and external noise, and is controlled by the boolean ´´´ t_feedback´´´. The calculations are in the function computeFeedbackGain() which is called by an interruption in the timer1, every 10ms (sampling rate = 100 Hz). The interruption takes over 1.24ms.
The feedback is composed by 4 features: deadzone, proportional gain, integral gain, and anti-windUp.
1. The error is the difference between the voltage read and the simulator value in volts and is the input of the feedback module. Then it is applied the dead zone block if ``` t_deadZone == true ``` with the threshold of ```VCC/MAX_DIGITAL == 5/255``` which represents the necessary voltage to increase or decrease at least 1 PWM.
2. The proportional has the goal to lead our response to the reference value (if possible), when the system is facing a disturbance. The proportional gain is ```t_kp = 0.70```.
3. The integral should bring the error to 0, due to the fact that it can written as a causal system. In other words, its response also depends on the past values. The integral gain is ```t_uInt = 0.030```.
4. The anti-windUp effect prevents the integral to accumulate when the system is saturated. This means that whenever the voltage read is not in the interval from 0 to VCC, the integral response which is saved in the variable ```t_uInt``` is saturated. The system response is ![u = u_{int} + k_p*error + u_{ff}](https://latex.codecogs.com/svg.latex?u%20=%20u_{int}%20+%20k_p*error%20+%20u_{ff}) and, with some algebraic manipulation, the integral response can be written as ![u_{int} = u - ( u_{ff} + k_p*error )](https://latex.codecogs.com/svg.latex?u_{int}%20=%20u%20-%20(%20u_{ff}%20+%20k_p*error%20)). To conclude, the integral saturarion limits are when ![u == 0 \lor u == V_{CC}](https://latex.codecogs.com/svg.latex?u%20==%200%20\lor%20u%20==%20V_{CC}), which leads to ![u_{int} \in [ - ( u_{ff} + k_p*error); V_{CC} - ( u_{ff} + k_p*error)] ](https://latex.codecogs.com/svg.latex?u_{int}%20\in%20[%20-%20(%20u_{ff}%20+%20k_p*error);%20V_{CC}%20-%20(%20u_{ff}%20+%20k_p*error)%20]).

Lastly the value read from analog pin is converted to Lux according to ![R_2 = R_1 \bigg( \frac{V_{cc}}{v_0}-1 \bigg)  \quad \land \quad Lux = 10^{\frac{log_{10}(R_2) - b}{m}}](https://latex.codecogs.com/svg.latex?R_2%20=%20R_1%20\bigg(%20\frac{V_{cc}}{v_0}-1%20\bigg)%20%20\quad%20\land%20\quad%20Lux%20=%2010^{\frac{log_{10}(R_2)%20-%20b}{m}}). It was also computed a mean of the last ```t_meanSize``` values, in ![\mathcal{O}(1)](https://latex.codecogs.com/svg.latex?\mathcal{O}(1)) for each iteration, in order to display a smoother plot to the human eye!

### Final remarks

In order not to overload the interruption, the illuminance is written on the led in every iteration, which might lead to some noise. It was not implement a condition to check if the value on rhe led was already correct, because it has an higher implementation cost than a basic instruction (analogWrite).
