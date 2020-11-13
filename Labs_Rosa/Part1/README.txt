Project:
---File pid.cpp/pid.h:
Defines the controller class and parameters such as gains and sample time, and other useful functions

---File LDR_class.cpp/LDR_class.h
Defines LDR class to simulate:
* Gain: simulates the gain that translates pwm to lux(lux = G*pwm);
* lux: simulates lux in the box based on voltage read in the Arduino.


---Function Feedback(int ref, int FF, int FB):

-To turn ON and OFF the feedforward component, we need to put 1 or 0 in the argument FF;
-The same to the feedback component (FB);
-The ref is the reference in lux that we want the controller to achieve.
We can change the reference value in the middle of the simulation!
