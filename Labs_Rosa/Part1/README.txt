**Project**
GETTING PARAMETERS FOR THE SIMULATOR:
----CALIBRATE M
--Run calibrate_m.ino file in the arduino;
--Open the Serial Monitor and copy the values to a file .txt;
--Open the .py file training_data and change the global variable "filename" to the new file .txt;
--Run the .py file and see if the graph seems like a straight line;
--If it's not straight line, open the calibrate_m.ino file and change m in the function voltage2Lux and proceed the same way until it's straight and m calibrated.

----GETTING TIME CONSTANT
--Run get_tau.ino file in the arduino;
--Open the Serial Monitor and copy the values to a file .txt;
--Open the .py file tau_plot and call the function load_variables with the name of the .txt file as argument;
--Save the image and the parameters of the time constant in  terms of pwm input, printed in the console;

---File utils.cpp/utils.h
Defines some useful functions, like voltage to lux convertion and some global constants, such as Maximum analog and digital value and the Vcc value

---File pid.cpp/pid.h:
Defines the controller class and parameters such as gains and sample time, and other useful functions

---File LDR_class.cpp/LDR_class.h
Defines LDR class to simulate:
* Gain: simulates the gain that translates pwm to lux(lux = G*pwm);
* lux: simulates lux in the box based on voltage read in the Arduino.


---Part1.ino
-Run this file and Type in the Serial monitor:
- S: and a int <value> to set the Led to that input value in pwm
- R: read the lux in the Box
-gain: to compute the gain of the system, in other words, Lux = Gain*pwm
(WARNING: Don't forget to open the Serial plotter and write the following to see the plot)
- feedback <value>: to control the luminosity of the led to the input value (reference) in Lux (See the function feedback description below!)
- occupied: do the same as the feedback funcionality, but the upper reference is not chosen
- unoccupied: do the same as the feedback funcionality, but the lower reference is not chosen


---Function Feedback(int ref, int FF, int FB):
-To turn ON and OFF the feedforward component, we need to put 1 or 0 in the argument FF;
-The same to the feedback component (FB);
-The ref is the reference in lux that we want the controller to achieve.
We can change the reference value in the middle of the simulation by typing the value in the Serial monitor of the arduino!
