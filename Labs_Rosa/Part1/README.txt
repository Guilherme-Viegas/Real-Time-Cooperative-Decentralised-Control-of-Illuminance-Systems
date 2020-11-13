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
