# Calibration files

This folder contains files that can be used to compute both taus and dead time of each LDR and Arduino, following the next steps:

## Files description

1. Run [ldr_fit.ino](./ldr_fit.ino) to compute both taus (time it take for the response to reach 63% of the final response) from a step up response (function: steps_up) or step down (function: steps_down). The program fits a exponential function: ![f(t) = a*e^{bt}+c ](https://latex.codecogs.com/svg.latex?f(t)%20=%20a*e^{bt}+c).

2. Copy the data from Serial Monitor, and paste it into [Text file.txt](./text_files); Each function will produce a response that, in order for the python file to work properly, should be save with the following extention <name_file>.txt;
  * ldr.setGain() -> calibrated.txt or multiples.txt (if called for multiples values of **m**)
  * steps_up() -> positive_step_response.txt
  * steps_down() -> negative_step_response.txt

Note that the office(box) should be isolated because it waits until it has read the extreme values in analog pin.

3. Run [Python file](./ldr_fit.py) according:
  * MULTIPLE            -> computes multiple slopes
  * CALIBRATED          -> computes the gain for the system
  * CALC_TAU_TEORICO    -> computes theoretical tau
  * IMAGE               -> shows all figures
  * STEP_RESPONSE       -> computes the tau for both steps and subtracts the dead_time

The [ldr_fit.ino](./ldr_fit.ino) is the main code, which calls the functions present in [ldr_fit.cpp](./ldr_fit.cpp) and its respective header: [ldr_fit.h](./ldr_fit.h).

4. Containing of each file:
  * [calibrated.txt](./text_files/calibrated.txt) - slope for ```m = -0.718``` [PWM/Lux]
  * [multiples.txt](./text_files/multiples.txt) - multiple tests for m : ```for(i=0; i<19; i++){ m = -0.71 -0.001*i }```
  * [multiples1.txt](./text_files/multiples1.txt) - multiple tests for m : ```for(i=0; i<21; i++){ m = -0.6 -0.01*i }```
  * [positive_step_response.txt](./text_files/positive_step_response.txt) - voltage values for tau computation for step up: {0...1...0...2...(...)...0...255}
  * [negative_step_response.txt](./text_files/negative_step_response.txt) - voltage values for tau computation for step down: {255...0...255...1...(...)...255...254}
