This folder contains files that can be use to compute the apropriate slope, tau, dead time of each LDR and Arduino, following the next steps:


1. Run [ldr_fit.ino](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/ldr_fit/ldr_fit.ino) to compute both taus (time to the response reach 63% of the final response) from a step up response (function: steps_up) or step down (function: steps_down). The program fits a exponencial funciton: $f(x) = A*e^{Bx}+c$. 

2. Copy the data from Serial Monitor, and paste it into [Text file.txt](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/ldr_fit/text_files); Each function will produces a response that, in order to the python file works properly, should be save with the following <name_file>.txt;
  * ldr.setGain() -> calibrated.txt or multiples.txt (if called for multiples values of **m**)
  * setps_up() -> positive_step_reponse.txt
  * setps_down() -> negative_step_reponse.txt

3. Run [Python file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/ldr_fit/ldr_fit.py) according:
  * MULTIPLE            -> compute multiple slopes
  * CALIBRATED          -> compute the gain for the system
  * CALC_TAU_TEORICO    -> compute theorical tau
  * IMAGE               -> shows all figures
  * STEP_RESPONSE       -> computes the tau for both steps and subtracts the dead_time

The [ldr_fit.ino](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/ldr_fit/ldr_fit.ino) is the main cod, which call the functions present in [ldr_fit.cpp](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/ldr_fit/ldr_fit.cpp) and its respectively header: [ldr_fit.h](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/ldr_fit/ldr_fit.cpp)
