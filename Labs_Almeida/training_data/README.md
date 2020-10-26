This folder contains files that can be use to compute the apropriate slope, tau, dead time of each LDR and Arduino, following the next steps:

1. Run [Arduino file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/training_data/training_data.ino), then press any key to start generate randomly LED's percentage of brightness;
2. Copy the data from Serial Monitor, and paste it into [Text file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/training_data/training_data.txt);
3. Run [Python file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/training_data/training_data.py), and finally copy the **(m,b)** value into the [main Arduino file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/session_1/session_1.ino).

1. 1. Run [Arduino file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/training_data/training_data.ino) and select wheares is needed to set up the slope (function: compute_m) or compute the tau( time to the response reach 63% of the final response) from a step up response (function: steps_up) or step down (function: steps_down).
2. Copy the data from Serial Monitor, and paste it into [Text file.txt](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/training_data/text_files); Each function will produce a response that, in order to the python file works properly, should be save with the following <name_file>.txt;
⋅⋅* compute_m() -> calibrated.txt or multiples.txt (if called for multiples values of **m**)
⋅⋅* setps_up() -> positive_step_reponse.txt
⋅⋅* setps_down() -> negative_step_reponse.txt
The number 1 is related to the big box, and number 2 related with the small one.

3. Run [Python file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/training_data/training_data.py) according:
⋅⋅* MULTIPLE            -> compute multiple slopes
⋅⋅* CALIBRATED          -> compute the gain for the system
⋅⋅* CALC_TAU_TEORICO    -> compute theorical tau
⋅⋅* IMAGE               -> shows all figures
⋅⋅* STEP_RESPONSE       -> computes the tau for both steps and subtracts the dead_time

Finally copy the **(m,b)** values if to adjusts the slope for the second case. If the goal is to compute the tau, the program fits a exponencial funciton: $f(x) = A*e^{Bx}+c$. Just copy it to the [main Arduino file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/session_1/session_1.ino).
