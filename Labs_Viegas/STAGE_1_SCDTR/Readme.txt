***** Código escrito por Guilherme Viegas *****

Existem 3 partes nesta 1a parte do projecto...
...E uma pasta com os vários gráficos guardados...

Dentro da pasta "get_controller_params" encontram-se duas pastas ("lab1_gain" e "calibrate_lab2"), referentes aos dois 1os laboratórios!

Dentro da pasta lab1_gain está o código arduino para calcular o ganho linear G e para algumas funções de conversão de analógico para tensão e assim, existem também os ficheiros .txt que resultam dos dados que o lab1_gain imprime no serial e que foram depois usados no ficheiro training_data.py para calcular respetivas regressões. Esta 1ª pasta já tem algum código deprecado, pois estas funções do 1o lab são também implementadas no código principal, onde o ganho linear é calculado em tempo real pelo Arduino (visto ser diferente para cada teste) em vez de ser necessário correr o script em python.
O script python serve também para calcular os parâmetros m e b do meu circuito ldr em específico, através dos dados do ficheiro "multiple.txt". Para o m o script faz sucessivas regressões com ligeiras variações do valor de m e calcula a soma do erro quadrático...posteriormente observo qual o valor com menor erro e passa a ser esse o meu m.

A pasta "calibrate_lab2" trata do laboratório 2 e por isso da calibração dos parâmetros do meu ldr em específico, ou seja calcular as curvas de tau tanto para steps up como steps down e também o cálculo do delay time! Correndo o código arduino obtenho os dados que passo para os ficheiros .txt tanto do step down como up, e corro o script python descomentando, na parte final do código, aquilo que quero calcular. O script apenas lê os dados e faz um curve fit (ajuste de função aos dados) a uma função exponencial e obtenho assim os vários valores dos parâmetros a, b e c da função da forma y = ae^xb + c, tanto para up como para down. Para o delay time, faço para cada step no ficheiro de dados o tempo que ele demora até a a curva se iniciar, ou seja até ficar diferente de zero.

Os ficheiros do código arduino principal encontram-se na diretoria principal, tendo 2as classes: Pid que corresponde ao controlador Pid e suas funções e Local_controller que corresponde ao meu ldr em específico e que tem todas as funções que acabam por depender de parâmetros especificos do meu ldr! Assim, com classes melhor a leitura do código e posso adicionar mais circuitos ldr e controladores Pid caso necessário

Para correr, inicialmente existe um menu, que infelizmente não é apresentado pois as instruções para print no Serial ocupam demasiada memória no Arduino, importante para outras funções... Assim, passo a usar o protocolo e sugestão de flow de inputs de entrada - Recomendo abrir logo o plotter para depois observar gráficos:


/********* INPUT PROTOCOL ***********
* CREATE <ledPort> <ldrPort> <G> <m> <b> <tau_a_up> <tau_b_up> <tau_c_up> <tau_a_down> <tau_b_down> <tau_c_down> <dead_time>  //Called by controller::init() 
* READ //Reads ldr value [outputs in LUX]
* SET <pwm_val> //Sets led to pwm value [0, 255]
* OCCUPIED <ON/OFF> //Sets desk as occupied(20lux/215pwm) or unoccupied(5lux)
* FEEDFORWARD <ON/OFF> //Enable/Disable feedforward
* FEEDBACK <ON/OFF> //Enable/Disable feedback
* COMPUTE G //Compute linear gain and automatically saves it to controller properties
* SIMULATE <final_lux> <initial_lux> <initial_time> <time_we_want_to_compute_the_lux> 
* 
* ******************************
 
* CREATE 3 A0 0.093 -0.77 50000 29047.1 -0.224531 9560.57 15904.85 -0.16087 7552.6 369.1
*/

So once you upload code to arduino -> Open SerialPlotter -> input "CREATE 3 A0 0.093 -0.77 50000 29047.1 -0.224531 9560.57 15904.85 -0.16087 7552.6 369.1" -> Wait for 1/2 seconds -> input "COMPUTE G" -> wait a few seconds, you can see the linear regression in plotter and this saves automathically the G to local_controller -> input other things like "OCCUPIED ON" -> observe the curve on plotter, open and close windows, etc -> input some other value of pwm like "100" to do other kinds of steps

The red line is the lux goal to have in the box, the blue line is the lux read by ldr so the lux inside the box, and maybe I can have a 3rd line for checking the simulator too

Reaprei agora que escrevi metade em português e metade em Inglês... ahahah
