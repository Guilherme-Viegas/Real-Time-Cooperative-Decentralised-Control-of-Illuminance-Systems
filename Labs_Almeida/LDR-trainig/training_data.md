This folder contains files that can be use to compute the apropriate slope of each LDR, following the next steps:

1. Run [Arduino file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/LDR-trainig/LDR-trainig.ino), then press any key to start generate randomly LED's percentage of brightness;
2. Copy the data from Serial Monitor, and paste it into [Text file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/LDR-trainig/training_data.txt);
3. Run [Python file](https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/LDR-trainig/training_data.py), and finally copy the **m** value into the [main Arduino file] (https://github.com/Guilherme-Viegas/SCTDR/blob/master/Labs_Almeida/session_1/session_1.ino).

Note: the b value is fixed in R=50KΩ.
