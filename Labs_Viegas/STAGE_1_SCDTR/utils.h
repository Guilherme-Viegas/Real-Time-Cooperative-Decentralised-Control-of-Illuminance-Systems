#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 
#define C1 0.000001
#define R1 10000 

float compute_theoric_Req(float);
float compute_theoric_tau(float);

float analogToVoltage(int analogInput);
float pwmToVoltage(float inputPwm);
int voltageToPwm(float inputVoltage);
float voltageToLux(float v0, float m, float b);
