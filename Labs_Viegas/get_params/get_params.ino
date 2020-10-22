#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 

float lin_Gain = -0.09967; //x(t) = G * u(t) done in python with SSE=1220

//(LUX, R2 em Ohm)
float voltageToLux(float v0, float m, float b) {
    float vcc = 5.0;
    float R1 = 10000.0;

    float function = (log10((vcc/v0)*R1 - R1) - log10(b)) / m;
    float lux = pow(10, function);
    return lux;
  }


//Start every experiment by calling computeGain(), Gain depends on a lot of factors
//Puts an input x and reads by LDR analog read the value(converts analog to LUX)
//Gain in [LUX / V]
void computeGain(int inputPort, int outputPort) {
    float readVals[256] = {0};  //52 because is 255 / 5, so 52 samples
    int read = -1;

    for(int i=0; i <= 255; i++) { //Input PWM, x
        analogWrite(outputPort, i);
        delay(20); //Wait for ldr to stabilize
        read = analogRead(inputPort);
        readVals[i] = voltageToLux(analogToVoltage(read, 5.0), -0.9583, 50000); // m and b used from rect on datasheet
        Serial.print(i);
        Serial.print(' ');
        Serial.println(readVals[i]);
    }
    analogWrite(outputPort, 0);
    delay(20);
}

void sloping(int inputPort, int outputPort)
{
    float readVals[256] = {0}; //52 because is 255 / 5, so 52 samples
    int read = -1;
    int turnFlag = 1;
    for (float j=-0.65; j>=-0.8; j-=0.01)
    {
      
        for (int i = 0; i <= 255; i++)
        { //Input PWM, x
            analogWrite(outputPort, i);
            delay(20); //Wait for ldr to stabilize
            read = analogRead(inputPort);
            readVals[i] = voltageToLux(analogToVoltage(read, 5.0), j, 50000); // m from trial and error, and b 50000 to be equal to Rosa and Almeida
            Serial.print(i);
            Serial.print(' ');
            Serial.println(readVals[i]);
        }
        analogWrite(outputPort, 0);
        delay(20);
        Serial.println("STEP");
    }
}

int voltageToPwm(float inputVoltage, float maxVoltage) {
    return (int)(255.0 * inputVoltage / maxVoltage);
}

float pwmToVoltage(float inputPwm, float maxVoltage) {
    return ((float)inputPwm) * maxVoltage / 255.0;
}

float analogToVoltage(int analogInput, float maxVoltage) {
    return ((float)analogInput) * (maxVoltage / 1023.0);
}

void computeTau(int inputPort, int outputPort) {
  
}


void setup() {
  Serial.begin(9600);
  pinMode(LED_PWM, OUTPUT);
}

void loop() {
  if(Serial.available() > 0) {
    Serial.println("*** WAIT *** - Computing the gain G...");

    //First compute gain G, uses m and b taken by drawing an aproximmate rect line on ldr datasheet
    //Saves the values to calibrated.txt, run training_data.py, save the SSE to compare after
    computeGain(LDR_ANALOG, LED_PWM);

    //Now we want to calibrate the ldr by changing the value of m
    //Save the values gotten to multiple.txt, run training_data.py, pick the m that gives the least SSE
    //sloping(LDR_ANALOG, LED_PWM);

    while(1) {}
  }
}
