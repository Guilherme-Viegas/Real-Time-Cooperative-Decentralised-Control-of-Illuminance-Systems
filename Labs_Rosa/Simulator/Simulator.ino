#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define MAX_DIGITAL 255 // maximum digital value 8 bits
#define VCC 5.0  // Power supply

//functions
float voltage2LUX(float voltageOUT, float m);
float computeGain(float m);

//global variables
const float m = -0.587; //slope calibrated
float G = 0; //Gain Lux = G*u(t)

//converts voltage to LUX
float voltage2LUX(float voltageOUT, float m){
  float b = 5E4; //Intersection = log10(60000)kOhm
  float lux = pow(10, (log10((VCC/voltageOUT)*R1 - R1) - log10(b)) / m);
  return lux;
}
//compute the Gain G always
float computeGain(float m){
  int i = 0;
  int n_cycles = 4;
  int flag = 1;
  int pwm = 0;
  float m_sum = 0;
  float b_sum = 0;
  int m_num = 0;
  int b_num = 0;
  for(i=0; i < 256*n_cycles; i++){
    analogWrite(LED_PWM, pwm);
    delay(10);
    //read voltage at LDR
    float v_out = analogRead(LDR_ANALOG)*VCC/MAX_ANALOG;
    float lux = voltage2LUX(v_out, m);
    if(pwm==255){
      flag=-1;
    }
    else if(pwm==0){
      flag=1;
    }
    if(pwm!=0){
      m_sum += lux/float(pwm);
      m_num++;//number of different m's
    }
    else{
      b_sum += lux;
      b_num++;//number of b's
    }
    pwm += flag;
  }
  G = m_sum/float(m_num); //Gain lux = G*pwm
  float b = b_sum/float(b_num);
  return G;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_PWM, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()==1){
    Serial.println("**BEGIN**");
    G = computeGain(m);
    Serial.println(G);
  }
}
