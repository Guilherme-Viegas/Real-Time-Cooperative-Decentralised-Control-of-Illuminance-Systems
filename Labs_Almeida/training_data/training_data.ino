#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 
#define MAX_DIGITAL 255.0 // maximum digital value 8 bits
#define MAX_LED 100.0 // maximum digital value 8 bits


float vo = 0.0;
float R2 = 0.0;
// float x[SIZE_DATA] = {0.0};
// float y[SIZE_DATA] = {0.0};
void setup() {

  Serial.begin(9600);

  pinMode(LED_PWM, OUTPUT);
  delay(3e3);

  //for(int i=0; i<21; i++){
  //  test_cont(-0.66-i*0.001);
  //  Serial.println("STEP");
  //}

  test_cont(-0.672);
  //test_steps();

  analogWrite(LED_PWM, 0);
}

void loop(){}

float voltageToLux(float v0, float m) {
  
  float b = log10(5E4);

  float function = (log10((VCC / v0) * R1 - R1) - b) / m;
  float lux = pow(10, function);
  return lux;
}

void test_cont(float m){ 

  byte pwm = 0;
  float flag = 1;
  float lux = 0.0;
  float voltageOut = 0.0;
  
  int cicles = 2000;
  
  for(int i=0; i<cicles; i++){

    if(pwm == 255){ flag = -1; }
    else if(pwm == 0){ flag = 1; }

    pwm += flag;
    
    analogWrite(LED_PWM, pwm);
    delay(20);
  
    voltageOut = analogRead(LDR_ANALOG)*(VCC/MAX_ANALOG);
  
    lux = voltageToLux(voltageOut,m);
  
    Serial.print(pwm);
    Serial.print('\t');
    Serial.println(lux);
   }
}

void test_steps(float m){
  
  float pwm = 0;
  float flag = 1;
  float lux = 0.0;
  float voltageOut = 0.0;
  byte w = 0.0;
  
  int cicles = 2000;
  
  for(int i=0; i<cicles; i++){

    if(pwm == 255){ flag = -1; }
    else if(pwm == 0){ flag = 1; }

    if(i%2 == 0){ //even number
      w = pwm;
      pwm += flag;
    }else{
      w = 0.5*(1 - flag)*255;
    }
    
    analogWrite(LED_PWM, w);
    delay(20);
  
    voltageOut = analogRead(LDR_ANALOG)*(VCC/MAX_ANALOG);
  
    lux = voltageToLux(voltageOut,m);
  
    Serial.print(w);
    Serial.print('\t');
    Serial.println(lux);
   }
  }
