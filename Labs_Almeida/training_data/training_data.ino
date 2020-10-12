#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 
#define MAX_DIGITAL 255.0 // maximum digital value 8 bits
#define MAX_LED 100.0 // maximum digital value 8 bits

#define SIZE_DATA 1e4

#define TRAINING 0

void getMatrix();
void getDarkness();

float vo = 0.0;
float R2 = 0.0;
// float x[SIZE_DATA] = {0.0};
// float y[SIZE_DATA] = {0.0};
void setup() {

  Serial.begin(9600);

  pinMode(LED_PWM, OUTPUT);
  delay(3e3);

  if(TRAINING){
    getMatrix();
    getDarkness();
    }
  else{
    test_mb();
    }

}

void loop(){}

void getMatrix(){

  byte b = 0;

  byte pwm = 0;
  byte flag = 1;
  
  for(int i=0; i<SIZE_DATA; i++){

    if(pwm == 255){ flag = -1; }
    else if(pwm == 1){ flag = 1; }
    
    pwm += flag;
    
    analogWrite(LED_PWM, pwm*MAX_LED/MAX_DIGITAL);
    delay(10);
    
    vo = analogRead(LDR_ANALOG) * VCC/MAX_ANALOG;
    
    R2 = (VCC-vo)*R1/vo;
    // pwm = pwm*100.0/255.0
    
    
    Serial.print(pwm);
    Serial.print('\t');
    Serial.println(R2);

    // x[i] = vo;
    // y[i] = R2;
  }
}

void getDarkness(){
  
    byte b = 1;
    analogWrite(LED_PWM, 0);

    delay(1e3); //wait until the darkness

    vo = analogRead(LDR_ANALOG) * VCC/MAX_ANALOG;
    R2 = (VCC-vo)*R1/vo;
    Serial.print(R2);
  
  }

float voltageToLux(float v0) {
  
  float m = -0.8162525263647151;
  float b = 5.800482334196736;

  float function = (log10((VCC / v0) * R1 - R1) - b) / m;
  float lux = pow(10, function);
  return lux;
}

void test_mb(){ 

  byte pwm = 0;
  byte flag = 1;
  float lux = 0.0;
  float voltageOut = 0.0;
  
  int cicles = (MAX_DIGITAL*2-1)*5;
  
  for(int i=0; i<cicles; i++){

    if(pwm == 255){ flag = -1; }
    else if(pwm == 1){ flag = 1; }

    pwm += flag;
    
    analogWrite(LED_PWM, pwm);
    delay(10);
  
    voltageOut = analogRead(LDR_ANALOG)*(VCC/MAX_ANALOG);
  
    lux = voltageToLux(voltageOut);
  
    Serial.print(pwm);
    Serial.print('\t');
    Serial.println(lux);
   }
}
