#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define MAX_DIGITAL 255 // maximum digital value 8 bits
#define VCC 5.0  // Power supply 

const int n_data = 500; // Number of values of LDR to this Number of inputs
void getMatrix();
float voltage2LUX(float voltageOUT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_PWM, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()==1){
    //getMatrix();
    analogWrite(LED_PWM, 0); //turns off the LED
    delay(100);
    Serial.println(analogRead(A0)*VCC/MAX_ANALOG);
    Serial.println((analogRead(A0)*VCC/MAX_ANALOG)*MAX_DIGITAL/VCC);
  }
  
}

void getMatrix(){
  byte b = 0;
  int i = 0;
  int sensor_val = 0;
  float v_out = 0;
  int pwm = 0;
  int flag = 1;
  for(i=0; i < 256*4; i++){
    analogWrite(LED_PWM, pwm); // volt to digital
    //read voltage at LDR
    delay(100);
    sensor_val = analogRead(LDR_ANALOG);
    v_out = sensor_val*VCC/MAX_ANALOG;
    
    float R2 = voltage2LUX(v_out);
    Serial.print(pwm);
    Serial.print('\t');
    Serial.println(R2); 
    if(pwm==255){flag=-1;}
    else if(pwm==0){flag=1;}
    pwm += flag;
  }
}

float voltage2LUX(float voltageOUT){
  //m and b were computed from datasheet characteristic
  float m = -0.587;//slope
  float b = log10(2E4); //Intersection = log10(60000)kOhm

  float lux = pow(10, (log10((VCC/voltageOUT)*R1 - R1) - b) / m);
  return lux;
}
