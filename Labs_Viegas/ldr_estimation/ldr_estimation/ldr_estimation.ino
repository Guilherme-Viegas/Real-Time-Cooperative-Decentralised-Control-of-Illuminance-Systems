#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 

#define SIZE_DATA 500 // 5V / 0.01, Getting 500 different vals of LDR for 500 different inputs

void getMatrix();

float vo = 0.0;
float R2 = 0.0;
// float x[SIZE_DATA] = {0.0};
// float y[SIZE_DATA] = {0.0};

void setup() {

  Serial.begin(9600);

  pinMode(LED_PWM, OUTPUT);


}

void loop(){
  
  if(Serial.available()==1)
    getMatrix();
}

void getMatrix(){

  byte b = 0;
  for(int i=0; i<SIZE_DATA; i++){

    //b = analogRead(LED_PWM);
    analogWrite(LED_PWM, map(i, 0, 500, 0, 255));
    
    do{
      delay(1);
    }while(b==analogRead(LED_PWM));
    b = analogRead(LED_PWM);
    
    vo = analogRead(LDR_ANALOG) * VCC/MAX_ANALOG;
    
    R2 = (VCC-vo)*R1/vo;
    
    Serial.print(i*0.01);
    Serial.print('\t');
    Serial.println(R2);

    // x[i] = vo;
    // y[i] = R2;
    
  }

}
