#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 

#define SIZE_DATA 1e4

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
  getMatrix();
  getDarkness();


}

void loop(){}

void getMatrix(){

  byte b = 0;
  for(int i=0; i<SIZE_DATA; i++){

    b = analogRead(LED_PWM);
    analogWrite(LED_PWM, random(0, 101));
    
    do{
      delay(1);
    }while(b==analogRead(LED_PWM));
    b = analogRead(LED_PWM);
    
    vo = analogRead(LDR_ANALOG) * VCC/MAX_ANALOG;
    
    R2 = (VCC-vo)*R1/vo;
    
    Serial.print(vo);
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
