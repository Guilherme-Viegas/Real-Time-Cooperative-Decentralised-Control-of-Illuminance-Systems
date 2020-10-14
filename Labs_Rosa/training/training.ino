#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 9 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define MAX_DIGITAL 255 // maximum digital value 8 bits
#define VCC 5.0  // Power supply 

const int n_data = 500; // Number of values of LDR to this Number of inputs
void getMatrix();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_PWM, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()==1){
    getMatrix();
  }
  analogWrite(LED_PWM, 0); //turns off the LED
}

void getMatrix(){
  byte b = 0;
  int i = 0;
  int sensor_val = 0;
  float v_out = 0;
  for(i=0; i < n_data; i++){
    analogWrite(LED_PWM, i*VCC/MAX_DIGITAL); // volt to digital

    //read voltage at LDR
    sensor_val = analogRead(LDR_ANALOG);
    v_out = sensor_val*VCC/MAX_ANALOG;
    
    delay(10);
    
    float R2 = (VCC-v_out)*R1/v_out;
    Serial.print(i);
    Serial.print('\t');
    Serial.println(R2); 
  }
}
