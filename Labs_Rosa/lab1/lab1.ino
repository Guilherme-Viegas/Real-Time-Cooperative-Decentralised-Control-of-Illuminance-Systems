#define VCC 5  // Voltage 
#define LED_PIN 9// PWM pin
#define R1 1E4  // Resistor
#define LDR_ANALOG A0 // Analog pin 0
#define MAX_ANALOG 1023 // maximum analog value 10 bits
#define MAX_DIGITAL 255 // maximum digital value 8 bits

String incomingByte;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  readLDR();
  // enviar resposta apenas quando receber dados:
  if (Serial.available() > 0) {
    // lê o dado recebido:
    incomingByte = Serial.readString();

    // responder o que foi recebido:
    Serial.print("Eu recebi: ");
    Serial.println(incomingByte);
    analogWrite(LED_PIN, map(incomingByte.toInt(), 0, 100, 0, MAX_DIGITAL));
  }
  delay(100);
}

void readLDR() {
  int sensor_val = analogRead(LDR_ANALOG);
  float voltageOUT = map(sensor_val, 0, MAX_ANALOG, 0, VCC);
  //convert voltage to LUX
  Serial.print("LUX: ");
  Serial.println(voltage2LUX(voltageOUT));
}

float voltage2LUX(float voltageOUT){
  //m and b were computed from datasheet characteristic
  float m = -1.7780;//slope
  float b = 4.778; //Intersection = log10(60000)kOhm

  float lux = pow(10, (log10((VCC/voltageOUT)*R1 - R1) - b) / m);
  return lux;
}
