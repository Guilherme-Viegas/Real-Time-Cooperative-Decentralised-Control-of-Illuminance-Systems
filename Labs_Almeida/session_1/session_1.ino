// Global variables

// Unchangeable
#define COMMUNICATION 9600 // communication
#define VCC 5  // Voltage 
#define LED_PWM 3 // PWM pin
#define R1 1E4  // Resistor
#define LED_PWM 3 // PWM pin 3
#define LDR_ANALOG A0 // Analog pin 0
#define MAX_ANALOG 1023 // maximum analog value 10 bits
#define MAX_DIGITAL 255 // maximum digital value 8 bits

#define DEBUG 1

// Changeable
byte ldr_status = VCC + 0.1; // different from any value that the next can be

void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(COMMUNICATION);
  pinMode(LED_PWM, OUTPUT);
}

void loop() {

  if (Serial.available()) { // anytime there is an input
    led_performance();  // read input value and ajust the led brightness
  }

  get_ldr_value();  // read ldr value

}

void led_performance() {

  String work_percentage = Serial.readString();  // read input at PWM pin 'led_pin'
  analogWrite(LED_PWM, map(work_percentage.toFloat(), 0, 100, 0, MAX_DIGITAL)); // changes the led work in %, non numeric characters is treated as zero

  if(DEBUG){
    work_percentage.replace('\n', '%');
    Serial.println ("The led is working at: " + work_percentage); // write input in the Serial
  }

  ldr_status = VCC + 0.1; // force to update the state, by manually giving a impossible number
}

void get_ldr_value() {
  
  unsigned short sensorValue = analogRead(LDR_ANALOG); // Read brightness
  
  float voltageOut = map( sensorValue, 0, (float)MAX_ANALOG, 0, (float)VCC); // Map analog value to vcc scale
  
  if(ldr_status != voltageOut) // print out the value you read:
    Serial.println("LUX value: " + String(voltageToLux(voltageOut)));

    ldr_status = voltageOut; 
}

float voltageToLux(float v0) {
  
  float m = -1.7780; //slope
  float b = 4.778; // Intersection = log10(60000)kOhm

  float function = (log10((VCC / v0) * R1 - R1) - b) / m;
  float lux = pow(10, function);
  return lux;
}
