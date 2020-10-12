// Global variables

// Unchangeable
#define COMMUNICATION 9600 // communication
#define VCC 5.0  // Power supply 
#define LED_PWM 3 // PWM pin
#define R1 1E4  // Resistor
#define LED_PWM 3 // PWM pin 3
#define LDR_ANALOG A0 // Analog pin 0
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define MAX_DIGITAL 255.0 // maximum digital value 8 bits

#define DEBUG 1

// Changeable
float ldr_status = VCC + 0.1; // different from any value that the next can be

// function
void led_performance();
void get_ldr_value();
float voltageToLux(float v0);

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
  
  float voltageOut = sensorValue*(VCC/MAX_ANALOG); // Map analog value to vcc scale
  float voltageLux = voltageToLux(voltageOut);
  float threshold = 0.03;
  
  if(abs(ldr_status - voltageLux) > threshold){ // print out the value you read:
    Serial.println("LUX value: " + String(voltageLux));
    ldr_status = voltageLux;
  }
}

float voltageToLux(float v0) {
  
  float m = -1.67398563022854; //slope
  float b = log10(50e3); // //Intersection = log10(60000)kOhm

  float function = (log10((VCC / v0) * R1 - R1) - b) / m;
  float lux = pow(10, function);
  return lux;
}
