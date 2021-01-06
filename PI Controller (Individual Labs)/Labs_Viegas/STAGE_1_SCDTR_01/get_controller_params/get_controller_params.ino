#include <math.h>


#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 

#define m -0.77
#define b 50000
#define C1 0.000001
#define R1 10000

String inBytes;
int pwmVal = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(LED_PWM, OUTPUT);
}

void loop() {
  readLdr();
  if(Serial.available() > 0) {
      inBytes = Serial.readString();
      Serial.print("I received: ");
      Serial.println(inBytes);
      analogWrite(LED_PWM, map(inBytes.toInt(), 0, 100, 0, 255));
    }
    delay(10);
}

void readLdr() {
    // read the input on analog pin 0:
    int sensorValue = analogRead(LDR_ANALOG);
    // Convert the analog reading (which goes from 0 - 1023) to a percentage (0-100)
    float voltageOut = sensorValue * (5.0 / 1023.0);
    // print out the value you read:
    Serial.print("LUX VALUE: ");
    Serial.println(String(voltageToLux(voltageOut)));
  }

float voltageToLux(float v0) {
    float vcc = 5.0;
    float R1 = 10000.0;
    float m = -1.7780;//slope
    float b = 4.778; //Intersection = log10(60000)kOhm

    float function = (log10((vcc/v0)*R1 - R1) - b) / m;
    float lux = pow(10, function);
    return lux;
  }
