#include <math.h>

String inBytes;
int pwmVal = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(10, OUTPUT);
}

void loop() {
  readLdr();
  if(Serial.available() > 0) {
      inBytes = Serial.readString();
      Serial.print("I received: "); Serial.println(inBytes);
      analogWrite(10, map(inBytes.toInt(), 0, 100, 0, 255));
      delay(2);
    }
}

void readLdr() {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // Convert the analog reading (which goes from 0 - 1023) to a percentage (0-100)
    float voltage = sensorValue * (100.0 / 1023.0);
    // print out the value you read:
    Serial.println(String(voltage));
  }

int voltageToLux(int v0) {
    int vcc = 5;
    int R1 = 10000;
    int m = -1;//slope
    int b = 32; //Intersection
  }
