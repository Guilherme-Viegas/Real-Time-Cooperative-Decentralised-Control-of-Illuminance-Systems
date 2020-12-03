void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 57600 baud
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
 Serial.print(105);
 Serial.write("\n");
 delay(500);
}
