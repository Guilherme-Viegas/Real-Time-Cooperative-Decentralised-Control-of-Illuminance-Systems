void setup() {
  Serial.begin(2000000); // opens serial port, sets data rate to 57600 baud
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}
 
void loop() {
 Serial.write("hi");
 delay(500);
 Serial.write("\n");
 delay(500);
}
