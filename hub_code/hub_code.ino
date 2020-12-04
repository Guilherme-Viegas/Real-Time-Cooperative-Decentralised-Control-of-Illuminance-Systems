char welcome[4];

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 57600 baud
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.flush();
}

void loop() {
  /*for(int i = 0; i < 127; i++)
  {
    Serial.write(i);
    delay(500);
  }*/

  if(Serial.available())
  {
    Serial.readBytes(welcome, 3);
    
    if( welcome[0] == 'R' && welcome[1] == 'P' && welcome[2] == 'i' )
    {
      Serial.write("Arduino");
      Serial.write('\n'
      );
    }
    
  }
}

// screen /dev/tty.usbmodem14601 baud_rate
