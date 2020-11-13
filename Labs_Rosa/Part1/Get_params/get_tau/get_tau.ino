#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 

volatile int counter;
volatile int pwm = 0;
volatile int output[71];
volatile long time_vector[71];
const byte mask= B11111000; // mask bits that are not prescale
void setup()
{
  Serial.begin(9600);
  TCCR2B = (TCCR2B & mask) | 1;
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set compareMatchReg to the correct value for our interrupt interval
  // compareMatchReg = [16, 000, 000Hz / (prescaler * desired interrupt frequency)] - 1
  
  /* E.g. 1Hz with 1024 Pre-Scaler:
    compareMatchReg = [16, 000, 000 / (prescaler * 1)] - 1
    compareMatchReg = [16, 000, 000 / (1024 * 1)] - 1 = 15624
  
      As this is > 256 Timer 1 Must be used for this..
  */
  TCNT1 = 0;   // preload timer
  OCR1A = 1249;

  /*
  Prescaler:
    (timer speed(Hz)) = (Arduino clock speed(16MHz)) / prescaler
      So 1Hz = 16000000 / 1 --> Prescaler: 1
      Prescaler can equal the below values and needs the relevant bits setting
      1    [CS10]
      8    [CS11]
      64   [CS11 + CS10]
      256  [CS12]
      1024 [CS12 + CS10]
  */
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << CS12);    // 256 prescaler 
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11);
 
  TIMSK1 |= (1 << OCIE1A);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}
int flag = 0;
ISR(TIMER1_COMPA_vect)        // interrupt service routine 
{ 
  flag = 1;
}

void loop(){
  // stepDown
  if(pwm < 256){
    //turn on LED
    analogWrite(LED_PWM, 0);
    delay(500);
    counter = 0;
    analogWrite(LED_PWM, pwm);
    unsigned long startTime = micros();
    output[0] = -1;
    time_vector[0] = micros();
    counter++;

    interrupts();
    while(micros()-startTime < 350000){
      if(flag){
        output[counter] = analogRead(LDR_ANALOG);
        time_vector[counter] = micros()-time_vector[0];
        counter++;
        flag = 0;
      }
    }
    noInterrupts();

    
    //print on Serial Monitor
    Serial.print("PWM: ");
    Serial.println(pwm);
    output[0]=0;
    time_vector[0]=0;
    for(int i=1; i<71; i++) {
      Serial.print(output[i]);
      Serial.print(" ");
      Serial.println(time_vector[i]);
      output[i]=0;
      time_vector[i]=0;
    }
    pwm+=5;
  }
}
