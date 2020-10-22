#define LDR_ANALOG A0 // Analog pin 0
#define LED_PWM 3 // PWM pin 3
#define MAX_ANALOG 1023.0 // maximum analog value 10 bits
#define VCC 5.0  // Power supply 

#define G 0.05353 //x(t) = G * u(t) done in python with SSE=1220
#define m -0.77
#define b 50000
#define C1 0.000001
#define R1 10000

const byte mask= B11111000; // mask bits that are not prescale
int prescale = 2; //second fastest possible (1 for fastest)

volatile bool flag;
volatile int counter = 0;
volatile int value[71];
volatile long val_time[71];

float vo = 0.0;

//Interrupt for analog reading 
ISR(TIMER1_COMPA_vect){
  flag = 1; //notify main loop
  value[counter] = analogRead(LDR_ANALOG);
  val_time[counter] = micros() - val_time[0];
  counter++;
}

//(LUX, R2 em Ohm)
float voltageToLux(float v0) {
    float function = (log10((VCC/v0)*R1 - R1) - log10(b)) / m;
    float lux = pow(10, function);
    return lux;
  }

//When computing the tau, at 63% of step size, we need to know the expected 100% X [LUX] value that should appear 
float getTheoricX(float x) {
  float function1 = m*log10(x) + log10(b);
  float function2 = pow(10, function1);
  return (VCC*R1)/function2;
}

//Value of R2 depends on the LUX x, which is G*u(t), or G*PWM
//Returns R2 in Ohms (e.i 13000ohm)
float getR2(float inputPwm) {
  float x = inputPwm * G;
  float function = m*(log10(x)) + log10(b);
  return pow(10, function);
}

//Value of the R equivelent depends on R2 which depends on the LUX, or G*PWM
float getReq(float r2) {
  return (R1 * r2)/(R1 + r2);
}

//Theoretic value of tau, it depends on R_eq and C1, so depends on R_2 which depends on LUX or G*PWM
//Returns tau in seconds (e.i 0.0003s)
float getTau(float r_eq) {
  return r_eq * C1;
}

int voltageToPwm(float inputVoltage) {
    return (int)(255.0 * inputVoltage / VCC);
}

float pwmToVoltage(float inputPwm) {
    return ((float)inputPwm) * VCC / 255.0;
}

float analogToVoltage(int analogInput) {
    return ((float)analogInput) * (VCC / 1023.0);
}

float voltageToAnalog(float analogInput) {
  return (analogInput * 1023.0) / VCC;
}

void computeTau(int inputPort, int outputPort) {
  
  for(int i=0; i<256; i+=5){
    counter=0;
    //Serial.print("Expected Theorical Max: ");
    //Serial.println(voltageToAnalog(getTheoricX(((float)(i))*G)));
    analogWrite(outputPort, i);
    unsigned long startTime = micros();
    value[counter] = -1;
    val_time[counter] = micros();
    counter++;

    sei(); //enable interrupt
    while(micros() - startTime < 350000) { //Teacher instructed for 0.5s but it's to much, with 0.35 I have less points on the stable part so that I can have more on the unstable
      if(flag) { //Post-interrupt related Operations
        flag = 0;
      }
    }
    cli(); //disable interrupt
    //TCCR1A = 0; // clear register
    //TCCR1B = 0; // clear register
    TCNT1 = 0; //reset counter

    analogWrite(outputPort, 0); //To make a step that always returns to zero...To get better values of tau
    Serial.print("PWM: ");
    Serial.println(i);
    for(int i=0; i<71; i++) {
      Serial.print(value[i]);
      Serial.print(" ");
      Serial.println(val_time[i]);
    }
    Serial.println();
    delay(50);
    
  }
  
}


void setup() {
  Serial.begin(9600);
  pinMode(LED_PWM, OUTPUT);
  TCCR2B = (TCCR2B & mask) | prescale;  //Raise Timer2 (pwm of port 3) for ldr not sensing led flicker

  //Fo preparing the Sampling Period Timer Interrupt
  //I STILL DONT KNOW MUCH ABOUT WHATS HAPPENING HERE
  //...ok now I know a bit more...On each 0.5s step I want samples of 0.005s = 200Hz (500 samples)(I tried with 50samples but then the time would be 0.01s and with that I couldn't pick the first values)
  cli(); //disable interrupts
  TCCR1A = 0; // clear register
  TCCR1B = 0; // clear register
  TCNT1 = 0; //reset counter
  // OCR1A = desired_period/clock_period – 1 //This -1 is needed for Arduino specifics
  // = clock_freq/desired_freq - 1
  // = (16*10^6 / 51200) - 1 / (200*1Hz) – 1  //Because prescale is 256, each second or 1Hz is 256, so 200Hz is 200*256
  // = 311.5
  OCR1A = 312; //must be <65536
  TCCR1B |= (1 << WGM12); //CTC On
  // Set prescaler for 256 -
  TCCR1B |= (1<<CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable interrupts
}

void loop() {
  if(Serial.available() > 0) {
    Serial.println("*** WAIT ***");

    //Now we want to compute the tau characteristic for different pwm input step values
    //Save it as tau.txt and run tau_plot.py
    computeTau(LDR_ANALOG, LED_PWM);

    while(1) {}
  }
}
