#include "pid.h"
#define LED_PIN 3
#define LDR_ANALOG A0
#define MAX_SAVE 10.0
/*
  INTERFACE MENU:

  -S + <int> --- Set the Led with an input of <int> pwm
  -R --- Reads the value of Lux in the LDR

  -feedforward + ON (or) OFF --- Turns Feedforward Control On or OFF
  -feedback + ON (or) OFF -- Turns Feedback Control ON or OFF
  -occupied + <Value>--- Set higher reference for Controller
  -unoccupied <VAlue>--- Set lower reference for Controller
  -gain --- compute LDR new gain
  -simulateM --- simulate multiple steps
*/

void feedBack(float ref, int FF, int FB);
int antiWindup(int u, int u_sat, int ui_before, pid controller, int error);
int isValidNumber(String str);

boolean flag = true;
LDR simulator;
pid controller(1, 41, 31, 0, 0);

volatile int u = 0;
volatile int ui = 0;
volatile int ui_before = 0;
volatile int error = 0;
volatile float error_v = 0;
volatile int uff = 0;
volatile float y = 0;
volatile float yref = 0;
volatile float y_v = 0;
volatile float yref_v = 0;
volatile float  last[int(MAX_SAVE)];
volatile float usat = 0;
int STOP = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);
  pinMode(LED_PIN, OUTPUT);
  simulator.init(LED_PIN, LDR_ANALOG, 2, 0,-0.587, 5E4, 19051.974, -0.027324, 5912.019);
  
  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  // OCR1A = [16, 000, 000Hz / (prescaler * desired interrupt frequency)] - 1
  // OCR1A = [16M / (64 * 100)] - 1;
  TCNT1 = 0;   // preload timer
  OCR1A = 2499;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << CS12);    // 64 prescaler 
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11);
 
  TIMSK1 |= (1 << OCIE1A);   // enable timer overflow interrupt
  interrupts();          // enable all interrupts
}

ISR(TIMER1_COMPA_vect)        // interrupt service routine 
{ 
  STOP = 1;
}
void loop() {
  flag = true;
  while(flag){
    if(Serial.available()>0){
      String inBytes = Serial.readString();
      char *strings[5];
      int i = 0;
      char* token = strtok(inBytes.c_str(), " ");
      while(token != NULL){
        strings[i] = token;
        i++;
        token = strtok(NULL, " ");
      }
      if(strcmp(strings[0], "feedback") == 0){
        Serial.println("FeedBack");
        Serial.println("LUX PEDIDO: " + String(strings[1]));
        simulator.computeGain();
        feedBack(atof(strings[1]), 1, 1);
      }
      else if(strcmp(strings[0], "occupied") == 0){
        Serial.println("Occupied");
        simulator.computeGain();
        feedBack(50, 1, 1);
      }
      else if(strcmp(strings[0], "unoccupied") == 0){
        Serial.println("Unoccupied");
        simulator.computeGain();
        feedBack(5, 1, 1);
      }
      else if(strcmp(strings[0], "S") == 0){
        analogWrite(simulator.getLedPin(), atoi(strings[1]));
        Serial.println("SETTING LED: " + String(strings[1]) + " pwm");
      }
      else if(strcmp(strings[0], "R") == 0){
        float lux = readLux(simulator.getLDRpin(), simulator.getM());
        Serial.println("READING LUX: " + String(lux) + " lux");
      }
      else if(strcmp(strings[0], "OFF") == 0){
        analogWrite(simulator.getLedPin(), 0);
        Serial.println("LED TURNED OFF");
      }
      else if(strcmp(strings[0], "gain") == 0){
        Serial.println("Computing Gain...");
        float G = simulator.computeGain();
        Serial.println("NEW GAIN: " + String(G));
      }
      else{
        Serial.println("ERROR");
      }
      flag = false;
    }
  }
}

int antiWindup(int u, int u_sat, int ui_before, pid controller, int error){
  int ui = ui_before;
  if(u > 255){
    ui = 255 - uff-error*controller.getKp();
  }
  if(u < 0){
    ui = 0-uff-error*controller.getKp();
  }
  return ui;
}
void feedBack(float ref,int FF, int FB){
  if(FF){
    //Do things uff [pwm]
      uff = controller.ref2pwm(ref, simulator);
  }
  int counter = 0;
  float sum = 0;
  float ti = micros();
  Serial.println("y_REF\ty");
  interrupts();
  while(true){
    
    if(Serial.available()> 0){
      String string = Serial.readString();
      ref = string.toFloat();
      
      uff = 0;
      ti = micros();
      y_v = 0;
      yref_v = 0;
      error_v = 0;
      counter = 0;
      sum = 0;
      ui= 0;
      ui_before = 0;
      u = 0;
      if(FF){
        //Do things uff [pwm]
        uff = controller.ref2pwm(ref, simulator);
      }
    }
    
    //feedforward ON
    if(STOP){
      //reset counter
      if(counter == MAX_SAVE){counter=0;}
      //reset input
      u = 0;
      y_v = 0;
      for(int i=0; i < 5; i++){
        y_v += analogRead(simulator.getLDRpin())*VCC/MAX_ANALOG;
      }
      y_v = y_v/5.0;
      y = voltage2Lux(y_v, simulator.getM());
      
      last[counter] = y;
      counter++;
      
      yref_v = simulator.simulate(ref, ti);
      yref = voltage2Lux(yref_v, simulator.getM());
      error_v = yref_v-y_v;
      
      for(int i = 0; i < MAX_SAVE; i++){
        sum += last[i];
      }
      //deadzone
      if(abs(error_v) <= VCC/MAX_DIGITAL){
        error_v = 0;
      }
      ui = ui_before + controller.getT()*controller.getKi()*error_v;
      ui_before = ui;

      if(FB){
        u = controller.getKp()*error_v + ui;
      }
      //add uff
      u += uff;
      usat = controller.saturation(u);
      //AntiWindup
      ui_before = antiWindup(u, usat, ui_before, controller, error_v);
      
      //write in the LED
      analogWrite(simulator.getLedPin(), usat);
      Serial.print(yref);
      Serial.print('\t');
      Serial.println(sum/MAX_SAVE);
      sum = 0;
      STOP = 0;
    }
    
  }
  noInterrupts();
}
