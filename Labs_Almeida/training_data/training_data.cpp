#include "training_data.h"

volatile unsigned short analog_value;
volatile boolean flag_itr = false;

float voltage_to_lux(float v0, float m){
  
  float b = log10(5E4); // guess value for the resistor during the dark

  float function = (log10((VCC / v0) * R1 - R1) - b) / m; // relation between lux and voltage read
  
  float lux = pow(10, function);  // log scale
  
  return lux;
}

void compute_m(float m, boolean G ){ 

  byte pwm = 0; // pwm to be written in led
  float flag = 1; // flag is 1 if the direction is up and 0 if it is down
  float lux = 0.0;  // lux computed 
  float voltageOut = 0.0; // voltage read in analog pin

  byte cicle_times = 1; // number of times the mountain is done
  unsigned short cicles = cicle_times*510; // number of instants per mountain
  float b_mean = 0.0; // mean of b
  float m_mean = 0.0; // mean of m
  unsigned short len_without_b = cicles - 1; // total times pwm is not 0
  
  for(int i=0; i<cicles; i++){

    analogWrite(LED_PWM, pwm); // sets the pwm 
    delay(50);
 
    voltageOut = analogRead(LDR_ANALOG)*(VCC/MAX_ANALOG); // read V0
  
    lux = voltage_to_lux(voltageOut,m); // compute the lux

    // wites in Serial the data to compute tau in the python file
    Serial.print(pwm);    
    Serial.print('\t');
    Serial.println(lux);

    if(pwm != 0){ // compute the gain in each instante
      m_mean += lux/(float)pwm;
    }else{  // adaptation becasue this is an indetermination but, there is never complete dark in the environment
      b_mean += lux;
    }
    
    // detect if the is reached the top or the bottom
    if(pwm == 255){ flag = -1; }  
    else if(pwm == 0){ flag = 1; }

    pwm += flag;  // update pwm value
    
   }
  
  // 'relax' the light in the box
  analogWrite(LED_PWM, 0); 
  delay(200);

  b_mean = b_mean/cicle_times;     // offset to compensate unpleasent light in the dark
  m_mean = m_mean/len_without_b;  // gain (G): x(t) = G*u(t)

  if (G){ //prints computed gain
  // print the gain and offset
  Serial.print("Gain and offset is: Lux = "); 
  Serial.print(m_mean, 4);
  Serial.print(" * PWM + ");
  Serial.println(b_mean, 4);
  }
  
   
}

void steps_up(float m){

  // static memory alocation
  short  size_b = 400; // size of the array that the data is stored 
  unsigned short time_array[size_b];
  unsigned short voltageOut[size_b];

  short b;  // index of that array
  unsigned short t = 0; // inits the counter
  unsigned int step_response; // theorical response
  float treeshold;  // theorical response 63%
  int samp_time_micro = 1000; // initializes timer1, and set a Sampling freq to 1KHz <-> samp. time: 1ms;
  Serial.println(samp_time_micro);
  
  // does the step up with a split pulse (pwm = 0) : pwm = {0, 1, 0, 2, ..., 0, 254, 0, 255}  
  for(byte pwm = 1; pwm != 0; pwm++){

    // defines the theorical responses for each step
    analogWrite(LED_PWM, pwm);
    delay(500);
    step_response = analogRead(LDR_ANALOG);
    treeshold = 0.63*step_response;
    
    // resets the light to 0 in order to compute the step up response
    analogWrite(LED_PWM, 0);
    while(analogRead(LDR_ANALOG) != 0){delay(10);} // wait until there is no light

    // starting values 
    b=0;
    voltageOut[b] = 0;
    time_array[b] = 0;
    t = 0;
    
    // starts the interruption at freq. samp. = 1KHz 
    Timer1.initialize(samp_time_micro); 
    Timer1.attachInterrupt(tau_interruption);

    // writes in the led the step up
    analogWrite(LED_PWM, pwm);

    // gets data, until the response is bellow the 63%
    do{
      if(flag_itr){ // every time the interruption happened, it matchs to an instant
          voltageOut[++b] = analog_value; // value measured in the interrupt
          time_array[b] = ++t; // the gap between to interrupts is constante (1ms) so it is not needed to call micros()
          flag_itr = false; // instant completed
      }
    } while ( voltageOut[b] < treeshold);

    // stops the interruption
    Timer1.detachInterrupt();
    
    // disables the led
    analogWrite(LED_PWM, 0);  
    
    // writes on the terminal the data to compute the tau(63%) and the dead time in the python file
    Serial.print("PWM");
    Serial.print('\t');
    Serial.print(pwm);
    Serial.print('\t');
    Serial.println(voltage_to_lux(treeshold*(VCC/MAX_ANALOG),m));

    for(int i= 0; i <= b; i++ ){
      Serial.print(time_array[i]);
      Serial.print('\t');
      Serial.println(voltage_to_lux(voltageOut[i]*(VCC/MAX_ANALOG),m));
    }
  }

}

void steps_down(float m){

  // static memory alocation
  short  size_b = 400; // size of the array that the data is stored 
  unsigned short time_array[size_b];
  unsigned short voltageOut[size_b];

  short b;  // index of that array
  unsigned short t = 0; // inits the counter
  unsigned int step_response; // theorical response
  float treeshold;  // theorical response 63%
  int samp_time_micro = 1000; // initializes timer1, and set a Sampling freq to 1KHz <-> samp. time: 1ms;
  Serial.println(samp_time_micro);

  // computes the maximum brightness 
  unsigned int aux_read;  
  analogWrite(LED_PWM, 255);
  delay(500);
  unsigned short max_brightness = analogRead(LDR_ANALOG); // value to be imposed during the steps
  
  // does the step down with a split pulse (pwm = 255) : pwm = {255, 0, 255, 1, ..., 255, 253, 255, 254}  
  for(int pwm = 0; pwm < 255; pwm++){

    // defines the theorical responses for each step
    analogWrite(LED_PWM, pwm);
    delay(500);
    step_response = analogRead(LDR_ANALOG);
    treeshold = max_brightness - 0.63*(max_brightness - step_response);

    // resets the light to 255 in order to compute the step down response
    analogWrite(LED_PWM, 255);
    while(analogRead(LDR_ANALOG) < max_brightness){delay(10);} // wait until the brightness is in the max level
   

    // starting values 
    b=0;
    voltageOut[b] = max_brightness;
    time_array[b] = 0;
    t = 0;

    // starts the interruption at freq. samp. = 1KHz 
    Timer1.initialize(samp_time_micro); 
    Timer1.attachInterrupt(tau_interruption);
    
    // writes in the led the step down
    analogWrite(LED_PWM, pwm);

    // gets data, until the response is bellow the 63%
    do{
      if(flag_itr){
          voltageOut[++b] = analog_value;
          time_array[b] = ++t;
          flag_itr = false;
      }
    } while ( voltageOut[b] > treeshold);

    // stops the interruption
    Timer1.detachInterrupt();

    // enables the led in the maximum brightness level
    analogWrite(LED_PWM, 255);
    
    // writes on the terminal the data to compute the tau(63%) and the dead time in the python file
    Serial.print("PWM");
    Serial.print('\t');
    Serial.print(pwm);
    Serial.print('\t');
    Serial.println(voltage_to_lux(treeshold*(VCC/MAX_ANALOG),m));
    
    for(int i= 0; i <= b; i++ ){
      Serial.print(time_array[i]);
      Serial.print('\t');
      Serial.println(voltage_to_lux(voltageOut[i]*(VCC/MAX_ANALOG),m));
    }
  }

}
/*
This interruption happens every 1ms seconds. 
*/
void tau_interruption()
{
  analog_value = analogRead(LDR_ANALOG);  // get the exact value of V0
  flag_itr = true;  // inform the code that there is a new value
}
