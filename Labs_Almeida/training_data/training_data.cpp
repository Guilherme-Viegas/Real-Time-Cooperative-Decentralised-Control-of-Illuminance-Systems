#include "training_data.h"

volatile unsigned short analog_value;
volatile boolean flag_itr = false;

float voltage_to_lux(float v0, float m){
  
  float b = log10(5E4);

  float function = (log10((VCC / v0) * R1 - R1) - b) / m;
  float lux = pow(10, function);
  return lux;
}

void test_cont(float m){ 

  byte pwm = 0;
  float flag = 1;
  float lux = 0.0;
  float voltageOut = 0.0;
  
  int cicles = 5*510;
  
  for(int i=0; i<cicles; i++){

    analogWrite(LED_PWM, pwm);
    delay(20);
 
    voltageOut = analogRead(LDR_ANALOG)*(VCC/MAX_ANALOG);
  
    lux = voltage_to_lux(voltageOut,m);
  
    Serial.print(pwm);
    Serial.print('\t');
    Serial.println(lux);

    if(pwm == 255){ flag = -1; }
    else if(pwm == 0){ flag = 1; }

    pwm += flag;
    
   }
   analogWrite(LED_PWM, 0);
   delay(20);
}

void test_steps_up(float m){

  Timer1.start();

  short  size_b = 400; // Atention
  short b;
  
  unsigned short time_array[size_b];
  unsigned short voltageOut[size_b];
  
  unsigned short t = 0; 

  unsigned int step_response;
  int samp_time_micro = 1000; // initialize timer1, and set a 1KHz;
  Serial.println(samp_time_micro);
  
  for(byte pwm = 1; pwm != 0; pwm++){

    analogWrite(LED_PWM, pwm);
    delay(500);
    step_response = analogRead(LDR_ANALOG);
    analogWrite(LED_PWM, 0);
    
    while(analogRead(LDR_ANALOG) != 0){delay(10);} // wait until there is no light

    b=-1;
    t=0;

    Timer1.initialize(samp_time_micro); 
    Timer1.attachInterrupt(tau_interruption);
    analogWrite(LED_PWM, pwm);

    do{
      if(flag_itr){
          voltageOut[++b] = analog_value;
          time_array[b] = t++;
          flag_itr = false;
      }
    } while ( voltageOut[b] < step_response);

    Timer1.detachInterrupt();
    
    analogWrite(LED_PWM, 0);
    
    Serial.print("PWM");
    Serial.print('\t');
    Serial.println(pwm);

    for(int i= 0; i <= b; i++ ){
      Serial.print(time_array[i]);
      Serial.print('\t');
      Serial.println(voltage_to_lux(voltageOut[i]*(VCC/MAX_ANALOG),m));
    }
  }

}

void test_steps_down(float m){

  Timer1.start();

  short  size_b = 400; // Atention
  short b;
  
  unsigned short time_array[size_b];
  unsigned short voltageOut[size_b];
  unsigned short t = 0;

  unsigned int step_response;
  int samp_time_micro = 1000; // initialize timer1, and set a 1KHz;
  Serial.println(samp_time_micro);
  
  for(byte pwm = 254; pwm != 255; pwm--){

    analogWrite(LED_PWM, pwm);
    delay(500);
    step_response = analogRead(LDR_ANALOG);
    analogWrite(LED_PWM, 255);
    delay(500);

    b=-1;
    t=0;

    Timer1.initialize(samp_time_micro); 
    Timer1.attachInterrupt(tau_interruption);
    analogWrite(LED_PWM, pwm);
  
    do{
      if(flag_itr){
          voltageOut[++b] = analog_value;
          time_array[b] = t++;
          flag_itr = false;
      }
    } while ( voltageOut[b] > step_response);
  
    Timer1.detachInterrupt();

     analogWrite(LED_PWM, 255);
    
    Serial.print("PWM");
    Serial.print('\t');
    Serial.println(pwm);

    for(int i= 0; i <= b; i++ ){
      Serial.print(time_array[i]);
      Serial.print('\t');
      Serial.println(voltage_to_lux(voltageOut[i]*(VCC/MAX_ANALOG),m));
    }
  }

}

void tau_interruption()
{
  analog_value = analogRead(LDR_ANALOG);
  flag_itr = true;
}
