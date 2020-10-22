#include "training_data.h"

volatile unsigned short analog_value;
volatile unsigned long t = 0;
volatile boolean flag = false;

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
  
  int cicles = 4*510;
  
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

void test_steps(float m){

  unsigned long time_start;
  Timer1.start();

  unsigned long  size = 1000100; // just in case
  int b;

  unsigned long time_array[size];
  float voltageOut[size];

  float step_response;


  for(byte pwm = 1; pwm != 0; pwm++){

    analogWrite(LED_PWM, pwm);
    delay(100);
    step_response = analogRead(LDR_ANALOG);
    analogWrite(LED_PWM, 0);
    delay(100);

    b=-1;
    //time_start = micros();
    t=0;
    Timer1.initialize(200); // initialize timer1, and set a 5KHz
    Timer1.attachInterrupt(tau_interruption); 
    analogWrite(LED_PWM, pwm);
    
    do{
      if(flag){
          voltageOut[++b] = analog_value;
          time_array[b] = t;
          flag = false;
      }

    } while ( voltageOut[b] < 0.63*step_response);
    Timer1.detachInterrupt();

    analogWrite(LED_PWM, 0);

    Serial.print("PWM");
    Serial.print('\t');
    Serial.println(pwm);
    Serial.print('\t');
    Serial.println(step_response);
    Serial.print('\t');
    Serial.println(b);


    for(int i= 0; i <= b; i++ ){
      Serial.print(time_array[i]);
      Serial.print('\t');
      //Serial.println(voltage_to_lux(voltageOut[i]*(VCC/MAX_ANALOG),m));
      Serial.println(voltageOut[i]);
    }

  }

}

void tau_interruption()
{
  analog_value = analogRead(LDR_ANALOG);
  flag = true;
  t++;
}
