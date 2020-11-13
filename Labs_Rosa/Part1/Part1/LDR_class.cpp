#include "LDR_class.h"

void LDR::init(int led_port, int _ldr_analog, float _G, float _offset, float _m, float _b, float _tau_a_U, float _tau_b_U, float _tau_c_U){

    led_pwm = led_port;
    ldr_analog = _ldr_analog;
    G = _G;
    offset = _offset;
    m = _m;
    b = _b;
    tau_a_U= _tau_a_U;
    tau_b_U = _tau_b_U;
    tau_c_U = _tau_c_U;
}

float LDR::computeGain(){
  int i = 0;
  int n_cycles = 4;
  int flag = 1;
  int pwm = 0;
  float m_sum = 0;
  float b_sum = 0;
  int m_num = 0;
  int b_num = 0;
  for(i=0; i < 256*n_cycles; i++){
    analogWrite(led_pwm, pwm);
    delay(10);
    //read voltage at LDR
    float v_out = analogRead(ldr_analog)*VCC/MAX_ANALOG;
    float lux = voltage2Lux(v_out, m);
    if(pwm==255){
      flag=-1;
    }
    else if(pwm==0){
      flag=1;
    }
    if(pwm!=0){
      m_sum += lux/float(pwm);
      m_num++;//number of different m's
    }
    else{
      b_sum += lux;
      b_num++;//number of b's
    }
    pwm += flag;
  }
  G = m_sum/float(m_num); //Gain lux = G*pwm
  offset = b_sum/float(b_num);
  return G;
};

/*
 * get the value of pwm and then computes
 * the tau
*/
float LDR::getTimeConstant(float pwm){
  return tau_a_U*exp(tau_b_U*pwm) + tau_c_U;
}
float LDR::getTheoricTau(int pwm){
  float lux = pwm2Lux(pwm);
  float R2 = getTheoricR2(lux);
  float Req = R1*R2/(R1+R2);
  return Req*C1*1E6;
}
//SIMULATE THE VOLTAGE IN LDR FROM THE LIGHT IN LUX
float LDR::simulate(float luxf, unsigned long ti){
  float tau = getTimeConstant((luxf-offset)/G);
  float vf = lux2Voltage(luxf, m);
  float Tfinal = micros();
  float vi = analogRead(ldr_analog)*VCC/MAX_ANALOG;

  float expo = 1/exp((Tfinal-ti)/tau);
  return (vf-(vf-vi)*expo);
}

/*
 * Get R2(lux)
*/
float LDR::getTheoricR2(float lux){
  float log_R2 = m*(log10(lux)) + log10(b);
  return pow(10, log_R2);
}
float LDR::getTheoricVFinal(float lux){
  float R2 = getTheoricR2(lux);
  return VCC*R1/(R1+R2);
}


/*
  converts pwm 2 Lux
  lux = G*pwm + offset
*/
float LDR::pwm2Lux(int pwm){
  return G*pwm+offset;
}
int LDR::lux2pwm(float lux){
  return (lux-offset)/G;
}

int LDR::getLedPin(){
  return led_pwm;
};

int LDR::getLDRpin(){
  return ldr_analog;
}
float LDR::getG(){
  return G;
};
float LDR::getOffset(){
  return offset;
}
float LDR::getM(){
  return m;
};
