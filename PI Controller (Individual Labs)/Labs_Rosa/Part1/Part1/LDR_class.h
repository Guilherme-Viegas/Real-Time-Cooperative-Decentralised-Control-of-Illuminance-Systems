#ifndef LDR_CLASS
#define LDR_CLASS
#include "utils.h"

class LDR{
    private:
        int led_pwm;
        int ldr_analog;

        float G;
        float offset;
        float m;
        float b;

        //tau = tau_a * exp(tau_b * pwm) + tau_c
        float tau_a_U;
        float tau_b_U;
        float tau_c_U;
    public:
        void init(int led_port, int _ldr_analog, float _G, float offset,float _m, float _b, float _tau_a_U, float _tau_b_U, float _tau_c_U);
        float computeGain();
        float getTimeConstant(float pwm);
        float simulate(float luxf, unsigned long ti);
        float getTheoricR2(float lux);
        float getTheoricTau(int pwm);
        float getTheoricVFinal(float lux);
        float pwm2Lux(int pwm);
        int lux2pwm(float lux);
        int getLedPin();
        int getLDRpin();
        float getG();
        float getM();
        float getOffset();
        void menu(String inbyte);

};


#endif
