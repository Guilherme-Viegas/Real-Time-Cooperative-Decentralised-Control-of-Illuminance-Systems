#ifndef LDR_CONTROLLER_H
#define LDR_CONTROLLER

#include "util.h"

/**
 * Tau is the time constant of the system.
 *
 * The time constant represents the elapsed time required for the system response
 * to decay to zero if the system had continued to decay at the initial rate.
 *
 * In an increasing system, the time constant is the time for the system's step response
 * to reach 1 − 1/e ≈ 63.2% of its final (asymptotic) value (say from a step increase).
 * Moreover, from a step decrease is  1/e ≈ 36.8%.
 *
 * This classes represents the ralation between Tau and PWM.
 *
 * It was fit a curve: Tau = A*e^{B * PWM} + C [ms]
 *
 * @param a, a >= 0 ( Needs to respect physical laws, time positive defined )
 * @param b, b < 0 ( in order to converge the function )
 * @param c, c >= 0 ( Lux offset, time is positive defined )
 */
typedef class Tau{

  private:
    volatile float t_a = 0.0;
    volatile float t_b = 0.0;
    volatile float t_c = 0.0;

    boolean t_isDefine = false;

  public:
    Tau();  // constructor
    void setParametersABC(float A, float B, float C); // declares the parameters
    float fTau(short x); // returns tau

}TAU;

typedef class LdrController{

  private:
    float t_m = -0.718;  // slope -> R2 = lux*m + b
    float t_bb = 0.0;

    float t_gain = 0.0;   // [Lux/PWM]
    float t_offset = 0.0; // [Lux]

    float t_maxLux = -1.0;

    int t_pin;

  public:
    LdrController( int t_pin = 0 );   // constructor
    float luxToOutputVoltage( float x, boolean reverse = false );
    float luxToPWM( float x, bool reverse = false );
    float getOutputVoltage();
    float get_m(){ return t_m; }
    float get_offset(){ return t_offset; }
    float boundLUX( float lux );
    void setGain( byte led_pin, float m = -0.718 );
    void compute_gain( byte led_pin );

    TAU t_tau_up; // create Tau type
    TAU t_tau_down; // create Tau type

}LDR;

#endif
