#ifndef LOCAL_CONTROLLER_H
#define LOCAL_CONTROLLER_H

class Local_controller {
  private:
    int led_pwm_port;
    int ldr_analog;

    float G;
  
    float m;
    float b;
    
    float tau_a_up;
    float tau_b_up;
    float tau_c_up;
    float tau_a_down;
    float tau_b_down;
    float tau_c_down;

    float dead_time;
   
  public:
    void init(int, int, float, float, float, float, float, float, float, float, float, float);

    float compute_linear_gain();

    float compute_theoric_R2(float);

    float tau_up_function(float);
    float tau_down_function(float);

    float simulate_Vt(float, float, long, long, float);

    void print_params();

    //Getters
    float getG();
    float get_led_port();
    float get_ldr_port();
    float getM();
    float getB();

    //Setters
    void setG(float);
};


#endif
