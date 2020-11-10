#ifndef LOCAL_CONTROLLER_H
#define LOCAL_CONTROLLER_H

class Local_controller {
  private:
    byte led_pwm_port;
    byte ldr_analog;

    float G;
    float offset;
  
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

    float simulate(float, float, long, long);

    //Getters
    float getG();
    byte get_led_port();
    byte get_ldr_port();
    float getM();
    float getB();


    //Setters
    void setG(float);
};


#endif
