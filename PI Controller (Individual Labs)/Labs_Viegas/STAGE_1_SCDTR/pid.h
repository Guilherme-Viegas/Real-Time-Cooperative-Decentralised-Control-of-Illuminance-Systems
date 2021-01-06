#ifndef PID_H
#define PID_H

class Pid {
  private:
   float Ki = 0;
   float Kd = 0;
   float Kp = 0;

   float T;

   float error_previous = 0;
   float y_previous = 0;
   float integral_previous = 0;
   float derivative_previous = 0;

   float K1, K2;
   
  public:
    void init(float, float, float, bool);
    float compute_pid(float, float, float);
    void zero_ui(bool);
    float dead_zone(float);

    Pid(float, float=0, float=0, float=0);

    //Getters
    float getT();
    float get_Ki();
    float get_Kd();
    float get_Kp();

    //Setters
    void setT(float);
    void set_Ki(float);
    void set_Kd(float);
    void set_Kp(float);
};


#endif
