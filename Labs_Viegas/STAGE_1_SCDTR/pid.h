#ifndef PID_H
#define PID_H

class Pid {
  private:
   float Ki;
   float Kd;
   float Kp;

   float u_i;
   float u_i_before;

   bool feedback;
   
  public:
    void init(float, float, float, bool);
    float compute_error(float, float);
    
};


#endif
