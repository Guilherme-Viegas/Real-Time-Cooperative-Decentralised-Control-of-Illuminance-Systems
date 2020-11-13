#ifndef PID_H
#define PID_H

#include "utils.h"
#include "LDR_class.h"


class pid{
    private:
        bool derivative;
        float kp, ki, kd, T, a;
    public:
    pid(float, float = 100, float = 0, float = 0, float = 0);
    ~pid();
    void printParam();
    int ref2pwm(float ref, LDR simulator);
    int saturation(int u);
    float getKp();
    float getKi();
    float getT();
};
#endif
