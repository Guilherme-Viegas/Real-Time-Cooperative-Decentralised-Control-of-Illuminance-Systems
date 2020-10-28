#include <Arduino.h>
#include "utils.h"
#include "pid.h"

void Pid::init(float _ki, float _kd, float _kp, bool _feedforward) {
  ki = _ki;
  kd = _kd;
  kp = _kp;
  feedforward = _feedforward;
}

float compute_error(float y_lux, float simulated_lux) {
  return ( y_lux - simulated_lux );
}
