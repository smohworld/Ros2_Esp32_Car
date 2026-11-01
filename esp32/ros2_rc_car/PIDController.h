#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <Arduino.h>

class PIDController {
  double _kp, _ki, _kd;
  double _minOutput, _maxOutput;
  double _prevInput;
  double _cumError, _prevError;
  double _filteredRateError;
  double _alpha;

public:
  PIDController(double kp, double ki, double kd, double minOutput, double maxOutput);
  double compute(double setPoint, double input, double dt);
  void reset();
};

#endif
