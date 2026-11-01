#include "PIDController.h"
  
PIDController::PIDController(double kp, double ki, double kd, double minOutput, double maxOutput)
  :_kp(kp), _ki(ki), _kd(kd), _minOutput(minOutput), _maxOutput(maxOutput), 
  _prevInput(0), _cumError(0), _prevError(0), _filteredRateError(0), _alpha(0.2) {}

double PIDController::compute(double setPoint, double input, double dt) {
  // 목표 속도가 0이면 리셋
  if(setPoint==0) {
    reset();
    return 0;
  }

  double error = setPoint - input;
  _prevInput = input;

  double absSet = abs(setPoint);
  // 저속 구간일수록 I항 가중치 줄이기
  double adaptiveKi = (absSet < 5) ? _ki * 0.3 : _ki;
  _cumError += error * dt;
  
  // Anti-windup
  double iLimit = _maxOutput / (2 * max(adaptiveKi, 0.0001));
  _cumError = constrain(_cumError, -iLimit, iLimit);

  // 미분항 노이즈 필터
  double rateErrorRaw = - (input - _prevInput) / dt;
  _filteredRateError = _alpha * rateErrorRaw + (1 - _alpha) * _filteredRateError;
  _prevError = error;

  double output = _kp * error + adaptiveKi * _cumError + _kd * _filteredRateError;
  
  return constrain(output, _minOutput, _maxOutput);
}

void PIDController::reset() {
  _prevInput = 0;
  _cumError = 0;
  _prevError = 0;
  _filteredRateError = 0;
}
