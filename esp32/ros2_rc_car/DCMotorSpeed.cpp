#include "DCMotorSpeed.h"

DCMotorSpeed::DCMotorSpeed(  
    int motor_pin_a, int motor_pin_b, int pwm_ch_a, int pwm_ch_b,
    int enc_pin_a, int enc_pin_b,
    double kp, double ki, double kd,
    double min_output, double max_output,
    const int PPR, const int GRR
): 
  _motor(motor_pin_a, motor_pin_b, pwm_ch_a, pwm_ch_b), 
  _encoder(enc_pin_a, enc_pin_b), 
  _pid(kp, ki, kd, min_output, max_output),
  _CPRWheel(PPR*4*GRR) {}

int DCMotorSpeed::getBasePWM(int rpm) {
  const int MAX_RPM = 120;
  const int DEAD_ZONE_PWM = 500;
  const int MAX_PWM = 980; // (980-500)/120=4
  if (rpm == 0) return 0;

  float ratio = abs(rpm) / (float)MAX_RPM;
  ratio = constrain(ratio, 0., 1.);

  int pwm = DEAD_ZONE_PWM + (MAX_PWM - DEAD_ZONE_PWM) * ratio;
  return (rpm > 0) ? pwm : -pwm;
}

void DCMotorSpeed::calculatePWM(double dt) {
  _encPosCurr = _encoder.getPos();
  _dEncPos = _encPosCurr - _encPosPrev;
  _encPosPrev = _encPosCurr;

  _currentRPM = (double)_dEncPos/_CPRWheel/dt*60;

  // 방향이 바뀌면 감속 + PID 리셋
  if (_setRPM * _prevSetRPM < 0) {
    _pid.reset();

    // 방향 반전 시 순간 감속 (브레이크 효과)
    _outputPWM = 0;
    _motor.setSpeed(_outputPWM);
    delay(100); // 감속 시간 (ms)
  }
  _prevSetRPM = _setRPM;

  // PID 오프셋 계산
  _offsetPWM = (int)_pid.compute(_setRPM, _currentRPM, dt);

  // Feedforward 기본 PWM
  int basePWM = getBasePWM(_setRPM);

  // Feedforward 중심 + PID 오프셋 일부 반영
  _outputPWM = basePWM + (int)(1.0 * _offsetPWM);

  // 안전 제한
  _outputPWM = constrain(_outputPWM, -1023, 1023);

  // 정지 명령 시 부드럽게 감속
  if (_setRPM == 0 && abs(_currentRPM) < 2.0) {
    _outputPWM = 0;
  }
}

long DCMotorSpeed::getEncoderPos() {
  return _encPosCurr;
}

long DCMotorSpeed::getDeltaEncoderPos() {
  return _dEncPos;
}

void DCMotorSpeed::begin() {
  _motor.begin();
  _encoder.begin();
}

void DCMotorSpeed::setSetRPM(int rpm) {
  _setRPM = rpm;
}
  
int DCMotorSpeed::getSetRPM() {
  return _setRPM;
}

double DCMotorSpeed::getCurrentRPM() {
  return _currentRPM;
}

int DCMotorSpeed::getOutputPWM() {
  return _outputPWM;  
}

double DCMotorSpeed::getOffsetPWM() {
  return _offsetPWM;
}

void DCMotorSpeed::setSpeed(int speed) {
  _motor.setSpeed(speed);
}

void DCMotorSpeed::setSpeedRamped(int speed) {
  _motor.setSpeedRampedEx(speed);
}
