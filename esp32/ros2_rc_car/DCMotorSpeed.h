#ifndef __DC_MOTOR_SPEED_H__
#define __DC_MOTOR_SPEED_H__

#include <Arduino.h>
#include "DCMotorPWM.h"
#include "Encoder.h"
#include "PIDController.h"

class DCMotorSpeed {
  // 다른 클래스들의 인스턴스
  DCMotorPWM _motor;
  Encoder _encoder;
  PIDController _pid;

  // 제어 루프 관련 변수
  int _setRPM=0;
  double _currentRPM=0;
  int _outputPWM=0;
  double _offsetPWM=0;
  int _prevSetRPM=0;

  const int _CPRWheel;
  long _encPosCurr=0;
  long _encPosPrev=0;
  long _dEncPos=0;

public:
  // 생성자: 모든 핀과 PID 상수를 초기화합니다.
  DCMotorSpeed(
    int motor_pin_a, int motor_pin_b, int pwm_ch_a, int pwm_ch_b,
    int enc_pin_a, int enc_pin_b,
    double kp=32.0, double ki=32.0, double kd=1.0,
    double min_output=-1023, double max_output=1023,
    const int PPR=7, const int GRR=298 // 150
  );

  int getBasePWM(int rpm);
  // 실제 PID 계산 로직을 담고 있는 멤버 함수
  void calculatePWM(double dt);
  long getEncoderPos();
  long getDeltaEncoderPos();

  // 모든 하드웨어 및 제어 루프를 시작합니다.
  void begin();

  // 목표 RPM을 설정합니다.
  void setSetRPM(int rpm);  
  int getSetRPM();

  // 현재 RPM을 반환합니다.
  double getCurrentRPM();

  // 계산된 PWM 값을 반환합니다.
  int getOutputPWM();
  double getOffsetPWM();
  
  void setSpeed(int speed);
  void setSpeedRamped(int speed);
  void stop();
  void brake();
};

#endif
