#ifndef __DC_MOTOR_PWM_H__
#define __DC_MOTOR_PWM_H__

#include <Arduino.h>

class DCMotorPWM {
  int M1;
  int M2;  
  int CH1;
  int CH2;
  int _prevSpeed=0;
  const int _DUTY_STEP=4;
  void control(int value1, int value2);
public:
  DCMotorPWM(int M1, int M2, int CH1, int CH2);
  void begin();
  void setSpeed(int speed);
  void setSpeedRamped(int speed);
  void setSpeedRampedEx(int speed);
  void stop();
  void brake();
};

#endif
