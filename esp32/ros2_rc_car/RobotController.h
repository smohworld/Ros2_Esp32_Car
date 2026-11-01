#ifndef __ROBOT_CONTROLLER_H__
#define __ROBOT_CONTROLLER_H__

#include "DCMotorSpeed.h"
#include "myMPU6050.h"

class RobotController {
  DCMotorSpeed _dcFL;
  DCMotorSpeed _dcRL;
  DCMotorSpeed _dcFR;
  DCMotorSpeed _dcRR;
  MyMPU6050 _imu;
  int _batPin;
  float _WHEEL_RADIUS;
  float _TRACK_WIDTH;

  int32_t _enc_sec = 0;
  uint32_t _enc_nanosec = 0;
  
public:
  RobotController();
  void begin();
  void setTargetSpeeds(float v, float w);
  void updateMotorSpeedsRamped(float dt);
  void getDeltaEncoderPos(long&, long&, long&, long&);
  int16_t getGyroZCalib();
  void setTargetRPMs(int rpm_left, int rpm_right);
  void getEncoderStamp(int32_t& sec, uint32_t& nanosec);
  void calculatePWM(double dt);
  void updateMotorSpeeds();  
  void peekAndUpdateMotorSpeeds(
    double&, double&, double&, double&,
    int&, int&, int&, int&,
    double&, double&, double&, double&);
  void peekAndUpdateMotorSpeedsRamped(
    double&, double&, double&, double&,
    int&, int&, int&, int&,
    double&, double&, double&, double&);
  int32_t getBatteryValue();
  int32_t getBatteryVoltage();
};

extern RobotController robot;

#endif
