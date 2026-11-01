#ifndef __MY_MPU_6050_H__
#define __MY_MPU_6050_H__

#include <Wire.h>
#include <Preferences.h>

class MyMPU6050 {
  static constexpr uint8_t GYRO_Z_REG = 0x47;
  static constexpr double GYROXYZ_TO_DPS = 131.0; // LSB/°/s
  uint8_t _addr;
  int16_t _gyroZOffset;
  Preferences prefs;

  int16_t readGyroZRaw();
public:
  MyMPU6050(uint8_t addr = 0x68); 
  void begin();
  int16_t getGyroZCalib();
  double getGyroZ_DegPerSec();
};

#endif