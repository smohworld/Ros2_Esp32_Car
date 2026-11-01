#include "myMPU6050.h"

MyMPU6050::MyMPU6050(uint8_t addr) 
  : _addr(addr) {}

void MyMPU6050::begin() {
  Wire.begin();
  Wire.setClock(400000);

  // MPU6050 활성화
  Wire.beginTransmission(_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Gyro 범위 ±250 deg/s
  // 131 LSB / (deg/s)
  Wire.beginTransmission(_addr);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // DLPF 44Hz (모터 진동 노이즈 감소, 10ms 샘플링 aliasing 방지)
  Wire.beginTransmission(_addr);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  // NVS에서 보정값 읽기
  prefs.begin("imu", true);
  _gyroZOffset = prefs.getInt("GyZOff", 65);
  prefs.end();

  Serial.print("Loaded GyZOff = ");
  Serial.println(_gyroZOffset);
}

int16_t MyMPU6050::readGyroZRaw() {
  Wire.beginTransmission(_addr);   
  Wire.write(GYRO_Z_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)_addr, (uint8_t)2, true);
  int16_t high = Wire.read();
  int16_t low = Wire.read();
  return (high << 8) | low;
}

// 보정된 각속도 (deg/s)
int16_t MyMPU6050::getGyroZCalib() {
  int16_t GyZ = readGyroZRaw();
  int16_t GyZCalib = GyZ - _gyroZOffset;
  return GyZCalib;
}

double MyMPU6050::getGyroZ_DegPerSec() {
  int16_t GyZCalib = getGyroZCalib();
  return GyZCalib/GYROXYZ_TO_DPS;
}