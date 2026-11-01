#include "RobotController.h"

RobotController::RobotController() : 
  _dcFL(19,27,0,1,14,4),
  _dcRL(0,15,4,5,13,18),
  _dcFR(26,25,2,3,34,35),
  _dcRR(23,12,6,7,32,33),
  _batPin(36),
  _WHEEL_RADIUS(0.065/2),
  _TRACK_WIDTH(0.145f) {} 
  
void RobotController::begin(){
  _dcFL.begin();
  _dcRL.begin();
  _dcFR.begin();
  _dcRR.begin();

  _imu.begin();

  analogSetPinAttenuation(_batPin, ADC_11db); // 감쇠 11dB 설정
}

void RobotController::setTargetSpeeds(float v, float w) {
  // 속도 계산
  float left_speed = v - w * _TRACK_WIDTH / 2.0f;
  float right_speed = v + w * _TRACK_WIDTH / 2.0f;

  // RPM 값으로 변환
  int left_rpm = (int)(left_speed / (TWO_PI * _WHEEL_RADIUS) * 60);
  int right_rpm = (int)(right_speed / (TWO_PI * _WHEEL_RADIUS) * 60);
  
  Serial.printf("left_rpm:%d, right_rpm:%d\n", left_rpm, right_rpm);
  
  setTargetRPMs(left_rpm, right_rpm);
}

void RobotController::updateMotorSpeedsRamped(float dt) {
  calculatePWM(dt);
  
  double currentRPMFL, currentRPMRL, currentRPMFR, currentRPMRR;
  int outputPWMFL, outputPWMRL, outputPWMFR, outputPWMRR;
  double offsetPWMFL, offsetPWMRL, offsetPWMFR, offsetPWMRR;
  peekAndUpdateMotorSpeedsRamped(
    currentRPMFL, currentRPMRL, currentRPMFR, currentRPMRR,
    outputPWMFL, outputPWMRL, outputPWMFR, outputPWMRR,
    offsetPWMFL, offsetPWMRL, offsetPWMFR, offsetPWMRR);
}

void RobotController::setTargetRPMs(int rpm_left, int rpm_right) {
  rpm_left = constrain(rpm_left, -130, 130);
  rpm_right = constrain(rpm_right, -130, 130);

\

  _dcFL.setSetRPM(rpm_left);
  _dcRL.setSetRPM(rpm_left);
  _dcFR.setSetRPM(rpm_right);
  _dcRR.setSetRPM(rpm_right);
}

void RobotController::getEncoderStamp(int32_t& sec, uint32_t& nanosec) {
  sec = _enc_sec;
  nanosec = _enc_nanosec;
}

void RobotController::getDeltaEncoderPos(
  long& dEncPosFL, 
  long& dEncPosRL, 
  long& dEncPosFR, 
  long& dEncPosRR) {
  dEncPosFL = _dcFL.getDeltaEncoderPos();
  dEncPosRL = _dcRL.getDeltaEncoderPos();
  dEncPosFR = _dcFR.getDeltaEncoderPos();
  dEncPosRR = _dcRR.getDeltaEncoderPos();
}

int16_t RobotController::getGyroZCalib() {
  return _imu.getGyroZCalib();
}

void RobotController::calculatePWM(double dt) {
  struct timespec tv = {0, 0};
  clock_gettime(CLOCK_REALTIME, &tv);
  _enc_sec = tv.tv_sec;
  _enc_nanosec = tv.tv_nsec;
 
  _dcFL.calculatePWM(dt);
  _dcRL.calculatePWM(dt);
  _dcFR.calculatePWM(dt);
  _dcRR.calculatePWM(dt);
}

void RobotController::updateMotorSpeeds() {
  int outputPWMFL = _dcFL.getOutputPWM();
  int outputPWMRL = _dcRL.getOutputPWM();
  int outputPWMFR = _dcFR.getOutputPWM();
  int outputPWMRR = _dcRR.getOutputPWM();

  _dcFL.setSpeed(outputPWMFL);
  _dcRL.setSpeed(outputPWMRL);
  _dcFR.setSpeed(outputPWMFR);
  _dcRR.setSpeed(outputPWMRR);
}

void RobotController::peekAndUpdateMotorSpeeds(
  double& currentRPMFL, double& currentRPMRL, double& currentRPMFR, double& currentRPMRR,
  int& outputPWMFL, int& outputPWMRL, int& outputPWMFR, int& outputPWMRR,
  double& offsetPWMFL,double& offsetPWMRL,double& offsetPWMFR,double& offsetPWMRR) {
  currentRPMFL = _dcFL.getCurrentRPM();
  outputPWMFL = _dcFL.getOutputPWM();
  offsetPWMFL = _dcFL.getOffsetPWM();
  currentRPMRL = _dcRL.getCurrentRPM();
  outputPWMRL = _dcRL.getOutputPWM();
  offsetPWMRL = _dcRL.getOffsetPWM();
  currentRPMFR = _dcFR.getCurrentRPM();
  outputPWMFR = _dcFR.getOutputPWM();
  offsetPWMFR = _dcFR.getOffsetPWM();
  currentRPMRR = _dcRR.getCurrentRPM();
  outputPWMRR = _dcRR.getOutputPWM();
  offsetPWMRR = _dcRR.getOffsetPWM();

  _dcFL.setSpeed(outputPWMFL);
  _dcRL.setSpeed(outputPWMRL);
  _dcFR.setSpeed(outputPWMFR);
  _dcRR.setSpeed(outputPWMRR);
}

void RobotController::peekAndUpdateMotorSpeedsRamped(
  double& currentRPMFL, double& currentRPMRL, double& currentRPMFR, double& currentRPMRR,
  int& outputPWMFL, int& outputPWMRL, int& outputPWMFR, int& outputPWMRR,
  double& offsetPWMFL,double& offsetPWMRL,double& offsetPWMFR,double& offsetPWMRR) {
  currentRPMFL = _dcFL.getCurrentRPM();
  outputPWMFL = _dcFL.getOutputPWM();
  offsetPWMFL = _dcFL.getOffsetPWM();
  currentRPMRL = _dcRL.getCurrentRPM();
  outputPWMRL = _dcRL.getOutputPWM();
  offsetPWMRL = _dcRL.getOffsetPWM();
  currentRPMFR = _dcFR.getCurrentRPM();
  outputPWMFR = _dcFR.getOutputPWM();
  offsetPWMFR = _dcFR.getOffsetPWM();
  currentRPMRR = _dcRR.getCurrentRPM();
  outputPWMRR = _dcRR.getOutputPWM();
  offsetPWMRR = _dcRR.getOffsetPWM();

  long dEncPosFL = _dcFL.getDeltaEncoderPos();
  long dEncPosRL = _dcRL.getDeltaEncoderPos();
  long dEncPosFR = _dcFR.getDeltaEncoderPos();
  long dEncPosRR = _dcRR.getDeltaEncoderPos();

  _dcFL.setSpeedRamped(outputPWMFL);
  _dcRL.setSpeedRamped(outputPWMRL);
  _dcFR.setSpeedRamped(outputPWMFR);
  _dcRR.setSpeedRamped(outputPWMRR);
}

int32_t RobotController::getBatteryValue() {
  int32_t adcRaw = analogRead(_batPin);
  return adcRaw;
}

int32_t RobotController::getBatteryVoltage() {
  int32_t adcRaw = getBatteryValue();

  // 12bit ADC → 0~3.6V (ADC_11db 기준)
  float v_adc = (float)adcRaw / 4095.0 * 3.6;

  // 저항비 보정 (120k + 20k)
  float v_bat = v_adc * 7.0;

  float v_bat_compensated = v_bat*1.069;

  return int32_t(v_bat_compensated*1000);
}
