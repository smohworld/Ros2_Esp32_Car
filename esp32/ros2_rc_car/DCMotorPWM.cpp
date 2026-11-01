#include "DCMotorPWM.h"

void DCMotorPWM::control(int value1, int value2) {
  ledcWrite(CH1, value1);
  ledcWrite(CH2, value2); 
}

DCMotorPWM::DCMotorPWM(int M1, int M2, int CH1, int CH2)
  :M1(M1), M2(M2), CH1(CH1), CH2(CH2) {}

void DCMotorPWM::begin() {
  ledcSetup(CH1, 10000, 10); // 채널, 20kHz, 10bit(0~1023)
  ledcSetup(CH2, 10000, 10);   
  ledcAttachPin(M1, CH1);
  ledcAttachPin(M2, CH2);
}

void DCMotorPWM::setSpeed(int speed) {
  speed=constrain(speed, -1023, 1023);
  int duty_cycle=abs(speed);

  if(speed>=0) {
    control(duty_cycle, 0); 
  } else if(speed<0) {
    control(0,duty_cycle);
  }
}

void DCMotorPWM::setSpeedRamped(int speed) {
  if(speed!=0) {
    int diff = speed - _prevSpeed;
    
    if(diff > _DUTY_STEP) {
      speed = _prevSpeed + _DUTY_STEP;
    } else if(diff < -_DUTY_STEP) {
      speed = _prevSpeed - _DUTY_STEP;
    }
  }

  _prevSpeed = speed;
  
  setSpeed(speed);
}

void DCMotorPWM::setSpeedRampedEx(int speed) {
  enum MotorState {
    Stopped,
    Rotating,
  };
  static MotorState motorState=Stopped;
  int KICK_OFF_DUTY_STEP = _DUTY_STEP*4;
  const int DEAD_ZONE_PWM = 500;
  
  if(speed!=0) {
    if(motorState==Stopped) {
      int diff = speed - _prevSpeed;
    
      if(diff > KICK_OFF_DUTY_STEP) {
        speed = _prevSpeed + KICK_OFF_DUTY_STEP;
      } else if(diff < -KICK_OFF_DUTY_STEP) {
        speed = _prevSpeed - KICK_OFF_DUTY_STEP;
      }
      
      if(speed>=DEAD_ZONE_PWM) {        
        motorState=Rotating;
      } else if(speed<=-DEAD_ZONE_PWM) {
        motorState=Rotating;
      } 
    } else { // if(motorState==Rotating)
      int diff = speed - _prevSpeed;
    
      if(diff > _DUTY_STEP) {
        speed = _prevSpeed + _DUTY_STEP;
      } else if(diff < -_DUTY_STEP) {
        speed = _prevSpeed - _DUTY_STEP;
      }
    }
  } else { //if(speed==0)
    motorState=Stopped;
  }

  _prevSpeed = speed;
  
  setSpeed(speed);
}

void DCMotorPWM::stop() {
  control(0, 0);
}

void DCMotorPWM::brake() {
  control(1023, 1023);
}
