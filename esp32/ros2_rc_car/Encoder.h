#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <Arduino.h>

class Encoder {
  int C1;
  int C2;
  volatile long pos;
  portMUX_TYPE encMux;

  void IRAM_ATTR C1_ISR();
  void IRAM_ATTR C2_ISR();

  static void IRAM_ATTR C1_ISR_static(void* arg);
  static void IRAM_ATTR C2_ISR_static(void* arg);

public:
  Encoder(int C1, int C2);
  void begin();
  long getPos();
  void reset();  
};

#endif
