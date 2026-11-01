#include "Encoder.h"

void IRAM_ATTR Encoder::C1_ISR() {
  byte val1=digitalRead(C1);
  byte val2=digitalRead(C2);
  if(val1==val2) pos++;
  else pos--;
}

void IRAM_ATTR Encoder::C2_ISR() {
  byte val1=digitalRead(C1);
  byte val2=digitalRead(C2);
  if(val1!=val2) pos++;
  else pos--;
}

void IRAM_ATTR Encoder::C1_ISR_static(void* arg) {
  Encoder* enc=static_cast<Encoder*>(arg);
  enc->C1_ISR();
}

void IRAM_ATTR Encoder::C2_ISR_static(void* arg) {
  Encoder* enc=static_cast<Encoder*>(arg);
  enc->C2_ISR();
}

Encoder::Encoder(int C1, int C2)
  : C1(C1),C2(C2), encMux(portMUX_INITIALIZER_UNLOCKED) {}

void Encoder::begin() {
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);
  attachInterruptArg(C1, C1_ISR_static, this, CHANGE);
  attachInterruptArg(C2, C2_ISR_static, this, CHANGE);    
}

long Encoder::getPos() {
  portENTER_CRITICAL(&encMux);
  long tmp_pos=pos;
  portEXIT_CRITICAL(&encMux);
  return tmp_pos;
}

void Encoder::reset() {
  portENTER_CRITICAL(&encMux);
  pos=0;
  portEXIT_CRITICAL(&encMux);
}
