#ifndef SENSORS_H
#define SENSORS_H



#include <Arduino.h>


namespace encoders {


  static const float REDUCTION_RATIO = 1.0f;
  static const int ENCODER_PPR = 48;
  static const int QUADRATURE = 4;

  void IRAM_ATTR isr_enc1();
  void IRAM_ATTR isr_enc2();
  void init();
  // void tick();
  void reset_encoder_count(int channel);
  long read_encoder(int channel);
  long read_rpm(int channel);
}














#endif