#include "sensors.h"





namespace encoders {

// encoder stuff
void reset_encoder_count(int channel) {
  noInterrupts();
  if (channel == 1) enc1_count = 0;
  else enc2_count = 0;
  interrupts();
}

// read counts
long read_encoder(int channel) {
  
  
  // noInterrupts();
  // long output = 0;
  // if (channel == 1) {
    //   output = enc1_count;
    //   enc1_count = 0;
    // }
  // else {
    //   output = enc2_count;
  //   enc2_count = 0;
  // }
  // interrupts();
  // return output;
  
  
  static long last1 = 0, last2 = 0;
  
  noInterrupts();
  long current = (channel == 1) ? enc1_count : enc2_count;
  interrupts();

  long delta = current - ((channel == 1) ? last1 : last2);
  ((channel == 1) ? last1 : last2) = current;

  return delta;
}

unsigned long last_encoder_sample_1 = millis();
unsigned long last_encoder_sample_2 = millis();

long read_rpm(int channel) {
  unsigned long now = micros();
  unsigned long dt_us = now - ((channel == 1) ? last_encoder_sample_1 : last_encoder_sample_2);
  // last_encoder_sample = now;
  ((channel == 1) ? last_encoder_sample_1 : last_encoder_sample_2) = now;

  long encoder_ticks = read_encoder(channel);

  float revolutions = (float)encoder_ticks / (ENCODER_PPR * 2);
  // float minutes = (float)dt / 60000.0f;
  float minutes = dt_us / 60e6;
  return revolutions / minutes;
}

}//end namespace encoders




