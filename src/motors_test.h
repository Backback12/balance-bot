#ifndef MOTORS_TEST_H
#define MOTORS_TEST_H




#include <Arduino.h>
#include "config.h"
#include "movement.h"


namespace motors_test {

// static const float REDUCTION_RATIO = 9.6f; // encoders are before reduction
static const float REDUCTION_RATIO = 1.0f; // encoders are before reduction
static const int ENCODER_PPR = 48; // adjust to your encoder PPR (per channel)
static const int QUADRATURE = 4;   // quadrature decoding multiplies pulses

// PWM settings for ESP32 (adjust frequency/resolution if desired)
static const int PWM_FREQ = 20000;
static const int PWM_RESOLUTION = 8; // 0..255
static const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;
static const int PWM_CH_M1A = 1;
static const int PWM_CH_M1B = 2;
static const int PWM_CH_M2A = 3;
static const int PWM_CH_M2B = 4;

// volatile long enc1_count = 0;
// volatile long enc2_count = 0;
// long enc1_count = 0;
// long enc2_count = 0;
// volatile long enc1_count = 0;
// volatile long enc2_count = 0;

// static inline int readPinFast(uint8_t pin) { return digitalRead(pin); }

// void IRAM_ATTR isr_enc1();
// void IRAM_ATTR isr_enc2();
void setMotorRaw(int, uint8_t, uint8_t);
// void setMotorRaw(int motorIndex, uint8_t duty8, uint8_t dir = LOW) {
void stopMotor(int);
float countsToMotorRPM(long, unsigned long);
float motorRPMToOutputRPM(float);
void begin();
void runSweep(int, unsigned long, int, int);
void startClosedLoop(int , float, unsigned long);

}

#endif