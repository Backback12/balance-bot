#ifndef TESTING_H
#define TESTING_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "motors_test.h"
#include <math.h>

namespace testing {

// Motor tests (wrappers around motors_test)
void runMotorSweep(int motorIndex = 1, unsigned long sample_ms = 800, int stepPercent = 1, int startStep = 50);
void runMotorClosedLoop(int motorIndex, float targetOutputRPM = 50.0f, unsigned long runTimeMs = 5000);

// IMU/MPU test (I2C). This will run an infinite loop printing IMU readings.
void imuTest(uint8_t i2c_sda_pin = SDA, uint8_t i2c_scl_pin = SCL, TwoWire &wire = Wire);
void i2cWriteByte(TwoWire &wire, uint8_t reg, uint8_t data);
void i2cReadBytes(TwoWire &wire, uint8_t reg, uint8_t count, uint8_t *dest);

// PCA9685 servo test. Sweeps channels 0 and 1 from 0..180 degrees.
void servoTest(uint8_t i2c_sda_pin = SDA, uint8_t i2c_scl_pin = SCL, TwoWire &wire = Wire);
uint16_t microsecondsToTicks(uint16_t us);
void servoWrite(uint8_t channel, uint8_t angle);
void legMove(uint8_t channel, uint8_t angle);
void servoLegTest(uint8_t i2c_sda_pin = SDA, uint8_t i2c_scl_pin = SCL, TwoWire &wire = Wire);

void oledBuzzerTest();

// Convenience: run motor sweep, servo test, then enter imuTest (infinite)
void runAllTests();




// start a balancing loop (blocking) that reads IMU and uses motors_test closed-loop
void balance(uint8_t motorIndex = 1, TwoWire &wire = Wire);

// calibrate imu pitch offset (assumes bot is stationary and "balanced" at startup)
float calibrateIMUOffset(TwoWire &wire, uint16_t samples = 200, uint16_t delayMs = 10);



} // namespace testing

#endif // TESTING_H