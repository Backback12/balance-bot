#ifndef MOVEMENT_H
#define MOVEMENT_H

#define MAX_ROLL_ANGLE 20   // limit leg difference

// #define SERVO_MAX_US 2500
// #define SERVO_MIN_US 500

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_VL53L0X.h>



namespace movement {

    

    void init();
    void loop();



} // namespace movement

namespace servos
{
    extern float left_percent;
    extern float right_percent;
    extern float left_angle;
    extern float right_angle;
    // TwoWire buss;
    // Adafruit_PWMServoDriver pwm;
    void init(TwoWire &wire);
    void tick();
    void legMove(uint8_t channel, float percent);
    void legsMove(float percent, float offset);
    void headMove(int pitch, int yaw);
} // namespace servos

namespace motors
{
  static const float REDUCTION_RATIO = 1.0f;
  static const int ENCODER_PPR = 48;
  static const int QUADRATURE = 4;

  static const int PWM_FREQ = 20000;
  static const int PWM_RESOLUTION = 8; // 0..255
  static const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;
  static const int PWM_CH_M1A = 1;
  static const int PWM_CH_M1B = 2;
  static const int PWM_CH_M2A = 3;
  static const int PWM_CH_M2B = 4;

  static float target_rpm_1_uhhh = 0;
  static float target_rpm_2_uhhh = 0;

  void IRAM_ATTR isr_enc1();
  void IRAM_ATTR isr_enc2();

  void init();
  void tick();
  void stopMotor(int channel);
  // void setMotorRaw(int motorIndex, uint8_t duty8, uint8_t dir = LOW);
  void setMotorPwmRaw(int motorIndex, int duty8);


  // encoder stuff!!
  void reset_encoder_count(int channel);
  long read_encoder(int channel);
  long read_rpm(int channel);
} // namespace motors

namespace imu
{
  // TwoWire &buss;
  
  // uint8_t buf14[14];
  // int16_t ax;
  // int16_t ay;
  // int16_t az;
  // int16_t gx;
  // int16_t gy;
  // int16_t gz;
  extern float angle_pitch;
  extern float angle_roll;
  extern float angle_yaw;

  void init(TwoWire &wire);
  void set_gyro_bias();
  void tick();
  void i2cWriteByte(uint8_t reg, uint8_t data);
  void i2cReadBytes(uint8_t reg, uint8_t count, uint8_t *dest);
  void run_calibration();
} // namespace imu






namespace ToF
{
    static const uint8_t NUM_SENSORS = 3;

    static const int XSHUT_0 = 23;
    static const int XSHUT_1 = 26;

    static const uint8_t ADDR_2 = 0x30;
    static const uint8_t ADDR_0 = 0x31;
    static const uint8_t ADDR_1 = 0x32;

    static Adafruit_VL53L0X sensors[NUM_SENSORS];
    static TwoWire* i2c = nullptr;

    // inline bool init(TwoWire& wire)
    // {
    //     i2c = &wire;
    //     i2c->begin();

    //     pinMode(XSHUT_0, OUTPUT);
    //     pinMode(XSHUT_1, OUTPUT);

    //     digitalWrite(XSHUT_0, LOW);
    //     digitalWrite(XSHUT_1, LOW);
    //     delay(10);

    //     // --- Sensor 2 (always on, default 0x29)
    //     if (!sensors[2].begin(0x29, false, i2c))
    //         return false;

    //     sensors[2].setAddress(ADDR_2);
    //     delay(10);

    //     // --- Sensor 0
    //     digitalWrite(XSHUT_0, HIGH);
    //     delay(10);

    //     if (!sensors[0].begin(0x29, false, i2c))
    //         return false;

    //     sensors[0].setAddress(ADDR_0);
    //     delay(10);

    //     // --- Sensor 1
    //     digitalWrite(XSHUT_1, HIGH);
    //     delay(10);

    //     if (!sensors[1].begin(0x29, false, i2c))
    //         return false;

    //     sensors[1].setAddress(ADDR_1);
    //     delay(10);

    //     return true;
    // }

    inline bool init(TwoWire& wire)
{
    i2c = &wire;
    i2c->begin();

    // Single sensor at default address 0x29
    if (!sensors[0].begin(0x29, false, i2c))
        return false;

    return true;
}

    inline uint16_t tofRead(uint8_t index)
    {
        if (index >= NUM_SENSORS) return 0;

        VL53L0X_RangingMeasurementData_t measure;
        sensors[index].rangingTest(&measure, false);

        if (measure.RangeStatus != 4)
            return measure.RangeMilliMeter;

        return 0;
    }
}
#endif