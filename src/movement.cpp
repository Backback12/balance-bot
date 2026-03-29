
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include "movement.h"


namespace movement {

void init() {
  // Init I2C bus #1
  // TwoWire &wire = Wire;   // start new bus? does this do this??? what if Wire is already inits?
  // YES, use Wire and Wire1
  Wire.begin(PIN_SDA1, PIN_SCL1);
  Wire.setClock(400000);

  servos::init(Wire);
  motors::init();
  imu::init(Wire);
}

void loop() {
  servos::tick();
  motors::tick();
  imu::tick();
}










} // end namespace


namespace servos
{
  float left_percent;
  float right_percent;
  float left_angle;
  float right_angle;

  Adafruit_PWMServoDriver pwm;


  void init(TwoWire &wire) {
    // Init servos
    pwm = Adafruit_PWMServoDriver(ADDR_PCA9265, wire);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);   // recommended?
    pwm.setPWMFreq(50);
  }
  void tick() {

  }


  uint16_t percentToPulse(float percent, bool inverted, float *debug_angle_pointer=nullptr) {
    percent = constrain(percent, 0.0f, 100.0f);

    float angle = map(percent, 0.0f, 100.0f, 10.0f, 60.0f); // safe angles found through testing
    if (debug_angle_pointer != nullptr) {
      *debug_angle_pointer = angle;
    }
    uint16_t pulse = map(angle, 0.0f, 180.0f, 500.0f, 2500.0f);

    if (inverted) {
      pulse = 3000 - pulse;
    }

    return pulse;
  }

  void legMove(uint8_t channel, float percent) {
    // percent = constrain(percent, 0, 100);

    // uint8_t angle = map(percent, 0, 100, 10, 60);
    // // found angle 10-70 through testing, but 10-60 seems to be safe zone
    // // find some way to record offsets if needed?

    // // uint16_t pulse_us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    // uint16_t pulse_us = map(angle, 0, 180, 500, 2500);
    // uint16_t pulse_us_inv = 2500 + 500 - pulse_us;

    // // is pwm.writeMicroseconds() "stable"?
    // if (channel == 2) {
    //   pwm.writeMicroseconds(0, pulse_us_inv);
    //   pwm.writeMicroseconds(1, pulse_us);
    // }
    // else if (channel == 1) {
    //   pwm.writeMicroseconds(channel, pulse_us);
    // }
    // else {
    //   pwm.writeMicroseconds(channel, pulse_us_inv);
    // }

    bool inverted = (channel != 1);
    pwm.writeMicroseconds(channel, percentToPulse(percent, inverted));
  }

  // move both legs
  void legsMove(float percent, float offset) {
    // offset parameter to balance roll
    percent = constrain(percent, 0.0f, 100.0f);
    offset = constrain(offset, -20.0f, 20.0f);

    // uint8_t percent_1 = constrain(percent + offset, 0, 100);
    // uint8_t percent_2 = constrain(percent - offset, 0, 100);

    // uint8_t angle_1 = map(percent_1, 0, 100, 10, 60);
    // uint8_t angle_2 = map(percent_2, 0, 100, 10, 60);

    // uint16_t pulse_us_1 = map(angle_1, 0, 180, 500, 2500);
    // uint16_t pulse_us_2 = map(angle_2, 0, 180, 2500, 500);

    // is pwm.writeMicroseconds() "stable"?
    // pwm.writeMicroseconds(1, pulse_us_1);
    // pwm.writeMicroseconds(0, pulse_us_2);

    float left = percent + offset;
    float right = percent - offset;

    left_percent = left;
    right_percent = right;

    pwm.writeMicroseconds(1, percentToPulse(left, false, &left_angle));
    pwm.writeMicroseconds(0, percentToPulse(right, true, &right_angle));
  }

  void headMove(int pitch, int yaw) {
    pitch = constrain(pitch, 0, 180); // for now 90 is center
    yaw = constrain(yaw, 0, 180);

    float pitch_pulse = map(pitch, 0.0f, 180.0f, 500.0f, 2500.0f);
    float yaw_pulse = map(pitch, 0.0f, 180.0f, 500.0f, 2500.0f);

    pwm.writeMicroseconds(2, pitch_pulse);
    pwm.writeMicroseconds(3, yaw_pulse);
  }

}


namespace motors
{
  volatile long enc1_count = 0;
  volatile long enc2_count = 0;
  static inline int readPinFast(uint8_t pin) { return digitalRead(pin); }
  // ISRs
  void IRAM_ATTR isr_enc1() {
    int a = readPinFast(PIN_ENCODER_1A);
    int b = readPinFast(PIN_ENCODER_1B);
    if (a == b) enc1_count--;
    else enc1_count++;      // FLIPPED
  }

  void IRAM_ATTR isr_enc2() {
    int a = readPinFast(PIN_ENCODER_2A);
    int b = readPinFast(PIN_ENCODER_2B);
    if (a == b) enc2_count++;
    else enc2_count--;
  }
  
  
  void init() {
    // encoders
    pinMode(PIN_ENCODER_1A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_1B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_2A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_2B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_1A), isr_enc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_2A), isr_enc2, CHANGE);


    // motor pins
    pinMode(PIN_MOTOR_1A, OUTPUT);
    pinMode(PIN_MOTOR_1B, OUTPUT);
    pinMode(PIN_MOTOR_2A, OUTPUT);
    pinMode(PIN_MOTOR_2B, OUTPUT);

    // PWM setup
    ledcSetup(PWM_CH_M1A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_1A, PWM_CH_M1A);
    ledcSetup(PWM_CH_M1B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_1B, PWM_CH_M1B);
    ledcSetup(PWM_CH_M2A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_2A, PWM_CH_M2A);
    ledcSetup(PWM_CH_M2B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_2B, PWM_CH_M2B);

    // ensure motors stopped
    stopMotor(1);
    stopMotor(2);
  }

  void tick() {

  }


  void stopMotor(int channel) {
    if (channel == 1) {
      ledcWrite(PWM_CH_M1A, 0);
      ledcWrite(PWM_CH_M1B, 0);
    } else {
      ledcWrite(PWM_CH_M2A, 0);
      ledcWrite(PWM_CH_M2B, 0);
    }
  }


  void setMotorPwmRaw(int motorIndex, int duty8) {
    duty8 = constrain(duty8, -PWM_MAX, PWM_MAX);


    duty8 = constrain(duty8, -200, 200);  // temp slow values!!!

    if (motorIndex == 1) {
        // PWM on A, B low for forward
        if (duty8 > 0) {
            ledcWrite(PWM_CH_M1A, abs(duty8));
            ledcWrite(PWM_CH_M1B, 0);
        }
        else {
            ledcWrite(PWM_CH_M1A, 0);
            ledcWrite(PWM_CH_M1B, abs(duty8));
        }
    } else {
        // ledcWrite(PWM_CH_M2, duty8);
        // digitalWrite(PIN_MOTOR_2B, dir);
        if (duty8 > 0) {
            ledcWrite(PWM_CH_M2A, abs(duty8));
            ledcWrite(PWM_CH_M2B, 0);
        }
        else {
            ledcWrite(PWM_CH_M2A, 0);
            ledcWrite(PWM_CH_M2B, abs(duty8));
        }
    }
  }


  
}

namespace imu
{
  TwoWire &buss = Wire;


  uint8_t buf14[14];
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  float angle_pitch;
  float angle_roll;
  float angle_yaw;
  unsigned long last_tick;

  float gyro_x_bias = 0;
  float gyro_y_bias = 0;
  float gyro_z_bias = 0;

  void init(TwoWire &wire) {
    buss = wire;
    // Init IMU
    i2cWriteByte(0x6B, 0x00);  // Wake up IMU
    delay(100);
    i2cWriteByte(0x1B, 0x00);   // Gyro ±250 dps
    i2cWriteByte(0x1C, 0x00);   // Accel ±2g

    // set_gyro_bias(); // temp off

    last_tick = millis();
  }
  void set_gyro_bias() {
    // setup gyro to disable drift
    gyro_x_bias = 0;
    gyro_y_bias = 0;
    gyro_z_bias = 0;

    const int SAMPLES = 500;

    for (int i = 0; i < SAMPLES; i++) {
      tick();

      gyro_x_bias += gx;
      gyro_y_bias += gy;
      gyro_z_bias += gz;

      delay(2);
    }

    gyro_x_bias /= SAMPLES;
    gyro_y_bias /= SAMPLES;
    gyro_z_bias /= SAMPLES;

    Serial.printf("Set Gyro Bias (x, y, z) : (%.2f, %.2f, %.2f)\n", gyro_x_bias, gyro_y_bias, gyro_z_bias);
  }
  void tick() {

    // LIMIT TO SPECIFIED SAMPLE FREQ HZ FOR IMU
    // if (last_tick + 50 > millis()) {

    // }
    // unsigned long dt = millis() - last_tick;
    unsigned long t = millis();
    float dt = (t - last_tick) / 1000.0f;
    if (dt <= 0.0f) dt = 0.0001f;
    

    // read and save to local variables
    i2cReadBytes(0x3B, 14, buf14);
    // accel
    ax = (buf14[0] << 8) | buf14[1];
    ay = (buf14[2] << 8) | buf14[3];
    az = (buf14[4] << 8) | buf14[5];
    // temp skip
    gx = (buf14[8]  << 8) | buf14[9];
    gy = (buf14[10] << 8) | buf14[11];
    gz = (buf14[12] << 8) | buf14[13];
    

    float COMPL_FILTER_ALPHA = 0.98;

    float accel_pitch = atan2f((float)ay, (float)az) * 180.0f / PI;
    float gyroRate_dps_pitch = ((float)gx - gyro_x_bias) / 131.0f;
    angle_pitch = COMPL_FILTER_ALPHA * (angle_pitch + gyroRate_dps_pitch * dt) + (1.0f - COMPL_FILTER_ALPHA) * accel_pitch;

    float accel_roll = atan2f((float)ax, (float)az) * 180.0f / PI;
    float gyroRate_dps_roll = ((float)gy - gyro_y_bias) / 131.0f;
    angle_roll = COMPL_FILTER_ALPHA * (angle_roll + gyroRate_dps_roll * dt) + (1.0f - COMPL_FILTER_ALPHA) * accel_roll;
    
    float gyroRate_dps_yaw = ((float)gz - gyro_z_bias) / 131.0f;
    angle_yaw += gyroRate_dps_yaw * dt;

    if (angle_yaw > 180.0f) angle_yaw -= 360.0f;
    if (angle_yaw < -180.0f) angle_yaw += 360.0f;


    last_tick = t;
  }
  void i2cWriteByte(uint8_t reg, uint8_t data) {
    buss.beginTransmission(ADDR_MPU9265);
    buss.write(reg);
    buss.write(data);
    buss.endTransmission();
  }

  void i2cReadBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
    buss.beginTransmission(ADDR_MPU9265);
    buss.write(reg);
    buss.endTransmission(false);
    buss.requestFrom(ADDR_MPU9265, count);
    for (uint8_t i = 0; i < count; i++) {
      dest[i] = buss.read();
    }
  }

  void run_calibration() {

  }
}