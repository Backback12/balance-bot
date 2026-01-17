#ifndef CONFIG_H
#define CONFIG_H

#define PIN_ENCODER_1A 36
#define PIN_ENCODER_1B 39
#define PIN_ENCODER_2A 34
#define PIN_ENCODER_2B 32

// #define PIN_MOTOR_1A 33
// #define PIN_MOTOR_1B 26
#define PIN_MOTOR_1A 17
#define PIN_MOTOR_1B 16
#define PIN_MOTOR_2A 13
#define PIN_MOTOR_2B 27

#define PIN_BUZZER 25   // ESP DAC pin
#define PIN_BAT 33      // ESP ADC pin

#define PIN_SCL1 19 // I2C Bus #1
#define PIN_SDA1 18 // I2C Bus #1
#define PIN_SCL2 22 // I2C Bus #2
#define PIN_SDA2 21 // I2C Bus #2



// I2C Addresses
#define ADDR_MPU9265 0x68   // IMU
#define ADDR_PCA9265 0x40   // Servo driver


#endif