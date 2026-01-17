#include "testing.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <BuzzerSounds.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>


namespace testing {

// --- Motor wrappers --------------------------------------------------------
void runMotorSweep(int motorIndex, unsigned long sample_ms, int stepPercent, int startStep) {
    motors_test::runSweep(motorIndex, sample_ms, stepPercent, startStep);
}

void runMotorClosedLoop(int motorIndex, float targetOutputRPM, unsigned long runTimeMs) {
    motors_test::startClosedLoop(motorIndex, targetOutputRPM, runTimeMs);
}

// --- IMU test --------------------------------------------------------------
/*
// This does a simple I2C scan then, if an MPU-like device is found (0x68/0x69),
// repeatedly reads accel/gyro registers (MPU register map) and prints them.
// It's intentionally simple: adjust scaling / register addresses for your sensor.
void imuTest(uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, TwoWire &wire) {
    wire.begin(i2c_sda_pin, i2c_scl_pin);
    // Serial.println("I2C scanning for devices...");
    // int found = 0;
    // for (uint8_t addr = 1; addr < 127; ++addr) {
    //     wire.beginTransmission(addr);
    //     if (wire.endTransmission() == 0) {
    //         Serial.printf("I2C device found at 0x%02X\n", addr);
    //         found++;
    //     }
    // }
    // if (found == 0) Serial.println("No I2C devices found.");

    // // prefer common MPU addresses 0x68 and 0x69
    // uint8_t imu_addr = 0;
    // for (uint8_t a : { (uint8_t)0x68, (uint8_t)0x69 }) {
    //     wire.beginTransmission(a);
    //     if (wire.endTransmission() == 0) { imu_addr = a; break; }
    // }

    uint8_t imu_addr = ADDR_MPU9265;

    if (imu_addr == 0) {
        Serial.println("No MPU-like device at 0x68/0x69. Still running scan loop.");
    } else {
        Serial.printf("Using IMU at 0x%02X\n", imu_addr);
        // try waking sensor (MPU PWR_MGMT_1 = 0x6B -> set 0)
        wire.beginTransmission(imu_addr);
        wire.write(0x6B);
        wire.write(0x00);
        wire.endTransmission();
    }

    // continuous read loop (infinite)
    while (true) {
        if (imu_addr != 0) {
            // read accel registers 0x3B..0x40 (6 bytes), then gyro 0x43..0x48 (6 bytes)
            // this matches MPU9250/MPU6050 family layout
            wire.beginTransmission(imu_addr);
            wire.write(0x3B); // ACCEL_XOUT_H
            if (wire.endTransmission(false) == 0 && wire.requestFrom((int)imu_addr, 14) == 14) {
                int16_t ax = (wire.read() << 8) | wire.read();
                int16_t ay = (wire.read() << 8) | wire.read();
                int16_t az = (wire.read() << 8) | wire.read();
                int16_t temp = (wire.read() << 8) | wire.read();
                int16_t gx = (wire.read() << 8) | wire.read();
                int16_t gy = (wire.read() << 8) | wire.read();
                int16_t gz = (wire.read() << 8) | wire.read();

                // basic scaling assumptions for debug: accel LSB/g = 16384 (±2g)
                float ax_g = ax / 16384.0f;
                float ay_g = ay / 16384.0f;
                float az_g = az / 16384.0f;

                Serial.printf("AX=%.3fg AY=%.3fg AZ=%.3fg  GX=%d GY=%d GZ=%d  T=%d\n",
                              ax_g, ay_g, az_g, gx, gy, gz, temp);
            } else {
                Serial.println("Failed to read IMU registers (device may differ)");
            }
        } else {
            // If no imu found at 68/69, do a periodic scan summary
            Serial.println("No MPU found at 0x68/0x69. Re-scan I2C for devices...");
            for (uint8_t addr = 1; addr < 127; ++addr) {
                wire.beginTransmission(addr);
                if (wire.endTransmission() == 0) {
                    Serial.printf("  device 0x%02X\n", addr);
                }
            }
        }
        delay(500);
    }
}*/
void imuTest(uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, TwoWire &wire) {

    // wire.begin(i2c_sda_pin, i2c_scl_pin);
    // wire.setPins(18, 19);
    wire.begin(18, 19);
    wire.setClock(400000); // Fast I2C

    Serial.begin(115200);
    delay(1000);

    // Wake up IMU
    i2cWriteByte(wire, 0x6B, 0x00);  // PWR_MGMT_1 = 0
    delay(100);

    // Gyro ±250 dps
    i2cWriteByte(wire, 0x1B, 0x00);

    // Accel ±2g
    i2cWriteByte(wire, 0x1C, 0x00);

    Serial.println("MPU9265 / MPU9250 IMU Test");

    uint8_t rawData[14];

    while (true) {
        i2cReadBytes(wire, 0x3B, 14, rawData);

        int16_t ax = (rawData[0] << 8) | rawData[1];
        int16_t ay = (rawData[2] << 8) | rawData[3];
        int16_t az = (rawData[4] << 8) | rawData[5];

        int16_t gx = (rawData[8]  << 8) | rawData[9];
        int16_t gy = (rawData[10] << 8) | rawData[11];
        int16_t gz = (rawData[12] << 8) | rawData[13];

        // Scale factors
        float accelScale = 16384.0; // LSB/g
        float gyroScale  = 131.0;   // LSB/(deg/s)

        float ax_g = ax / accelScale;
        float ay_g = ay / accelScale;
        float az_g = az / accelScale;

        float gx_dps = gx / gyroScale;
        float gy_dps = gy / gyroScale;
        float gz_dps = gz / gyroScale;

        Serial.print("Accel (g): ");
        Serial.print(ax_g, 3); Serial.print(", ");
        Serial.print(ay_g, 3); Serial.print(", ");
        Serial.print(az_g, 3);

        Serial.print(" | Gyro (dps): ");
        Serial.print(gx_dps, 3); Serial.print(", ");
        Serial.print(gy_dps, 3); Serial.print(", ");
        Serial.println(gz_dps, 3);

        delay(100); // ~100 Hz
    }
}
void i2cWriteByte(TwoWire &wire, uint8_t reg, uint8_t data) {
    wire.beginTransmission(ADDR_MPU9265);
    wire.write(reg);
    wire.write(data);
    wire.endTransmission();
}

void i2cReadBytes(TwoWire &wire, uint8_t reg, uint8_t count, uint8_t *dest) {
    wire.beginTransmission(ADDR_MPU9265);
    wire.write(reg);
    wire.endTransmission(false);
    wire.requestFrom(ADDR_MPU9265, count);
    for (uint8_t i = 0; i < count; i++) {
        dest[i] = wire.read();
    }
}

// --- Servo test ------------------------------------------------------------
/*
void servoTest(uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, TwoWire &wire) {
    // PCA9685 default address 0x40
    const uint8_t PCA_ADDR = 0x40;
    // const uint8_t PCA_ADDR = 0x14;
    // wire.begin(i2c_sda_pin, i2c_scl_pin);
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR, wire);

    Serial.println("Starting PCA9685 servo test...");
    pwm.begin();
    pwm.setPWMFreq(50); // typical for hobby servos

    // pulse mapping parameters (tweak if your servos require different pulse widths)
    const int servoMin = 150; // ~0.75ms
    const int servoMax = 600; // ~3.0ms

    for (int angle = 0; angle <= 180; angle += 5) {
        int pulse = map(angle, 0, 180, servoMin, servoMax);
        pwm.setPWM(0, 0, pulse);
        pwm.setPWM(1, 0, pulse);
        Serial.printf("Servos -> angle=%d pulse=%d\n", angle, pulse);
        delay(150);
    }
    // sweep back
    for (int angle = 180; angle >= 0; angle -= 5) {
        int pulse = map(angle, 0, 180, servoMin, servoMax);
        pwm.setPWM(0, 0, pulse);
        pwm.setPWM(1, 0, pulse);
        delay(150);
    }

    // turn servos off (0 pulses)
    pwm.setPWM(0, 0, 0);
    pwm.setPWM(1, 0, 0);
    Serial.println("Servo test complete.");
}*/
constexpr uint16_t SERVO_MIN_US = 500;   // 0 degrees
constexpr uint16_t SERVO_MAX_US = 2500;  // 180 degrees
constexpr uint16_t SERVO_FREQ   = 50;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // default I2C addr
void servoTest(uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, TwoWire &wire) {

    Serial.println("Starting servo test...");

    // Wire.setPins(sdaPin, sclPin);
    wire.setPins(18, 19);
    wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);   // recommended?
    // pwm.setOscillatorFrequency(25000000);   // recommended?
    pwm.setPWMFreq(50);
    
    Serial.println("Moving servos...");

    // servoWrite(0, 90);
    // servoWrite(1, 90);
    pwm.writeMicroseconds(0, 500);
    delay(500);

    for (int angle = 0; angle <= 180; angle++) {
        // servoWrite(0, angle);
        // servoWrite(1, angle);
        pwm.writeMicroseconds(0, map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
        delay(40);
    }
    for (int angle = 180; angle >= 0; angle--) {
        // servoWrite(0, angle);
        // servoWrite(1, angle);
        pwm.writeMicroseconds(0, map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
        delay(80);
    }

    // servoWrite(0, 90);
    // servoWrite(1, 90);
    pwm.writeMicroseconds(0, 1500);

    Serial.println("Done moving servos");
}
uint16_t microsecondsToTicks(uint16_t us) {
    float period_us = 1000000.0f / SERVO_FREQ;   // 20,000 µs at 50 Hz
    return (uint16_t)((us / period_us) * 4096.0f);
}   
void servoWrite(uint8_t channel, uint8_t angle) {
    angle = constrain(angle, 0, 180);

    uint16_t pulse_us = map(
        angle,
        0, 180,
        SERVO_MIN_US,
        SERVO_MAX_US
    );

    // uint16_t ticks = microsecondsToTicks(pulse_us);
    // pwm.setPWM(channel, 0, ticks);
    // pwm.setPWM(channel, 0, pulse_us);

    // is this "stable"?
    pwm.writeMicroseconds(channel, pulse_us);
}
void legMove(uint8_t channel, uint8_t percent) {
    percent = constrain(percent, 0, 100);
    uint8_t angle = map(
        percent,
        0, 100,
        //10, 70
        10, 60
    );
    // angle = constrain(angle, 10, 70);

    // found 10-70 range before, might need to adjust direct offsets
    // but sticking with 10-60 for testing, extending too far

    uint16_t pulse_us = map(
        angle,
        0, 180,
        SERVO_MIN_US, SERVO_MAX_US
    );
    uint16_t pulse_us_inv = map(
        angle,
        0, 180,
        SERVO_MAX_US, SERVO_MIN_US
    );



    // is pwm.writeMicroseconds() "stable"?
    if (channel == 2) {
        pwm.writeMicroseconds(0, pulse_us_inv);
        pwm.writeMicroseconds(1, pulse_us);
    }
    else if (channel == 1) {
        pwm.writeMicroseconds(channel, pulse_us);
    }
    else {
        pwm.writeMicroseconds(channel, pulse_us_inv);
    }
}
void servoLegTest(uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, TwoWire &wire) {

    Serial.println("Starting servo leg test...");
    // Wire.setPins(sdaPin, sclPin);
    // wire.setPins(18, 19);
    // wire.begin();

    wire.begin(18, 19);

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);   // recommended?
    pwm.setPWMFreq(50);
    

    // oscillate legs
    for (int i = 0; i < 2; i++) {
        for (int x = 0; x < 360; x++) {
            
            double y = (1 - cos(x * PI / 180))/2 * 100; // oscillate 0-100%
            legMove(2, y);
            delay(5);

        }
    }


    Serial.println("Fully extended");
    legMove(2, 100);
    delay(1000);
    Serial.println("Fully retracted");
    legMove(2, 0);
    delay(1000);
    // Serial.println("Fully extended");
    // legMove(2, 100);
    // delay(1000);
    // Serial.println("Fully retracted");
    // legMove(2, 0);
    // delay(1000);
    // Serial.println("Fully extended");
    // legMove(2, 100);
    // delay(1000);
    // Serial.println("Fully retracted");
    // legMove(2, 0);
    // delay(1000);
    // Serial.println("Fully extended");
    // legMove(2, 100);
    // delay(1000);
    // Serial.println("Fully retracted");
    // legMove(2, 0);
    // delay(1000);


    

    Serial.println("Done moving servos");
}



void oledBuzzerTest() {
    float last_tick = 0;
    int iteration = 0;

    BuzzerSounds bs;
    Adafruit_SSD1306 display(128, 64, &Wire, -1);

    
    // create a RoboEyes instance using an Adafruit_SSD1306 display driver
    RoboEyes<Adafruit_SSD1306> roboEyes(display); 


    // setup
    bs.begin(PIN_BUZZER);

    // OLED Display
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C or 0x3D
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    // Startup robo eyes
    roboEyes.begin(128, 64, 100); // screen-width, screen-height, max framerate - 60-100fps are good for smooth animations
    roboEyes.setPosition(DEFAULT); // eye position should be middle center
    roboEyes.close(); // start with closed eyes 
    roboEyes.setAutoblinker(true);
    roboEyes.setIdleMode(true, 5, 1);
    roboEyes.setWidth(26, 26);
    roboEyes.setHeight(32, 32);
    roboEyes.setSpacebetween(50);
    roboEyes.setBorderradius(13, 13);
    // roboEyes.setCuriosity(true);

    bs.play(1);

    while (true) {
        roboEyes.update(); // update eyes drawings
        bs.update();

        if (millis() > last_tick + 3000)
        {
            switch (iteration)
            {
                case 0:
                    Serial.println("Sound test");
                    bs.play(0, 100); // SOUND_TEST
                    // roboEyes.anim_laugh();
                    roboEyes.setMood(HAPPY);
                break;
                    case 1:
                    Serial.println("Sound happy");
                    bs.play(1); // SOUND_HAPPY
                    roboEyes.anim_laugh();
                    break;
                case 2:
                    Serial.println("Sound sad");
                    bs.play(2); // SOUND_SAD
                    // roboEyes.anim_confused();
                    roboEyes.setMood(TIRED);
                    break;
                case 3:
                    Serial.println("Sound confused");
                    roboEyes.setMood(DEFAULT);
                    bs.play(3); // SOUND_CONFUSED
                    roboEyes.anim_confused();
                    break;
                case 4:
                    Serial.println("Sound cos");
                    bs.play(4, 50); // SOUND_COS
                    // roboEyes.setMood()
                    roboEyes.anim_laugh();
                    break;
            }
            last_tick = millis();
            iteration = (iteration + 1) % 5;
        }
    }
}
// --- Combined test --------------------------------------------------------
void runAllTests() {
    // 1) quick motor sweep
    Serial.println("=== Motor sweep (motor 1) ===");
    runMotorSweep(1, 500, 1, 50);

    // 2) servo test
    Serial.println("=== Servo test ===");
    servoTest();

    // 3) enter IMU test (infinite)
    Serial.println("=== IMU test (entering loop) ===");
    imuTest();

    // 4) oled and buzzer test (infinite)
    // Serial.println("=== OLED BUZZER TEST ===");
    // oledBuzzerTest();
}








// --- new: balance parameters (tweak these as needed) ----------------------
static constexpr uint8_t BALANCE_IMU_ADDR = 0x68;    // fallback IMU I2C addr (MPU9250/MPU9265)
static constexpr uint16_t BALANCE_CAL_SAMPLES = 300; // samples to compute offset at startup
static constexpr uint16_t BALANCE_CAL_DELAY_MS = 8;  // delay between calibration samples
// static constexpr uint16_t BALANCE_CONTROL_PERIOD_MS = 150; // controller period per iteration
static constexpr uint16_t BALANCE_CONTROL_PERIOD_MS = 20; // controller period per iteration
static constexpr float COMPL_FILTER_ALPHA = 0.98f;  // complementary filter alpha
// Map angle error (deg) to target output RPM: targetRPM = ANGLE_TO_RPM * angle_error
static constexpr float ANGLE_TO_RPM = 16.0f; // initial gain (tune on robot)
// Deadband (deg) around offset where we command zero RPM
static constexpr float ANGLE_DEADBAND_DEG = 1.0f;

// prototypes for local helpers (i2c helpers already exist in this file)
float readAccelPitchDeg(TwoWire &wire);
float calibrateIMUOffset(TwoWire &wire, uint16_t samples, uint16_t delayMs);

// Public API: calibrateIMUOffset (also declared in header)
float calibrateIMUOffset(TwoWire &wire, uint16_t samples, uint16_t delayMs) {
    float sum = 0.0f;
    uint16_t valid = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        // read accel X,Z to compute pitch
        uint8_t buf[6];
        // ACCEL_XOUT_H = 0x3B
        i2cReadBytes(wire, 0x3B, 6, buf);
        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];
        // compute pitch: atan2(ax, az)
        float pitch = atan2f((float)ax, (float)az) * 180.0f / PI;
        if (!isnan(pitch) && isfinite(pitch)) {
            sum += pitch;
            valid++;
        }
        delay(delayMs);
    }
    if (valid == 0) return 0.0f;
    return sum / (float)valid;
}

// read current pitch (deg) from accel registers (simple accel-based angle)
float readAccelPitchDeg(TwoWire &wire) {
    uint8_t buf[6];
    i2cReadBytes(wire, 0x3B, 6, buf);
    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    float pitch = atan2f((float)ax, (float)az) * 180.0f / PI;
    return pitch;
}

// balance(): blocking loop that reads IMU, computes angle error vs calibrated offset,
// maps that to a target output RPM and uses motors_test::startClosedLoop to drive motors.
// motorIndex: 1 or 2 (use 0/1 if your motors_test uses 1/2). This implementation uses
// the existing closed-loop routine for short intervals to simplify integration.
void balance(uint8_t motorIndex, TwoWire &wire) {
    // initialize I2C (use default pins configured elsewhere; allow user-provided wire)
    wire.begin();         // if user wants custom pins, call wire.begin(sda, scl) prior
    wire.setClock(400000); // fast i2c

    // Wake IMU (best-effort)
    i2cWriteByte(wire, 0x6B, 0x00); // PWR_MGMT_1 = 0

    Serial.println("Calibrating IMU offset (assumes robot is level/steady on start)...");
    float offset = calibrateIMUOffset(wire, BALANCE_CAL_SAMPLES, BALANCE_CAL_DELAY_MS);
    Serial.printf("Calibration complete. pitch_offset=%.2f deg\n", offset);

    // complementary filter state
    float angle_est = offset; // start at offset
    unsigned long last_t = millis();

    // main balancing loop (blocking)
    while (true) {
        unsigned long t = millis();
        float dt = (t - last_t) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;

        // read gyro Z or Y depending on orientation. Using GX (reg 0x43) as example:
        // read gyro X (0x43..0x48) and accel (0x3B..)
        uint8_t buf14[14];
        i2cReadBytes(wire, 0x3B, 14, buf14);
        // accel
        int16_t ax = (buf14[0] << 8) | buf14[1];
        int16_t ay = (buf14[2] << 8) | buf14[3];
        int16_t az = (buf14[4] << 8) | buf14[5];
        // temp skip
        int16_t gx = (buf14[8]  << 8) | buf14[9];
        int16_t gy = (buf14[10] << 8) | buf14[11];
        int16_t gz = (buf14[12] << 8) | buf14[13];

        // accel angle (deg)
        // float accel_pitch = atan2f((float)ax, (float)az) * 180.0f / PI;
        float accel_pitch = atan2f((float)ay, (float)az) * 180.0f / PI;

        // choose gyro axis (depends on orientation). Use gy (gyro Y) as angular rate around pitch axis in deg/s.
        // MPU gyro LSB for ±250 dps is 131 LSB/(deg/s)
        // float gyroRate_dps = (float)gy / 131.0f;
        float gyroRate_dps = (float)gx / 131.0f;

        // complementary filter to estimate angle
        angle_est = COMPL_FILTER_ALPHA * (angle_est + gyroRate_dps * dt) + (1.0f - COMPL_FILTER_ALPHA) * accel_pitch;

        float angle_error = angle_est - offset;

        // deadband: if within small angle, do not command motors
        float targetOutputRPM = 0.0f;
        if (fabs(angle_error) > ANGLE_DEADBAND_DEG) {
            // map angle error to RPM using simple proportional mapping. sign: positive error -> forward/backwards
            targetOutputRPM = ANGLE_TO_RPM * angle_error;
            // clamp to safe RPM range (user tweak)
            if (targetOutputRPM > 400.0f) targetOutputRPM = 400.0f;
            if (targetOutputRPM < -400.0f) targetOutputRPM = -400.0f;
        }

        // Print debug
        Serial.printf("t=%.2f dt=%.3f angle=%.2f accel=%.2f gyro=%.2f err=%.2f targetRPM=%.2f\n",
                      (t/1000.0f), dt, angle_est, accel_pitch, gyroRate_dps, angle_error, targetOutputRPM);

        // Use the existing closed-loop motor routine to command desired output RPM for a short period.
        // startClosedLoop expects positive targetOutputRPM; it uses the encoder before reduction.
        // We call it for a short control window to let the PI controller act continuously.
        // The function in motors_test expects runTimeMs (blocking). Use BALANCE_CONTROL_PERIOD_MS.
        // Call with absolute RPM for now; direction handling is internal to your motor driver mapping.
        
        motors_test::startClosedLoop(motorIndex, fabs(targetOutputRPM), BALANCE_CONTROL_PERIOD_MS);
        
        // test with raw
        // motors_test::setMotorRaw(2, map(targetOutputRPM, -200, 200, 0, 255));
        // uint8_t dir = LOW;
        // if (targetOutputRPM > 0) {
        //     dir = HIGH;
        // }

        // motors_test::setMotorRaw(1, 
        //     map(abs(targetOutputRPM), 0, 200, 128, 255), 
        //     dir
        // );
        

        last_t = t;
        // loop continues
    }
}

}