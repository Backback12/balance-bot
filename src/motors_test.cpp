#include "motors_test.h"
#include "movement.h"

namespace motors_test {

volatile long enc1_count = 0;
volatile long enc2_count = 0;

static inline int readPinFast(uint8_t pin) { return digitalRead(pin); }

// ISRs
// void IRAM_ATTR isr_enc1() {
//     int a = readPinFast(PIN_ENCODER_1A);
//     int b = readPinFast(PIN_ENCODER_1B);
//     if (a == b) enc1_count++;
//     else enc1_count--;
// }

// void IRAM_ATTR isr_enc2() {
//     int a = readPinFast(PIN_ENCODER_2A);
//     int b = readPinFast(PIN_ENCODER_2B);
//     if (a == b) enc2_count++;
//     else enc2_count--;
// }

// helper to set motor PWM and direction (simple forward-only demo)
void setMotorRaw(int motorIndex, uint8_t duty8, uint8_t dir = LOW) {
    duty8 = constrain(duty8, 0, PWM_MAX);
    if (motorIndex == 1) {
        // PWM on A, B low for forward
        if (dir == LOW) {
            ledcWrite(PWM_CH_M1A, duty8);
            ledcWrite(PWM_CH_M1B, 0);
        }
        else {
            ledcWrite(PWM_CH_M1A, 0);
            ledcWrite(PWM_CH_M1B, duty8);
        }
    } else {
        // ledcWrite(PWM_CH_M2, duty8);
        // digitalWrite(PIN_MOTOR_2B, dir);
        if (dir == LOW) {
            ledcWrite(PWM_CH_M2A, duty8);
            ledcWrite(PWM_CH_M2B, 0);
        }
        else {
            ledcWrite(PWM_CH_M2A, 0);
            ledcWrite(PWM_CH_M2B, duty8);
        }
    }
}

// stop motor
void stopMotor(int motorIndex) {
    if (motorIndex == 1) {
        ledcWrite(PWM_CH_M1A, 0);
        // digitalWrite(PIN_MOTOR_1B, LOW);
        ledcWrite(PWM_CH_M1B, 0);
    } else {
        ledcWrite(PWM_CH_M2A, 0);
        // digitalWrite(PIN_MOTOR_2B, LOW);
        ledcWrite(PWM_CH_M2B, 0);
    }
}

// compute motor RPM from encoder counts delta over interval_ms
float countsToMotorRPM(long counts_delta, unsigned long interval_ms) {
    if (interval_ms == 0) return 0.0f;
    float revolutions = (float)counts_delta / (ENCODER_PPR * QUADRATURE);
    float minutes = (float)interval_ms / 60000.0f;
    return revolutions / minutes; // RPM
}

// estimated output RPM (after reduction)
float motorRPMToOutputRPM(float motor_rpm) {
    return motor_rpm / REDUCTION_RATIO;
}

// initialize pins and PWM
void begin() {
    /*
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
    */
}

// Sweep PWM duty from 0..100% and print measured output RPM.
// motorIndex: 1 or 2. sample_ms: how long to sample at each duty step.
void runSweep(int motorIndex = 1, unsigned long sample_ms = 800, int stepPercent = 1, int startStep = 0) {
    begin();
    Serial.println("Starting PWM sweep test...");
    for (int p = startStep; p <= 100; p += stepPercent) {
        // map percent to duty (0..PWM_MAX)
        uint8_t duty = (uint8_t)map(p, 0, 100, 0, PWM_MAX);
        // clear encoder count
        noInterrupts();
        if (motorIndex == 1) enc1_count = 0;
        else enc2_count = 0;
        interrupts();

        setMotorRaw(motorIndex, duty);

        unsigned long start = millis();
        delay(sample_ms);
        unsigned long elapsed = millis() - start;

        long counts_delta = 0;
        noInterrupts();
        counts_delta = (motorIndex == 1) ? enc1_count : enc2_count;
        interrupts();

        float motor_rpm = countsToMotorRPM(counts_delta, elapsed);
        float output_rpm = motorRPMToOutputRPM(motor_rpm);

        Serial.printf("PWM %3d%% (%3u/255)  counts=%ld  motorRPM=%.1f  outputRPM=%.1f\n",
                      p, (unsigned int)duty, counts_delta, motor_rpm, output_rpm);
        // small cooldown between steps
        stopMotor(motorIndex);
        delay(150);
    }
    Serial.println("Sweep complete.");
    stopMotor(motorIndex);
}

// Simple closed-loop controller driving motor to target output RPM.
// motorIndex: 1 or 2. targetOutputRPM: desired RPM at gearbox output.
// runTimeMs: how long to run the controller.
void startClosedLoop(int motorIndex, float targetOutputRPM, unsigned long runTimeMs = 5000) {
    // begin();
    Serial.printf("Starting closed-loop test: motor=%d targetOutputRPM=%.1f\n", motorIndex, targetOutputRPM);
    // convert to motor RPM (encoder measures motor)
    float targetMotorRPM = targetOutputRPM * REDUCTION_RATIO;

    // controller state
    float duty = 50.0f; // start with an initial duty (0..PWM_MAX)
    float duty8 = map((int)constrain((int)duty, 0, 100), 0, 100, 0, PWM_MAX); // initial mapped
    const float Kp = 0.5f; // proportional gain (tweak)
    const float Ki = 0.8f; // integral gain (tweak)
    float integral = 0.0f;

    unsigned long t0 = millis();
    unsigned long lastSample = t0;
    unsigned long sampleMs = 5; // controller sample time

    // zero encoder counts
    noInterrupts();
    if (motorIndex == 1) enc1_count = 0;
    else enc2_count = 0;
    interrupts();

    while (millis() - t0 < runTimeMs) {
        unsigned long now = millis();
        if (now - lastSample < sampleMs) {
            // delay(5);
            continue;
        }
        unsigned long dt = now - lastSample;
        lastSample = now;

        // read counts
        long counts_delta = 0;
        noInterrupts();
        if (motorIndex == 1) {
            counts_delta = enc1_count;
            enc1_count = 0;
        } else {
            counts_delta = enc2_count;
            enc2_count = 0;
        }
        interrupts();

        float motor_rpm = countsToMotorRPM(counts_delta, dt);
        float output_rpm = motorRPMToOutputRPM(motor_rpm);

        float error = targetMotorRPM - motor_rpm;
        integral += error * (dt / 1000.0f);

        // simple PI -> adjust duty percent (0..100)
        float adjust = Kp * error + Ki * integral;
        float dutyPercent = constrain(duty + adjust * 0.02f, 0.0f, 100.0f); // scale adjust
        duty = dutyPercent; // store percent
        uint8_t duty_mapped = (uint8_t)map((int)dutyPercent, 0, 100, 0, PWM_MAX);
        setMotorRaw(motorIndex, duty_mapped);

        Serial.printf("t=%lums pwm=%3d%% motorRPM=%.1f outRPM=%.1f err=%.1f\n",
                      millis() - t0, (int)dutyPercent, motor_rpm, output_rpm, (targetMotorRPM - motor_rpm));
    }

    stopMotor(motorIndex);
    Serial.println("Closed-loop run complete.");
}

} // namespace motors_test

// Example usage hint (call from your main):
// void setup() {
//   Serial.begin(115200);
//   motors_test::runSweep(1, 800, 1); // sweep motor 1, 800ms per step, 1% step
//   // or:
//   // motors_test::startClosedLoop(1, 50.0); // target 50 RPM at output
// }
//
// void loop() { /* empty or other tasks