/*
https://github.com/evertonramires/CuteBuzzerSounds/blob/master/src/Sounds.h
https://github.com/FluxGarage/RoboEyes/blob/main/examples/i2c_SSD1306_AnimationSequences/i2c_SSD1306_AnimationSequences.ino
*/


#include <Arduino.h>

#include "config.h"
// #include "motors_test.h"
// #include "testing.h"
#include "movement.h"
#include "controls.h"
#include "cosmetics.h"
#include "PID.h"

TaskHandle_t Core0Task;
TaskHandle_t Core1Task;
void Core0Loop(void *pvParameters);
void Core1Loop(void *pvParameters);


//------------------- ROLL -------------------
// make servos differential to affect roll
//
float target_roll = 0.0;  // change when "skating"?
float current_leg_extension = 30;
float roll_P = 1.30f, roll_I = 0.0015f, roll_D = 0.5f; // SET THROUGH TESTING

// --- PID Functions ---
float getRoll() {
  // Serial.print("Roll: ");
  // Serial.print(imu::angle_roll);
  return imu::angle_roll; 
}

void applyRollOutput(float output) {
  // Serial.print("  Servo offset: ");
  // Serial.println(output);
  servos::legsMove(current_leg_extension, output);
}

PIDController<float> rollPID(roll_P, roll_I, roll_D, getRoll, applyRollOutput);




//------------------- MAIN PITCH PID -------------------
float target_pitch = 10.5; // close found through testing
// float pitch_P = 4.0, pitch_I = 0.02, pitch_D = 0.08;
// float pitch_P = 6.0, pitch_I = 0.06, pitch_D = 0.03;
float pitch_P = 8.0, pitch_I = 0.085, pitch_D = 0.025;

float getPitch() {
  // Serial.print("\tPitch: ");
  // Serial.print(imu::angle_pitch);

  return imu::angle_pitch;
}

float live_motor_output = 0;  // for wifi debug thing
/*
void applyPitchOutput(float output) {
  // // Combine Pitch balance with Yaw correction
  // int left_motor = static_cast<int>(output + yaw_adjustment);
  // int right_motor = static_cast<int>(output - yaw_adjustment);

  // // Apply motor limits and deadzone logic as you did before
  // motors::setMotorPwmRaw(1, left_motor);
  // motors::setMotorPwmRaw(2, right_motor);

  // Output range -135 to 135
  int motor_speed = static_cast<int>(output);


  int after_dead_space = motor_speed + ((motor_speed > 0) ? 120 : -120);

  // int spd1 = after_dead_space + yaw_adjustment;
  // int spd2 = after_dead_space - yaw_adjustment;
  int spd1 = after_dead_space;
  int spd2 = after_dead_space;

  // motors::setMotorPwmRaw(1, after_dead_space);

  // Serial.print("\tSpeed: ");
  // Serial.print(spd1);
  
  // Drive both wheels together to balance
  motors::setMotorPwmRaw(1, spd1); // Left Motor
  motors::setMotorPwmRaw(2, spd2); // Right Motor

  live_motor_output = spd1;
}
*/
void applyPitchOutput(float output) {
    // 'output' is the balancing effort from Pitch PID
    int motor_base = static_cast<int>(output);
    
    // Apply deadzone only if there is actual intended movement
    if (abs(motor_base) > 1) {
        motor_base += (motor_base > 0) ? 120 : -120;
    }

    // Combine with Yaw Adjustment (from PID + Manual Input)
    // int left_motor = motor_base + yaw_adjustment;
    // int right_motor = motor_base - yaw_adjustment;
    int left_motor = motor_base;
    int right_motor = motor_base;

    motors::setMotorPwmRaw(1, left_motor);
    motors::setMotorPwmRaw(2, right_motor);

    live_motor_output = motor_base; 
}

PIDController<float> pitchPID(pitch_P, pitch_I, pitch_D, getPitch, applyPitchOutput);






//------------------- Adjust target pitch PID -------------------
// measured around 9.5 or 10.0deg pitch
// reads average velocity to adjust target pitch
// float current_target_pitch = 10.0; 
float current_target_pitch = 6.0; 
float yaw_adjustment = 0.0;
float target_yaw = 0.0; 
// float avg_rpm = 0.0;

float getAverageRPM() {
  // return (motors::read_rpm(1) + motors::read_rpm(2)) / 2.0f;
  // float average = (motors::read_rpm(1) + motors::read_rpm(2)) / 2.0f;
  // avg_rpm = 0.5 *avg_rpm + 0.5 * ((motors::read_rpm(1) + motors::read_rpm(2)) / 2.0f);
  // avg_rpm = 0.5 *avg_rpm + 0.5 * ((motors::read_encoder(1) + motors::read_encoder(2)) / 2.0f);
  float avg_rpm = (motors::read_encoder(1) + motors::read_encoder(2)) / 2.0f;
  // Serial.print("\tAverage ticks: ");
  // Serial.print(avg_rpm);

  return avg_rpm;
}

void applyVelocityOutput(float output) {
  // output = 9.5f; // lol hardcode it for now bruh
  // Serial.print("\tTarget Pitch: ");
  // Serial.print(output);

  current_target_pitch = output; 

}

// float vel_p = 0.55, vel_i = 0.0000, vel_d = 0.001;
// float vel_p = 0.002, vel_i = 0.001, vel_d = 0.000;
float vel_p = 0.001, vel_i = 0.001, vel_d = 0.2;
PIDController<float> velocityPID(vel_p, vel_i, vel_d, getAverageRPM, applyVelocityOutput);


//------------------- Yaw PID -------------------
// applies differential to motors to cancel yaw drift?
float getYaw() {
  return imu::angle_yaw; 
}

void applyYawOutput(float output) {
  yaw_adjustment = output;
}

float yaw_p = 1.5, yaw_i = 0.05, yaw_d = 0.2;
PIDController<float> yawPID(yaw_p, yaw_i, yaw_d, getYaw, applyYawOutput);


// RemoteTuner tuner(pitchPID, velocityPID);
RemoteTuner tuner(
  pitchPID, 
  velocityPID, 
  &imu::angle_pitch, 
  &current_target_pitch, 
  &live_motor_output,

  &current_leg_extension,
  &servos::left_percent,
  &servos::right_percent,
  &servos::left_angle,
  &servos::right_angle
);



Cosmetics robotHead(&Wire1);


// potential testing vals:
//pitch pid:  6.0, 0.06, 0.03
//vel pid:    0.001, 0.001, 0.2

void setup() {
  Serial.begin(115200);
  
  
  // inits IMU, motors, servos
  movement::init();

  rollPID.registerTimeFunction(millis);
  rollPID.setOutputBounds(-20.0f, 20.0f);
  rollPID.setTarget(target_roll);

  pitchPID.registerTimeFunction(millis);
  pitchPID.setOutputBounds(-135, 135);  // max speed minus dead zone
  pitchPID.setTarget(target_pitch);



  velocityPID.registerTimeFunction(millis);
  velocityPID.setOutputBounds(-4.0, 16.0); // 10 deg is close?
  velocityPID.setTarget(0); 

  target_yaw = imu::angle_yaw;
  yawPID.setTarget(target_yaw);
  yawPID.setOutputBounds(-1, 1);  // change
  yawPID.registerTimeFunction(millis);

  // Pitch Loop
  pitchPID.setOutputBounds(-135, 135);
  pitchPID.registerTimeFunction(millis);


  /*
  unsigned long lastUpdateTime = 0;
  const int updateInterval = 10; // 10ms = 100Hz


  while (true) {
    if (millis() - lastUpdateTime >= updateInterval) {
      lastUpdateTime = millis();

      imu::tick();
      
      // check limit for falling
      if (imu::angle_pitch < -25.0 || imu::angle_pitch > 35.0) {
        motors::setMotorPwmRaw(1, 0);
        motors::setMotorPwmRaw(2, 0);
        while (imu::angle_pitch < -25.0 || imu::angle_pitch > 35.0) {
          imu::tick();
        }
        delay(1000);
      }

      velocityPID.tick();
      // yawPID.tick();
      pitchPID.setTarget(current_target_pitch);
      pitchPID.tick();
      rollPID.tick();

      Serial.println(""); // newline for any functions printing

    }

  }
  */
  
  xTaskCreatePinnedToCore(
    Core0Loop,
    "Core0",
    8192,  // stack size?
    NULL, // task parameters
    1,  // task priority
    &Core0Task,
    0 // set to core 0
  );
  xTaskCreatePinnedToCore(
    Core1Loop,
    "Core1",
    8192,  // stack size?
    NULL, // task parameters
    1,  // task priority
    &Core1Task,
    1 // set to core 0
  );
  
}

void loop() 
{
   // blank loop cause tasks do the loops
}



// CORE 0 - Movement
void Core0Loop(void *pvParameters) {
  // setup...
  // movement::init();
  servos::headMove(90, 90);

  unsigned long temp_start = millis();
  while (millis() < temp_start + 500) {
    // temp wait for oled to run?
  }

  for(;;) {
    // movement::loop();

    // imu update (at freq)

    // roll update (at freq)

    // pitch update (at freq)
    


    // handle communication between cores...
    // receive controls
    // receive any cosmetics


    
    /*
    imu::tick();
    
    // check limit for falling
    if (imu::angle_pitch < -25.0 || imu::angle_pitch > 35.0) {
      motors::setMotorPwmRaw(1, 0);
      motors::setMotorPwmRaw(2, 0);
      while (imu::angle_pitch < -25.0 || imu::angle_pitch > 35.0) {
        imu::tick();
      }
      delay(1000);
    }

    velocityPID.tick();
    // yawPID.tick();
    pitchPID.setTarget(current_target_pitch);
    pitchPID.tick();
    rollPID.tick();

    Serial.println(""); // newline for any functions printing

    vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz frequency
    */

    imu::tick();

    // 1. GET INPUTS FROM TUNER
    // Convert normalized -1.0...1.0 to actual target units
    // For velocity, this might be target encoder ticks per 10ms
    // float moveInput = tuner.getTargetMove(); // -1.0 to 1.0
    // float turnInput = tuner.getTargetTurn(); // -1.0 to 1.0
    float moveInput = tuner._targetMove; // -1.0 to 1.0
    float turnInput = tuner._targetTurn; // -1.0 to 1.0
    

    // 2. FORWARD MOVEMENT
    // Instead of targeting 0 speed, we target the moveInput
    // We scale moveInput by a 'max speed' factor (e.g., 20 ticks)
    velocityPID.setTarget(moveInput * 20.0f); 
    velocityPID.tick();

    // 3. PITCH BALANCE
    pitchPID.setTarget(current_target_pitch); // This is the output from velocityPID
    pitchPID.tick();

    // 4. TURNING (Yaw)
    // We can simply add the turnInput to the yaw adjustment
    float finalYawAdj = yaw_adjustment + (turnInput * 40.0f); // Scale 40 for turn strength

    // 5. FINAL MOTOR OUTPUT (Update applyPitchOutput to use finalYawAdj)
    // (See updated applyPitchOutput below)

    rollPID.tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// CORE 1 - Cosmetics
void Core1Loop(void *pvParameters) {
  // setup...
  // controls::init();
  // cosmetics::init();
  // return;
  // for(;;) {}

  robotHead.begin(PIN_BUZZER, PIN_SDA2, PIN_SCL2);


  // temp run oled for 5 seconds
  unsigned long temp_start = millis();
  while (millis() < temp_start + 500) {
    robotHead.tick();
  }
  robotHead.tick();

  // ToF::init(Wire1);  // make robot head expose the wire as well bru


  tuner.begin("Backback12-2.4", "password");  // connor here is wifi password pls remember to delete
  // tuner.begin("Zapper4000", "password");  // connor here is wifi password pls remember to delete

  for(;;) {
    // controls::loop(); // does the wifi controls?
    // cosmetics::loop();
    robotHead.tick();

    tuner.handle();

    // handle communication between cores...
    // transmit controls
    // receive any cosmetics

    // uint16_t d = ToF::tofRead(0);
    // Serial.print("Distance: ");
    // Serial.println(d);

    // vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz frequency
    vTaskDelay(pdMS_TO_TICKS(10)); // 200Hz frequency
  }
}

