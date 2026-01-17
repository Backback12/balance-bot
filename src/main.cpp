/*
https://github.com/evertonramires/CuteBuzzerSounds/blob/master/src/Sounds.h
https://github.com/FluxGarage/RoboEyes/blob/main/examples/i2c_SSD1306_AnimationSequences/i2c_SSD1306_AnimationSequences.ino
*/


#include <Arduino.h>

#include "config.h"
// #include "motors_test.h"
// #include "testing.h"
#include "movement.h"
#include "PID.h"

TaskHandle_t Core0Task;
TaskHandle_t Core1Task;
void Core0Loop(void *pvParameters);
void Core1Loop(void *pvParameters);



/*
/// TESTING PID LOOP LIBRARY
int pidSource() {
  long val = motors::read_encoder(1);
  Serial.print("READ:\t");
  Serial.print(val);
  Serial.print("\t");
  return val;
}
void pidOutput(float output) {
  int int_val_full = static_cast<int>(output);  // -128-0, 0-128
  // remove dead space, start driving around 120
  int after_dead_space = int_val_full + ((int_val_full > 0) ? 120 : -120);

  motors::setMotorPwmRaw(1, after_dead_space);
  Serial.print("OUTPUT:\t");
  Serial.println(int_val_full);
}
//
const float P = 0.5f;
const float I = 0.01f;
const float D = 0.01f;
PIDController<float> myPIDController(P, I, D, pidSource, pidOutput);
*/


// --- Constants & Variables ---
float target_roll = 0.0;  // change when "skating"?
uint8_t current_leg_extension = 80;
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




// --- Constants & Variables ---
float target_pitch = 9.5; // Offset found through testing
float pitch_P = 4.0, pitch_I = 0.04, pitch_D = 0.10; // Higher P for motors usually
// float pitch_P = 0.5, pitch_I = 0.01, pitch_D = 0.01; // Higher P for motors usually

// --- PID Functions ---
// float getPitch() {
//     Serial.print("Pitch: ");
//     Serial.print(imu::angle_pitch);
//     return imu::angle_pitch; // Assuming pitch exists similarly to roll
// }

// void applyPitchOutput(float output) {
//     // Output range -135 to 135
//     int motor_speed = static_cast<int>(output);


//     int after_dead_space = motor_speed + ((motor_speed > 0) ? 120 : -120);

//     // motors::setMotorPwmRaw(1, after_dead_space);
//     Serial.print("\tOUTPUT:");
//     Serial.println(motor_speed);
    
//     // Drive both wheels together to balance
//     motors::setMotorPwmRaw(1, after_dead_space); // Left Motor
//     motors::setMotorPwmRaw(2, after_dead_space); // Right Motor
// }

// // Initialize Pitch PID
// PIDController<float> pitchPID(pitch_P, pitch_I, pitch_D, getPitch, applyPitchOutput);









// --- Global Variables for Loop Communication ---
float current_target_pitch = 0.0; 
float yaw_adjustment = 0.0;
float target_yaw = 0.0; // Set this in setup() to the current heading

// --- PID 1: Velocity Loop (The "Manager") ---
float getAverageRPM() {
    // Average speed of both wheels
    return (motors::read_rpm(1) + motors::read_rpm(2)) / 2.0f;
}

void applyVelocityOutput(float output) {
    // This output adjusts the target angle of the Pitch PID.
    // If the bot drifts forward (positive RPM), we increase the target pitch 
    // to make it lean back. 
    current_target_pitch = output; 

    Serial.print("\tTarget Pitch: ");
    Serial.print(current_target_pitch);
}

// --- PID 2: Yaw Loop (Steering) ---
float getYaw() {
    return imu::angle_yaw; 
}

void applyYawOutput(float output) {
    // This value will be added to one motor and subtracted from the other
    yaw_adjustment = output;
}

// --- Updated PID 3: Pitch Loop (The "Worker") ---
float getPitch() {
    Serial.print("\tPitch: ");
    Serial.print(imu::angle_pitch);

    return imu::angle_pitch;
}

void applyFinalMotorOutput(float output) {
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
    Serial.print("\tSpeed: ");
    Serial.print(spd1);
    
    // Drive both wheels together to balance
    motors::setMotorPwmRaw(1, spd1); // Left Motor
    motors::setMotorPwmRaw(2, spd2); // Right Motor
}

// --- Loop Initializations ---
// Velocity: P should be small, I is very important here to stop drift
PIDController<float> velocityPID(0.05, 0.001, 0.000, getAverageRPM, applyVelocityOutput);

// Yaw: Keeps the bot facing one way
PIDController<float> yawPID(1.5, 0.05, 0.2, getYaw, applyYawOutput);

// Pitch: (Your existing tuned constants)
PIDController<float> pitchPID(pitch_P, pitch_I, pitch_D, getPitch, applyFinalMotorOutput);









void setup() {
  Serial.begin(115200);
  
  
  
  // motors_test::begin();
  // movement::init();
  //================= TESTING =================
  // testing::runAllTests();

  // testing::runMotorSweep(1, 1000, 5, 50);
  // testing::runMotorSweep(1, 1000, 5, 50);

  // for (int i = 0; i < 200; i += 20) {
  //   motors_test::startClosedLoop(0, i, 5000);
  // }

  // testing::servoLegTest();
  // testing::servoLegTest();
  // testing::servoLegTest();
  // testing::servoLegTest();

  // testing::imuTest();
  // testing::oledBuzzerTest();
  
  // motors_test::startClosedLoop(1, 50, 5000);
  // motors_test::startClosedLoop(1, 100, 5000);
  // motors_test::startClosedLoop(1, 300, 5000);

  // motors_test::startClosedLoop(1, 50, 5000);
  // motors_test::startClosedLoop(1, 100, 5000);
  // motors_test::startClosedLoop(1, 300, 5000);

  // motors_test::startClosedLoop(1, 50, 5000);
  // motors_test::startClosedLoop(1, 100, 5000);
  // motors_test::startClosedLoop(1, 300, 5000);


  // testing::balance(1);

  
  // testing PID class from online

  // motors::init();
  movement::init();

  /*
  myPIDController.registerTimeFunction(millis);
  myPIDController.setOutputBounds(-135, 135); // currently driving raw PWM, but can increase resolution. +-2047?

  
  unsigned long start = millis();
  myPIDController.setTarget(-50);
  Serial.println("Driving to 50 RPM");
  while (start + 5000 > millis()) {
    myPIDController.tick();
    delay(50);
  }

  start = millis();
  myPIDController.setTarget(-100);
  Serial.println("Driving to 100 RPM");
  while (start + 5000 > millis()) {
    myPIDController.tick();
    delay(50);
  }

  start = millis();
  myPIDController.setTarget(-200);
  Serial.println("Driving to 200 RPM");
  while (start + 5000 > millis()) {
    myPIDController.tick();
    delay(50);
  }

  // motors::stopMotor(1);
  
  // sweep!!!
  // for (int i = -50; i < 50; i++) {
  //   myPIDController.setTarget(i);

  // }


  motors::stopMotor(1);
  */

  rollPID.registerTimeFunction(millis);
  rollPID.setOutputBounds(-20.0f, 20.0f); // Keep offset within your < 20 limit
  rollPID.setTarget(target_roll);

  pitchPID.registerTimeFunction(millis);
  pitchPID.setOutputBounds(-135, 135);
  pitchPID.setTarget(target_pitch);



  velocityPID.setTarget(0); 
  // velocityPID.setOutputBounds(-2.0, 2.0); // Limit lean to +/- 5 degrees
  velocityPID.setOutputBounds(7.0, 13.0); // 10 deg is close?
  velocityPID.registerTimeFunction(millis);

  // Yaw Loop: Stay at current heading
  target_yaw = imu::angle_yaw;
  yawPID.setTarget(target_yaw);
  yawPID.setOutputBounds(-1, 1); // Max power to use for turning
  yawPID.registerTimeFunction(millis);

  // Pitch Loop
  pitchPID.setOutputBounds(-135, 135);
  pitchPID.registerTimeFunction(millis);


  unsigned long lastUpdateTime = 0;
  const int updateInterval = 10; // 10ms = 100Hz


  while (true) {
    if (millis() - lastUpdateTime >= updateInterval) {
        lastUpdateTime = millis();

        imu::tick();

        /*
        // 1. Update Roll (Legs)
        rollPID.tick();

        // 2. Update Pitch (Wheels)
        pitchPID.tick();
        */
       // 1. Calculate how much we need to lean to stop drifting
        velocityPID.tick();

        // 2. Calculate how much we need to turn to stay straight
        // yawPID.tick();

        // 3. Update the Pitch target based on the Velocity PID output
        pitchPID.setTarget(current_target_pitch);

        // 4. Run the balance and yaw correction
        pitchPID.tick();
        
        // 5. Run the Roll/Leg balance from Step 1
        rollPID.tick();


        // if (millis() > 1000 * 20) break;
        Serial.println("");


        if (imu::angle_pitch < -25.0 || imu::angle_pitch > 35.0) {
          motors::stopMotor(1);
          motors::stopMotor(2);
          break;
        }
    }

  }


  //================= TESTING CLASSES =================
  // motors_test::runSweep(1, 1000, 1, 50);
  // motors_test::runSweep(2, 1000, 1, 50);

  // motors_test::begin();
  // motors_test::setMotorRaw(2, 100);
  //===================================================
  
  
  /*
  xTaskCreatePinnedToCore(
    Core0Loop,
    "Core0",
    10000,  // stack size?
    NULL, // task parameters
    1,  // task priority
    &Core0Task,
    0 // set to core 0
  );
  xTaskCreatePinnedToCore(
    Core1Loop,
    "Core1",
    10000,  // stack size?
    NULL, // task parameters
    1,  // task priority
    &Core1Task,
    1 // set to core 0
  );
  */
}

void loop() 
{
   // blank loop cause tasks do the loops
}

/*

// CORE 0 - Movement
void Core0Loop(void *pvParameters) {
  // setup...
  movement::init();


  for(;;) {
    // movement::loop();

    // imu update (at freq)

    // roll update (at freq)

    // pitch update (at freq)
    


    // handle communication between cores...
    // receive controls
    // receive any cosmetics
  }
}

// CORE 1 - Cosmetics
void Core1Loop(void *pvParameters) {
  // setup...
  controls::init();
  cosmetics::init();

  for(;;) {
    controls::loop(); // does the wifi controls?
    cosmetics::loop();

    // handle communication between cores...
    // transmit controls
    // receive any cosmetics
  }
}
*/