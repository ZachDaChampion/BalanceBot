#include <errno.h>
#include <stdlib.h>

#include <Arduino.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <Wire.h>

#include <config.h>
#include <motor-controller.h>
#include <robot.h>

#ifdef USE_PID_CONTROLLER
#include <controller-pid.h>
#endif

// IMU pins
#define PIN_IMU_RESET 2
#define PIN_IMU_INT 3

// IMU params
#define IMU_I2C_ADDRESS 0x4A
#define IMU_I2C_SPEED 400000
#define IMU_UPDATE_RATE 10

// Motor pins
#define PIN_MOTOR_LEFT 9
#define PIN_MOTOR_RIGHT 10

// Other pins
#define PIN_LED 13

// Interrupt data
bool new_accel_data = false;
bool new_lin_accel_data = false;
bool new_quat_data = false;

// Global variables
BNO080 imu; // IMU
Robot robot(3.14, 6.5, 100, 22, -26); // Robot
MotorController motor_left(PIN_MOTOR_LEFT, false, -24, 20); // Left motor controller
MotorController motor_right(PIN_MOTOR_RIGHT, false, -24, 20); // Right motor controller
bool running = false; // Is the robot running?

// Set controller
#ifdef USE_PID_CONTROLLER
ControllerPID controller;
#endif

// Forward declarations
void calibrateImu();

void setup()
{

  /*
   * Set up serial
   */

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Balance Bot starting up..."));

  /*
   * Set up IMU
   */

  Wire.begin();

  // Init IMU
  if (imu.begin(IMU_I2C_ADDRESS, Wire) == false) {
    // if (imu.begin(IMU_I2C_ADDRESS, Wire, PIN_IMU_INT) == false) {
    Serial.println(
        F("BNO080 not detected at default I2C address. Check your jumpers and "
          "the hookup guide. Freezing..."));
    while (1) { }
  }

// Enable IMU interrupt
#if IMU_I2C_INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), intHandler, FALLING);
  interrupts();
#endif

  // Setup IMU communication
  Wire.setClock(IMU_I2C_SPEED); // Set I2C data rate
  imu.enableRotationVector(IMU_UPDATE_RATE);

// Calibrate IMU
#if IMU_CONTINUOUS_CALIBRATION
  imu.calibrateAll();
#endif

  Serial.println(F("IMU enabled"));

  /*
   * Set up motors
   */

  motor_left.init();
  motor_right.init();
  Serial.println(F("Motors enabled"));

  /*
   * Set up controller
   */

#ifdef USE_PID_CONTROLLER

  // Primary balancing PID
  controller.params_angle.kp = 50;
  controller.params_angle.ki = 500;
  controller.params_angle.kd = 5;
  controller.params_angle.kf = 10;
  controller.params_angle.min_val = -255;
  controller.params_angle.max_val = 255;
  controller.params_angle.i_zero_threshold = 0.008;
  controller.params_angle.reset_i_on_zero = true;
  controller.params_angle.clamp_i = true;
  controller.params_angle.d_smoothing = 0.05;
  controller.angular_vel_smoothing = 0.8;
  controller.torque_length = 6.35; // cm
  controller.ff_add_gravity = true;
  controller.ff_add_sensor = false;

  // Yaw correction PID
  controller.params_yaw.kp = 300 / PI;
  controller.params_yaw.min_val = -150;
  controller.params_yaw.max_val = 150;
#endif

  /*
   * Start
   */

#if START_ON_BOOT
  running = true;
#endif

  pinMode(PIN_LED, OUTPUT);
  Serial.println(F("Ready"));
}

#if IMU_I2C_INTERRUPTS

/**
 * Interrupt handler for IMU data.
 */
void intHandler()
{
  switch (imu.getReadings()) {
  case SENSOR_REPORTID_ACCELEROMETER:
    new_accel_data = true;
    break;
  case SENSOR_REPORTID_LINEAR_ACCELERATION:
    new_lin_accel_data = true;
    break;
  case SENSOR_REPORTID_ROTATION_VECTOR:
    new_quat_data = true;
    break;
  }
}
#endif

unsigned long last_print = 0;
void loop()
{
  /*
   * Handle new rotation data
   */

  if (new_quat_data || imu.dataAvailable()) {
    new_quat_data = false;

    // Get IMU data
    float roll = imu.getRoll();
    float yaw = imu.getYaw();
    if (isnan(roll) || isnan(yaw)) {
      return;
    }
    float roll_adjusted = roll - HALF_PI; // Adjust roll to be 0 when upright

#if PRINT_ROBOT_DATA

    // Update robot state
    robot.updateState(roll_adjusted, yaw,
        (motor_left.getSpeed() + motor_right.getSpeed()) / 2, millis());

#endif

    // Update controller
    Controller::MotorValues motor_vals = controller.update(roll_adjusted, yaw);

    // Turn on LED when main controller is saturated
    if (controller.isAngleSaturated()) {
      digitalWrite(PIN_LED, HIGH);
    } else {
      digitalWrite(PIN_LED, LOW);
    }

    // Set motor speed
    if (running) {
      motor_left.setSpeed(motor_vals.left);
      motor_right.setSpeed(motor_vals.right);
    }

    // Print data for serial plotter
#if PRINT_SERIAL_PLOTTER
#if PRINT_SERIAL_PLOTTER_NOT_RUNNING
    if (1) {
#else
    if (running) {
#endif
      Serial.print(" t:");
      Serial.print(roll_adjusted);
      Serial.print(" y:");
      Serial.println(yaw);
    }
#endif
  }

#if PRINT_ROBOT_DATA
  /*
   * Print data regularly.
   */

  if (millis() - last_print > PRINT_ROBOT_DATA_INTERVAL) {
    Serial.println(robot.toString());
    last_print = millis();
  }
#endif

  /*
   * Read for serial commands.
   */

  // Read serial input
  int serial_input = Serial.read();

  // Start motors on start signal ('s')
  if (serial_input == 's') {
    robot.reset();
    controller.reset();
    controller.setSetpointYaw(imu.getYaw());
    motor_left.setSpeed(0);
    motor_right.setSpeed(0);
    Serial.println(F("Motors started"));
    running = true;
  }

  // Stop motors on stop signal (Ctrl-C or 'e')
  else if (serial_input == 0x03 || serial_input == 'e') {
    motor_left.setSpeed(0);
    motor_right.setSpeed(0);
    Serial.println(F("Motors stopped"));
    running = false;
  }

  // Calibrate IMU ('c')
  else if (serial_input == 'c') {
    calibrateImu();
    controller.reset();
  }

  // Handle numeric input
  else if (serial_input != -1) {
    errno = 0;
    static char buffer[32];
    size_t len = Serial.readBytesUntil('\n', buffer, 32);
    buffer[len] = '\0';
    float new_val = strtod(buffer, NULL);

    if (errno == 0) {
      switch (serial_input) {

      // Set controller setpoints
      case 'a':
        controller.setSetpointAngle(new_val);
        break;
      case 'y':
        controller.setSetpointYaw(new_val);
        break;

#ifdef USE_PID_CONTROLLER

      // Set PID parameters
      case 'p':
        controller.params_angle.kp = new_val;
        break;
      case 'i':
        controller.params_angle.ki = new_val;
        break;
      case 'd':
        controller.params_angle.kd = new_val;
        break;
      case 'f':
        controller.params_angle.kf = new_val;
        break;

      // Print PID parameters
      case 'P': {
        Serial.println(F("ANGLE PID: "));
        Serial.print(F("  kp: "));
        Serial.println(controller.params_angle.kp);
        Serial.print(F("  ki: "));
        Serial.println(controller.params_angle.ki);
        Serial.print(F("  kd: "));
        Serial.println(controller.params_angle.kd);
        Serial.print(F("  kf: "));
        Serial.println(controller.params_angle.kf);
        Serial.println(F("YAW PID: "));
        Serial.print(F("  kp: "));
        Serial.println(controller.params_yaw.kp);
        Serial.print(F("  ki: "));
        Serial.println(controller.params_yaw.ki);
        Serial.print(F("  kd: "));
        Serial.println(controller.params_yaw.kd);
      } break;

#endif
      }
    } else {
      Serial.println(F("Invalid input"));
    }
  }
}

/**
 * Print the accuracy of the an IMU parameter.
 *
 * \param num The accuracy number.
 */
inline void printAccuracy(int num)
{
  switch (num) {
  case 0:
    Serial.print("Unreliable");
    break;
  case 1:
    Serial.print("Low");
    break;
  case 2:
    Serial.print("Medium");
    break;
  case 3:
    Serial.print("High");
    break;
  }
}

void calibrateImu()
{

  /*
   * Configure IMU for calibration.
   */

  imu.enableGameRotationVector(100);
  imu.enableMagnetometer(100);
  imu.calibrateAll();

  /*
   * Print instructions.
   */

  Serial.println();
  Serial.println(F("====== Calibration ======"));
  Serial.println();
  Serial.println(F("Please follow the instructions below to calibrate the IMU."));
  Serial.println(
      F("When you are done, enter the character 's' to save the calibration."));
  Serial.println();
  Serial.println(
      F("1. Calibrate the accelerometer by rotating the robot in all directions."));
  Serial.println(F("Hold the robot in each position for 1 second."));
  Serial.println();
  Serial.println(
      F("2. Calibrate the gyroscope by setting device down on a flat surface "
        "for 3 seconds."));
  Serial.println();
  Serial.println(
      F("3. Calibrate the magnetometer by rotating the robot 180 degrees and "
        "back in each axis."));
  Serial.println(F("Each rotation should take 2 seconds."));
  Serial.println();
  Serial.println();
  Serial.println();

  /*
   * Wait for calibration to be complete.
   */

  while (1) {

    // Check if user wants to save calibration
    char c = Serial.read();
    if (c == 's') {
      break;
    }

    // Print accuracy
    if (imu.dataAvailable()) {
      Serial.print(F("\033[F\033[FGyroscope accuracy: "));
      printAccuracy(imu.getGyroAccuracy());
      Serial.println();
      Serial.print(F("Magnetometer accuracy: "));
      printAccuracy(imu.getMagAccuracy());
      Serial.println();
    }
  }

  /*
   * Save calibration.
   */

  Serial.println(F("Saving calibration..."));

  imu.saveCalibration();
  imu.requestCalibrationStatus();

  size_t counter = 1000;
  while (1) {
    if (--counter == 0) {
      Serial.println(F("Saving calibration failed!"));
      break;
    }

    if (imu.dataAvailable()) {
      if (imu.calibrationComplete()) {
        Serial.println(F("Calibration complete!"));
        break;
      }
    }

    delay(1);
  }

#if !IMU_CONTINUOUS_CALIBRATION
  imu.endCalibration();
#endif
}