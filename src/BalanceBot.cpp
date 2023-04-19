#include <Arduino.h>

#include <SparkFun_BNO080_Arduino_Library.h>
#include <Wire.h>

#include <motor-controller.h>
#include <pid.h>
#include <robot.h>

// IMU pins
#define PIN_IMU_RESET 2
#define PIN_IMU_INT 3

// IMU params
#define IMU_I2C_ADDRESS 0x4A
#define IMU_I2C_SPEED 400000
#define IMU_UPDATE_RATE 10
#define ANGLE_MAX_SPEED (30.0 * PI / 180.0)

// Motor pins
#define PIN_MOTOR_LEFT 9
#define PIN_MOTOR_RIGHT 10

// Other pins
#define PIN_LED 13

// Interrupt data
bool new_accel_data = false;
bool new_lin_accel_data = false;
bool new_quat_data = false;

BNO080 imu;
Robot robot(3.14, 6.5, 100, 22, -26);
MotorController motor_left(PIN_MOTOR_LEFT, false, -24, 20);
MotorController motor_right(PIN_MOTOR_RIGHT, false, -24, 20);
float angle_offset = 0;
PID pid_roll;
PID pid_yaw;

long start_time;
bool started = false;
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

  imu.calibrateAll();

  // Enable IMU interrupt
  // attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), intHandler, FALLING);
  // interrupts();

  // Setup IMU communication
  Wire.setClock(IMU_I2C_SPEED); // Set I2C data rate
  imu.enableRotationVector(IMU_UPDATE_RATE);
  // imu.enableAccelerometer(IMU_UPDATE_RATE);
  // imu.enableLinearAccelerometer(IMU_UPDATE_RATE);

  Serial.println(F("IMU enabled"));

  /*
   * Set up motors
   */

  motor_left.init();
  motor_right.init();
  Serial.println(F("Motors enabled"));

  /*
   * Set up PID controllers
   */

  // Primary balancing PID
  pid_roll.kp = 0;
  pid_roll.ki = 0;
  pid_roll.kd = 0;
  pid_roll.min = -255;
  pid_roll.max = 255;
  pid_roll.integral_zero_threshold = 0.008;
  pid_roll.reset_i_on_zero = true;
  pid_roll.clamp_integral = true;
  pid_roll.derivative_smoothing = 0.15;

  // Yaw correction PID
  pid_yaw.kp = 300 / PI;
  pid_yaw.min = -150;
  pid_yaw.max = 150;

  /*
   * Start
   */

  pinMode(PIN_LED, OUTPUT);
  Serial.println(F("Ready"));
  start_time = millis();
}

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

long last_print = 0;
float yaw_offset = 0;
void loop()
{
  /*
   * Handle new rotation data
   */

  if (new_quat_data || imu.dataAvailable()) {
    new_quat_data = false;

    // Get IMU data
    float roll = imu.getRoll();
    float yaw = imu.getYaw() - yaw_offset;
    float roll_adjusted
        = roll - HALF_PI - angle_offset; // Adjust roll to be 0 when upright

    // Update robot state
    robot.updateState(roll_adjusted, yaw,
        (motor_left.getSpeed() + motor_right.getSpeed()) / 2, millis());

    // Calculate motor speed relative to roll
    float error
        = constrain(roll_adjusted / ANGLE_MAX_SPEED, -ANGLE_MAX_SPEED, ANGLE_MAX_SPEED);
    error = error * error * error;
    float motor_speed = pid_roll.update(error);

    // Turn on LED when motor is saturated
    if (motor_speed == 255 || motor_speed == -255) {
      digitalWrite(PIN_LED, HIGH);
    } else {
      digitalWrite(PIN_LED, LOW);
    }

    // Adjust motor speed to maintain 0 yaw
    float yaw_adjust = pid_yaw.update(yaw);

    // Set motor speed
    if (started) {
      motor_left.setSpeed(round(motor_speed + yaw_adjust));
      motor_right.setSpeed(round(motor_speed - yaw_adjust));

      // Print data
      Serial.print("a:");
      Serial.print(roll_adjusted);
      Serial.print(",e:");
      Serial.println(error);
    }
    // Serial.print("a:");
    // Serial.print(roll_adjusted * 1000);
    // Serial.print(",e:");
    // Serial.println(error * 1000);
  }

  /*
   * Print data regularly
   */

#if 0
  if (millis() - last_print > 500) {
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
    motor_left.setSpeed(0);
    motor_right.setSpeed(0);
    robot.reset();
    pid_roll.reset();
    pid_yaw.reset();
    yaw_offset = imu.getYaw();
    Serial.println(F("Motors started"));
    started = true;
  }

  // Stop motors on stop signal (Ctrl-C or 'e')
  else if (serial_input == 0x03 || serial_input == 'e') {
    motor_left.setSpeed(0);
    motor_right.setSpeed(0);
    Serial.println(F("Motors stopped"));
    started = false;
  }

  // Update PID constants
  else if (serial_input == 'p') {
    char buffer[32];
    size_t len = Serial.readBytesUntil('\n', buffer, 32);
    buffer[len] = '\0';
    pid_roll.kp = atof(buffer);
    pid_roll.reset();
  } else if (serial_input == 'i') {
    char buffer[32];
    size_t len = Serial.readBytesUntil('\n', buffer, 32);
    buffer[len] = '\0';
    pid_roll.ki = atof(buffer);
    pid_roll.reset();
  } else if (serial_input == 'd') {
    char buffer[32];
    size_t len = Serial.readBytesUntil('\n', buffer, 32);
    buffer[len] = '\0';
    pid_roll.kd = atof(buffer);
    pid_roll.reset();
  }

  // Update angle offset
  else if (serial_input == 'o') {
    char buffer[32];
    size_t len = Serial.readBytesUntil('\n', buffer, 32);
    buffer[len] = '\0';
    angle_offset = atof(buffer);
    pid_roll.reset();
  }

  // Print PID constants
  else if (serial_input == 'P') {
    Serial.print(F("kp "));
    Serial.println(pid_roll.kp);
    Serial.print(F("ki "));
    Serial.println(pid_roll.ki);
    Serial.print(F("kd "));
    Serial.println(pid_roll.kd);
  }
}