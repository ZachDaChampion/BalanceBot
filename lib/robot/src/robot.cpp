#include "robot.h"

Robot::Robot(float wheel_radius, float imu_wheel_distance, float max_motor_speed,
    int motor_deadband_plus, int motor_deadband_minus)
{
  this->wheel_radius = wheel_radius;
  this->imu_wheel_distance = imu_wheel_distance;
  this->max_motor_speed = max_motor_speed * 2 * TWO_PI / 60000; // Convert to rad/ms
  this->motor_deadband_plus = motor_deadband_plus;
  this->motor_deadband_minus = motor_deadband_minus;
}

void Robot::reset()
{
  initialized = false;
  angle = 0;
  velocity = 0;
  displacement = 0;
  speed = 0;
  time = 0;
}

void Robot::updateState(float angle, float yaw, int speed, long time)
{
  long time_diff = time - this->time;

  if (!initialized) {
    this->angle = angle;
    this->yaw = yaw;
    this->speed = speed;
    this->velocity = 0;
    this->displacement = 0;
    this->time = time;
    initialized = true;
    return;
  }

  /*
   * Approximate displacement.
   */

  // Angle that wheel has turned in radians (assuming perfect motor response)
  float wheel_angle = 0;
  if (speed > motor_deadband_plus) {
    wheel_angle = (speed - motor_deadband_plus) / 255.0 * max_motor_speed * time_diff;
  } else if (speed < motor_deadband_minus) {
    wheel_angle = (speed - motor_deadband_minus) / 255.0 * max_motor_speed * time_diff;
  }

  // Difference between current and previous angle of robot in radians
  float robot_angle_diff = angle - this->angle;

  // Add wheel and robot angle differences to get total wheel movement in radians
  float actual_wheel_angle = robot_angle_diff + wheel_angle;

  // Distance traveled by wheels in cm
  float wheel_distance = actual_wheel_angle * wheel_radius;

  /*
   * Update state.
   */

  this->angle = angle;
  this->yaw = yaw;
  this->speed = speed;
  this->velocity = time_diff > 0 ? wheel_distance / time_diff : 0;
  this->displacement += wheel_distance;
  this->time = time;
}

String Robot::toString()
{
  String str = "Angle: ";
  str += angle * 180 / PI;
  str += " Yaw: ";
  str += yaw * 180 / PI;
  // str += " Velocity: ";
  // str += velocity;
  str += " Displacement: ";
  str += displacement;
  str += " Speed: ";
  str += speed;
  return str;
}

float Robot::getAngle() { return angle; }

float Robot::getVelocity() { return velocity; }

float Robot::getDisplacement() { return displacement; }

int Robot::getSpeed() { return speed; }
