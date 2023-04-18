#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>

class Robot {
  public:
  /**
   * Construct a new Robot.
   *
   * \param wheel_radius Radius of wheels in cm
   * \param imu_wheel_distance Distance between IMU and center of wheels in cm
   * \param max_motor_speed Maximum speed of motors in rpm
   * \param motor_deadband Deadband of motors in range [0, 255]
   */
  Robot(float wheel_radius, float imu_wheel_distance, float max_motor_speed,
      int motor_deadband_plus, int motor_deadband_minus);

  /**
   * Reset robot state.
   */
  void reset();

  /**
   * Update the robot state.
   *
   * \param angle New angle of robot in radians
   * \param speed New speed of robot in cm/s
   * \param time Current time in ms
   */
  void updateState(float angle, float yaw, int speed, long time);

  /**
   * Convert robot state to string.
   */
  String toString();

  /**
   * Get angle.
   *
   * \return Angle of robot in radians
   */
  float getAngle();

  /**
   * Get velocity.
   *
   * \return Velocity of robot in cm/s
   */
  float getVelocity();

  /**
   * Get displacement.
   *
   * \return Displacement of robot in cm
   */
  float getDisplacement();

  /**
   * Get speed.
   *
   * \return Speed of motors in range [-255, 255]
   */
  int getSpeed();

  private:
  float wheel_radius; // Radius of wheels in cm
  float imu_wheel_distance; // Distance between IMU and center of wheels in cm
  float max_motor_speed; // Maximum speed of motors in rad/ms
  float angle; // Angle of robot in radians
  float yaw; // Yaw of robot in radians
  float velocity; // Velocity of robot in cm/ms
  float displacement; // Displacement of robot in cm
  int speed; // Speed of motors in range [-255, 255]
  long time; // Time of last update in ms
  bool initialized; // Whether robot state has been initialized
  int motor_deadband_plus; // Deadband of motors in range [0, 255]
  int motor_deadband_minus; // Deadband of motors in range [0, -255]
};

#endif // ROBOT_H