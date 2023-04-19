#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include "controller.h"

#include <stdint.h>

class ControllerPID : public Controller {
  public:

  /**
   * Struct to hold PID parameters.
   */
  struct Params {
    float kp = 0; // Proportional gain
    float ki = 0; // Integral gain
    float kd = 0; // Derivative gain
    float kf = 0; // Feedforward gain
    float i_zero_threshold = 0; // Threshold for integral to be considered zero
    float d_smoothing = 0; // Smoothing factor for derivative
    bool clamp_i = false; // Whether to clamp the integral
    bool reset_i_on_zero = false; // Whether to reset the integral when error is zero
    float min_val = -255; // Minimum output
    float max_val = 255; // Maximum output
  };

  /**
   * Construct a new ControllerPID object.
   */
  ControllerPID();

  /**
   * Update the controller.
   *
   * \param robot_angle The angle of the robot.
   * \param robot_yaw The yaw of the robot.
   * \return The correction to apply to the motors.
   */
  MotorValues update(float robot_angle, float robot_yaw) override;

  /**
   * Set the setpoint angle.
   *
   * \param setpoint_angle The setpoint angle.
   */
  void setSetpointAngle(float setpoint_angle) override;

  /**
   * Set the setpoint yaw.
   *
   * \param setpoint_yaw The setpoint yaw.
   */
  void setSetpointYaw(float setpoint_yaw) override;

  /**
   * Check if the angle controller is saturated.
   *
   * \return Whether the angle controller is saturated.
   */
  bool isAngleSaturated() override;

  /**
   * Check if the yaw controller is saturated.
   *
   * \return Whether the yaw controller is saturated.
   */
  bool isYawSaturated() override;

  /**
   * Reset the controller.
   */
  void reset() override;

  /*
   * Tuning parameters.
   */
  Params params_angle; // PID parameters for angle
  Params params_yaw; // PID parameters for yaw
  float torque_length = 0; // Length of torque arm
  bool ff_add_gravity = false; // Whether to add gravity to the feedforward term
  bool ff_add_sensor = true; // Whether to add sensor data to the feedforward term
  float angular_vel_smoothing = 0; // Smoothing factor for angular velocity
  float motor_speed = 100; // Motor speed (RPM)

  private:

  /**
   * Reset the angle PID.
   */
  void reset_angle_pid();

  /**
   * Reset the yaw PID.
   */
  void reset_yaw_pid();

  float setpoint_angle; // Setpoint angle
  float setpoint_yaw; // Setpoint yaw

  float last_value_angle; // Last value for angle
  float last_value_yaw; // Last value for yaw

  float last_angle; // Last angle
  float last_yaw; // Last yaw

  float last_error_angle; // Last error for angle
  float last_error_yaw; // Last error for yaw

  float integral_angle; // Integral of error for angle
  float integral_yaw; // Integral of error for yaw

  float derivative_angle; // Derivative of error for angle
  float derivative_yaw; // Derivative of error for yaw

  unsigned long last_time; // Last time update was called

  bool angle_saturated; // Whether the angle PID is saturated
  bool yaw_saturated; // Whether the yaw PID is saturated

  bool angle_reset; // Whether the angle PID has been reset
  bool yaw_reset; // Whether the yaw PID has been reset

  float last_angular_velocity; // Last angular velocity of the robot
};

#endif