#include "controller-pid.h"

#include <Arduino.h>

ControllerPID::ControllerPID() { reset(); }

Controller::MotorValues ControllerPID::update(float robot_angle, float robot_yaw)
{
  /*
   * Update time.
   */

  unsigned long current_time = millis();

  float dt // Time since last update (seconds) or 0 if this is the first update
      = last_time // Check if this is the first update
      ? (current_time - last_time) / 1000.0 // Convert from milliseconds to seconds
      : 0; // 0 if this is the first update

  last_time = current_time;

  /*
   * Calculate error.
   */

  float error_angle = robot_angle - setpoint_angle; // Error in robot angle
  float error_yaw = robot_yaw - setpoint_yaw; // Error in robot yaw

  /*
   * Calculate feedforward.
   */

  // Calculate angular velocity
  float angular_velocity;
  if (angle_reset || dt == 0 || !ff_add_sensor) {
    angular_velocity = 0;
  } else {

    // Calculate raw angular velocity
    angular_velocity = (robot_angle - last_angle) / dt;

    // Subtract motor speed. This will approximate the influence of external forces
    // alone, without the influence of the motor. This is important because the
    // feedforward term should only be used to compensate for external forces.
    angular_velocity += last_value_angle * (motor_speed / 255) * (TWO_PI / 60);

    // Apply smoothing
    angular_velocity = (1 - angular_vel_smoothing) * angular_velocity
        + angular_vel_smoothing * last_angular_velocity;
  }

  // Calculate angular acceleration of robot due to gravity (if enabled)
  if (ff_add_gravity && torque_length > 0) {

    /*
     * F = m * g
     *   F: force of gravity
     *   g: acceleration due to gravity
     *   m: mass of robot
     *
     * I = m * L^2
     *   I: moment of inertia of robot
     *   L: length of torque arm
     *
     * T = F * L * sin(theta)
     *   T: torque of gravity
     *   theta: angle of robot
     *
     * alpha = T / I
     *   alpha: angular acceleration of robot due to gravity
     *
     * therefore:
     *
     * alpha = m * g * L * sin(theta) / (m * L^2)
     *       = g * sin(theta) / L
     */

    float a = 981 * sin(robot_angle) / torque_length; // Angular acceleration of robot
    angular_velocity += a * dt; // Update angular velocity with angular acceleration
  }

  /*
   * Update angle integral.
   */

  // Skip integrating if reset
  if (angle_reset) {
    goto skip_integrate_angle;
  }

  // Reset angle integral if it crosses zero (if enabled)
  if (params_angle.reset_i_on_zero && (error_angle > 0) != (last_error_angle > 0)) {
    integral_angle = 0;
  }

  // Skip integrating angle if it is close to the setpoint (if enabled)
  if (params_angle.reset_i_on_zero
      && abs(error_angle) < params_angle.i_zero_threshold) {
    goto skip_integrate_angle;
  }

  // Skip integrating angle if saturated (if enabled)
  if (params_angle.clamp_i
      && (last_value_angle < params_angle.min_val
          || last_value_angle > params_angle.max_val)) {
    goto skip_integrate_angle;
  }

  // Integrate angle
  integral_angle += error_angle * dt;
skip_integrate_angle:

  // Skip integrating if reset
  if (yaw_reset) {
    goto skip_integrate_yaw;
  }

  /*
   * Update yaw integral.
   */

  // Reset yaw integral if it crosses zero (if enabled)
  if (params_yaw.reset_i_on_zero && (error_yaw > 0) != (last_error_yaw > 0)) {
    integral_yaw = 0;
  }

  // Skip integrating yaw if it is close to the setpoint (if enabled)
  if (params_yaw.reset_i_on_zero && abs(error_yaw) < params_yaw.i_zero_threshold) {
    goto skip_integrate_yaw;
  }

  // Skip integrating yaw if saturated (if enabled)
  if (params_yaw.clamp_i
      && (last_value_yaw < params_yaw.min_val || last_value_yaw > params_yaw.max_val)) {
    goto skip_integrate_yaw;
  }

  // Integrate yaw
  integral_yaw += error_yaw * dt;
skip_integrate_yaw:

  /*
   * Update derivative.
   */

  // Calculate derivative of angle
  if (dt > 0 && !angle_reset) {
    float raw_derivative = (error_angle - last_error_angle) / dt; // Raw derivative
    derivative_angle // Apply smoothing
        = (1 - params_angle.d_smoothing) * raw_derivative
        + params_angle.d_smoothing * derivative_angle;
  }

  // Calculate derivative of yaw
  if (dt > 0 && !yaw_reset) {
    float raw_derivative = (error_yaw - last_error_yaw) / dt; // Raw derivative
    derivative_yaw // Apply smoothing
        = (1 - params_yaw.d_smoothing) * raw_derivative
        + params_yaw.d_smoothing * derivative_yaw;
  }

  /*
   * Calculate PID outputs.
   */

  // Calculate PID output for angle
  float value_angle = params_angle.kp * error_angle // Proportional term
      + params_angle.ki * integral_angle // Integral term
      + params_angle.kd * derivative_angle // Derivative term
      + params_angle.kf * angular_velocity; // Feedforward term

  // Constrain PID output for angle
  value_angle = constrain(value_angle, params_angle.min_val, params_angle.max_val);

  // Calculate PID output for yaw
  float value_yaw = params_yaw.kp * error_yaw // Proportional term
      + params_yaw.ki * integral_yaw // Integral term
      + params_yaw.kd * derivative_yaw; // Derivative term

  // Constrain PID output for yaw
  value_yaw = constrain(value_yaw, params_yaw.min_val, params_yaw.max_val);

  // Update saturation flags
  angle_saturated
      = value_angle == params_angle.min_val || value_angle == params_angle.max_val;
  yaw_saturated = value_yaw == params_yaw.min_val || value_yaw == params_yaw.max_val;

  /*
   * Mix PID outputs.
   */

  float value_left = value_angle + value_yaw; // Left motor value
  float value_right = value_angle - value_yaw; // Right motor value

  /*
   * Constrain PID outputs to motor limits.
   */
  int motor_left = constrain(round(value_left), -255, 255); // Left motor value
  int motor_right = constrain(round(value_right), -255, 255); // Right motor value

  /*
   * Update last values.
   */

  // Update last values for angle
  last_value_angle = value_angle;
  last_angle = robot_angle;
  last_error_angle = error_angle;
  last_angular_velocity = angular_velocity;
  angle_reset = false;

  // Update last values for yaw
  last_value_yaw = value_yaw;
  last_yaw = robot_yaw;
  last_error_yaw = error_yaw;
  yaw_reset = false;

  return { motor_left, motor_right };
}

void ControllerPID::setSetpointAngle(float setpoint_angle)
{
  this->setpoint_angle = setpoint_angle;
  reset_angle_pid();
}

void ControllerPID::setSetpointYaw(float setpoint_yaw)
{
  this->setpoint_yaw = setpoint_yaw;
  reset_yaw_pid();
}

bool ControllerPID::isAngleSaturated() { return angle_saturated; }

bool ControllerPID::isYawSaturated() { return yaw_saturated; }

void ControllerPID::reset_angle_pid()
{
  last_value_angle = 0;
  last_angle = 0;
  last_error_angle = 0;
  integral_angle = 0;
  derivative_angle = 0;
  last_angular_velocity = 0;
  angle_reset = true;
}

void ControllerPID::reset_yaw_pid()
{
  last_value_yaw = 0;
  last_yaw = 0;
  last_error_yaw = 0;
  integral_yaw = 0;
  derivative_yaw = 0;
  yaw_reset = true;
}

void ControllerPID::reset()
{
  reset_angle_pid();
  reset_yaw_pid();
  last_time = 0;
}
