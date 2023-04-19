#ifndef CONTROLLER_H
#define CONTROLLER_H

/**
 * Abstract class for a controller.
 *
 * A controller is used to control the robot's motors. It can correct the robot's
 * angle and yaw, allowing it to balance and maintain a direction.
 */
class Controller {
  public:

  /**
   * Struct to hold left and right motor values.
   */
  struct MotorValues {
    int left; // Left motor value
    int right; // Right motor value
  };

  /**
   * Update the controller.
   *
   * \param robot_angle The angle of the robot.
   * \param robot_yaw The yaw of the robot.
   * \return The correction to apply to the motors.
   */
  virtual MotorValues update(float robot_angle, float robot_yaw);

  /**
   * Set the setpoint angle.
   *
   * \param setpoint_angle The setpoint angle.
   */
  virtual void setSetpointAngle(float setpoint_angle);

  /**
   * Set the setpoint yaw.
   *
   * \param setpoint_yaw The setpoint yaw.
   */
  virtual void setSetpointYaw(float setpoint_yaw);

  /**
   * Check if the angle controller is saturated.
   *
   * \return Whether the angle controller is saturated.
   */
  virtual bool isAngleSaturated();

  /**
   * Check if the yaw controller is saturated.
   *
   * \return Whether the yaw controller is saturated.
   */
  virtual bool isYawSaturated();

  /**
   * Reset the controller.
   */
  virtual void reset();
};

#endif // CONTROLLER_H