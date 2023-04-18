#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Servo.h>

class MotorController {
  public:
  /**
   * \brief Construct a new Motor Controller object.
   *
   * \param pin The pin that the motor controller PWM is connected to.
   * \param reversed If true, the motor controller will be reversed.
   */
  MotorController(
      int pin, bool reversed = false, int deadband_min = 0, int deadband_max = 0);

  /**
   * \brief Destroy the Motor Controller object.
   *
   */
  ~MotorController();

  /**
   * \brief Initialize the motor controller.
   */
  void init();

  /**
   * \brief Set the speed of the motor controller.
   *
   * \param speed The speed of the motor controller in the range [-255, 255].
   * \param true_speed If true, the value sent to the motor controller will be
   *                   modified to produce a more linear response.
   */
  void setSpeed(int speed, bool true_speed = true);

  /**
   * \brief Get the speed of the motor controller.
   *
   * \return int The speed of the motor controller.
   */
  int getSpeed();

  /**
   * \brief Set the reversed state of the motor controller.
   *
   * \param reversed If true, the motor controller will be reversed.
   */
  void setReversed(bool reversed);

  /**
   * \brief Get the reversed state of the motor controller.
   *
   * \return true If the motor controller is reversed.
   * \return false If the motor controller is not reversed.
   */
  bool getReversed();

  private:
  int pin;
  int speed;
  bool reversed;
  int deadband_min;
  int deadband_max;
  Servo servo;
};

#endif // MOTOR_CONTROLLER_H