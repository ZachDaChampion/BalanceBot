#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  public:

  struct Params { };

  /**
   * Construct a new PID.
   */
  PID();

  /**
   * Update the PID controller.
   *
   * \param error Error to correct
   * \return Correction to apply
   */
  float update(float error);

  /**
   * Get most recent result.
   *
   * \param raw Whether to return the raw result or the clamped result
   * \return Most recent result
   */
  float get_result(bool raw = false);

  /**
   * Reset the PID controller.
   */
  void reset();

  /*
   * Params.
   */
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float kf = 0;
  float derivative_smoothing = 0;
  float min = -INFINITY;
  float max = INFINITY;
  float integral_zero_threshold = 0;
  bool clamp_integral = false;
  bool reset_i_on_zero = false;

  private:
  Params params; // PID parameters
  float integral; // Integral of error
  float derivative; // Derivative of error
  float last_error; // Last error
  float last_result; // Last result
  float last_result_raw; // Last result
  unsigned long last_time; // Last time update was called
};

#endif