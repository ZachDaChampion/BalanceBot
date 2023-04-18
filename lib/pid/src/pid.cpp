#include "pid.h"

PID::PID() { reset(); }

float PID::update(float error)
{
  // Calculate time since last update
  unsigned long time = millis();
  unsigned long time_diff = last_time == 0 ? 0 : time - last_time;
  last_time = time;

  // Make sure we don't have any NaNs
  if (isnan(error)) {
    return last_result;
  }

  // Reset integral if error crosses zero (if enabled)
  if (reset_i_on_zero
      && (abs(error) <= integral_zero_threshold || (error > 0 && last_error < 0)
          || (error < 0 && last_error > 0))) {
    integral = 0;
  }

  // Update integral
  if (abs(error) > integral_zero_threshold
      && (!clamp_integral || (last_result_raw > min && last_result_raw < max))) {
    integral += error * time_diff;
  }

  // Update derivative
  if (time_diff > 0) {
    derivative = derivative * derivative_smoothing
        + ((error - last_error) / time_diff) * (1 - derivative_smoothing);
  }

  // Calculate result
  last_result_raw = kp * error + ki * integral + kd * derivative + kf;

  // Make sure result is within bounds
  if (last_result_raw < min)
    last_result = min;
  else if (last_result_raw > max)
    last_result = max;
  else
    last_result = last_result_raw;

  last_error = error;
  return last_result;
}

float PID::get_result(bool raw) { return raw ? last_result_raw : last_result; }

void PID::reset()
{
  integral = 0;
  derivative = 0;
  last_error = 0;
  last_result = 0;
  last_result_raw = 0;
  last_time = 0;
}