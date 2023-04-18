#include "motor-controller.h"

#include <Arduino.h>

// Values adapted from https://www.vexforum.com/t/aidans-truespeed/45160
static const uint8_t TRUESPEED_TABLE[] = { 0, 3, 6, 8, 10, 12, 14, 15, 16, 17, 18, 19,
  20, 20, 20, 21, 22, 22, 22, 22, 22, 23, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24,
  25, 26, 26, 26, 26, 26, 26, 26, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29, 30, 30,
  30, 30, 30, 30, 30, 31, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 33, 34, 34, 34,
  34, 34, 34, 34, 35, 36, 36, 36, 36, 36, 36, 36, 36, 36, 37, 38, 38, 38, 38, 38, 39,
  40, 40, 40, 40, 40, 40, 40, 41, 42, 42, 42, 42, 42, 43, 44, 44, 44, 44, 44, 44, 44,
  45, 46, 46, 46, 46, 46, 47, 48, 48, 48, 48, 48, 48, 48, 49, 50, 50, 50, 50, 50, 51,
  52, 52, 52, 52, 52, 53, 54, 54, 54, 54, 54, 55, 56, 56, 56, 56, 56, 57, 58, 58, 58,
  59, 60, 60, 60, 61, 62, 62, 62, 63, 64, 64, 64, 65, 66, 66, 66, 67, 68, 69, 70, 70,
  70, 71, 72, 73, 74, 74, 74, 75, 76, 77, 78, 78, 78, 79, 80, 81, 82, 82, 82, 83, 84,
  85, 86, 87, 88, 89, 90, 91, 92, 92, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102,
  104, 106, 107, 108, 109, 110, 112, 114, 117, 120, 122, 124, 128, 131, 135, 139, 142,
  145, 149, 153, 157, 161, 163, 165, 168, 171, 173, 175, 179, 183, 193, 203, 229, 255,
  255 };

MotorController::MotorController(
    int pin, bool reversed, int deadband_min, int deadband_max)
{
  this->pin = pin;
  this->reversed = reversed;
  this->deadband_min = deadband_min;
  this->deadband_max = deadband_max;
  this->speed = 0;
}

MotorController::~MotorController() { this->setSpeed(0); }

void MotorController::init()
{
  pinMode(pin, OUTPUT);
  this->servo.attach(pin);
  this->setSpeed(0);
}

void MotorController::setSpeed(int speed, bool true_speed)
{
  // Ensure speed is within bounds
  speed = constrain(speed, -255, 255);

  // Store speed
  this->speed = speed;

  // Reverse speed if requested
  if (this->reversed) {
    speed = -speed;
  }

  // Apply truespeed if requested
  if (true_speed) {
    speed = TRUESPEED_TABLE[abs(speed)] * (speed < 0 ? -1 : 1);
    if (speed > 0) {
      speed += deadband_max;
    }
    if (speed < 0) {
      speed += deadband_min;
    }

    // Ensure speed is within bounds
    speed = constrain(speed, -255, 255);
  }

  // Calculate PWM signal
  // 1ms - 2ms will give full reverse to full forward, 1.5ms is neutral
  int pwm_signal;
  if (speed == 0) {
    pwm_signal = 1500; // Neutral
  } else {
    pwm_signal = map(speed, -255, 255, 1000, 2000);
  }

  // Send PWM signal
  this->servo.writeMicroseconds(pwm_signal);
}

int MotorController::getSpeed() { return speed; }
