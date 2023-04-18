#if 0

#include <Arduino.h>

#include <motor-controller.h>

// Motor pins
#define PIN_MOTOR_LEFT 9
#define PIN_MOTOR_RIGHT 10

MotorController motor_left(PIN_MOTOR_LEFT);
MotorController motor_right(PIN_MOTOR_RIGHT);

void setup()
{

  /*
   * Set up serial
   */

  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Motor Test starting up..."));

  /*
   * Set up motors
   */

  motor_left.init();
  motor_right.init();

  Serial.println(F("Motors enabled"));
}

int motor_speed = 0;
void loop()
{
  /*
   * Handle serial input.
   */

  // Read serial input to buffer
  int available = 0;
  do {
    available = Serial.available();
    delay(1);
  } while (Serial.available() > available);

  if (available > 0) {
    Serial.print(F("Received "));
    Serial.print(available);
    Serial.println(F(" bytes"));
    char input[available + 1];
    for (int i = 0; i < available; ++i) {
      input[i] = Serial.read();
    }
    input[available] = '\0';

    // Convert to int
    int input_speed = atoi(input);

    // Validate input
    if (input_speed > 255 || input_speed < -255) {
      Serial.println(F("Invalid speed"));
    } else {
      motor_speed = input_speed;
      Serial.print(F("Setting speed to "));
      Serial.println(motor_speed);
    }

    // Move motors
    motor_left.setSpeed(motor_speed);
    motor_right.setSpeed(motor_speed);
  }
}

#endif