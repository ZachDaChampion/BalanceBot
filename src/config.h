#ifndef CONFIG_H
#define CONFIG_H

// Decide whether to start the robot on boot (otherwise start on serial command)
#define START_ON_BOOT false

// Select the controller to use
#define USE_PID_CONTROLLER

// Enable/disable the IMU interrupt on I2C
#define IMU_I2C_INTERRUPTS false

// Enable/disable continuous IMU calibration
#define IMU_CONTINUOUS_CALIBRATION true

// Config robot data printing
#define PRINT_ROBOT_DATA false
#define PRINT_ROBOT_DATA_INTERVAL 500
#define PRINT_SERIAL_PLOTTER false
#define PRINT_SERIAL_PLOTTER_NOT_RUNNING false

#endif // CONFIG_H