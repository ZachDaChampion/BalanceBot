#ifndef CONFIG_H
#define CONFIG_H

// Select the controller to use
#define USE_PID_CONTROLLER

// Enable/disable the IMU interrupt on I2C
#define IMU_I2C_INTERRUPTS false

// Config robot data printing
#define PRINT_ROBOT_DATA false
#define PRINT_ROBOT_DATA_INTERVAL 500
#define PRINT_SERIAL_PLOTTER true
#define PRINT_SERIAL_PLOTTER_NOT_RUNNING false

#endif // CONFIG_H