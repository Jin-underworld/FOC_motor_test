#define SELECTED_BOARD BOARD_ARDUINO_UNO
#include "boardConfig.h"

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

#define BAUD_RATE 9600
// Physical parameters for gravity compensation
#define ARM_LENGTH 0.1f    // 100mm = 0.1m
#define LOAD_MASS 0.01f    // 20g = 0.02kg
#define GRAVITY 9.81f      // m/s^2
#define VOLTAGE_TO_TORQUE_RATIO 0.0260f  // TUNED [Nm/V]

// Motor configuration
#define MOTOR_POLE_PAIRS 11
#define MOTOR_PHASE_RESISTANCE 8.8f
#define MOTOR_VOLTAGE_LIMIT 12.0f
#define MOTOR_PID_VELOCITY_P 0.3f
#define MOTOR_PID_VELOCITY_I 0.00f
#define MOTOR_PID_VELOCITY_D 0.05f
#define MOTOR_PID_VELOCITY_RAMP 1000.0f
#define MOTOR_P_ANGLE 2.0f
#define MOTOR_VELOCITY_LIMIT 20.0f
#define MOTOR_ZERO_ELECTRIC_ANGLE 2.21f
#define MOTOR_SENSOR_DIRECTION CCW
#define MOTOR_ANGLE_OFFSET 2.54f  // Offset for "zero" position

// Position control parameters
#define POSITION_GAIN 0.5f
#define DAMPING_GAIN 0.01f

#endif // MAIN_CONFIG_H