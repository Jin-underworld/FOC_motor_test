#define SELECTED_BOARD BOARD_ARDUINO_MEGA
#include "boardConfig.h"

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// Physical parameters for gravity compensation
#define ARM_LENGTH 0.1f    // 100mm = 0.1m
#define LOAD_MASS 0.06f    // 20g = 0.02kg
#define GRAVITY 9.81f      // m/s^2
#define VOLTAGE_TO_TORQUE_RATIO 2.13f  // TUNED [Nm/V]


// // Motor configuration
// #define MOTOR_POLE_PAIRS 14 
// #define MOTOR_PHASE_RESISTANCE 13.84f
// #define MOTOR_TORQUE_CONSTANT 0.00047f
// #define MOTOR_PHASE_INDUCTANCE 0.0645f
// #define MOTOR_VOLTAGE_LIMIT 12.0f
// #define MOTOR_PID_VELOCITY_P 0.5f
// #define MOTOR_PID_VELOCITY_I 0.0f
// #define MOTOR_PID_VELOCITY_D 0.0f
// #define MOTOR_PID_VELOCITY_RAMP 1000.0f
// #define MOTOR_PID_VELOCITY_LIMIT 5.0f


// Motor configuration
#define MOTOR_POLE_PAIRS 14 
#define MOTOR_PHASE_RESISTANCE 10.28f
#define MOTOR_TORQUE_CONSTANT 4.2f
#define MOTOR_PHASE_INDUCTANCE 0.123f
#define MOTOR_VOLTAGE_LIMIT 12.0f
#define MOTOR_PID_VELOCITY_P 0.1f
#define MOTOR_PID_VELOCITY_I 0.0f
#define MOTOR_PID_VELOCITY_D 0.0f
#define MOTOR_PID_VELOCITY_RAMP 1000.0f
#define MOTOR_PID_VELOCITY_LIMIT 5.0f

#define MOTOR_P_ANGLE 2.0f
#define MOTOR_VELOCITY_LIMIT 20.0f
#define MOTOR_ZERO_ELECTRIC_ANGLE 1.30f
#define MOTOR_SENSOR_DIRECTION CW
#define MOTOR_ANGLE_OFFSET 2.54f  // Offset for "zero" position

// Position control parameters
#define POSITION_GAIN 0.5f
#define DAMPING_GAIN 0.01f

#define LowPassVelFilter 0.03f
#define LowPassCurrFilter 0.02f
#define LowPassTorqueFilter 0.02f

#endif // MAIN_CONFIG_H