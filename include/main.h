#include "boardConfig.h"
#include "encoderConfig.h"
#include "boardConfig.h"

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// Physical parameters for gravity compensation
#define ARM_LENGTH 0.1f    // 100mm = 0.1m
#define LOAD_MASS 0.02f    // 20g = 0.02kg
#define GRAVITY 9.81f      // m/s^2
#define VOLTAGE_TO_TORQUE_RATIO 0.0260f  // TUNED [Nm/V]

// Motor configuration
#define MOTOR_POLE_PAIRS 7
#define MOTOR_PHASE_RESISTANCE 3.0f
#define MOTOR_VOLTAGE_LIMIT 12.0f
#define MOTOR_PID_VELOCITY_P 0.04f
#define MOTOR_PID_VELOCITY_I 0.01f
#define MOTOR_PID_VELOCITY_D 0.0f
#define MOTOR_PID_VELOCITY_RAMP 1000.0f
#define MOTOR_P_ANGLE 2.0f
#define MOTOR_VELOCITY_LIMIT 20.0f
#define MOTOR_ZERO_ELECTRIC_ANGLE 2.21f
#define MOTOR_SENSOR_DIRECTION CCW
#define MOTOR_ANGLE_OFFSET 2.54f  // Offset for "zero" position

// Pin configuration (Arduino Uno/Mega)
#define PIN_PWM_A 3
#define PIN_PWM_B 5
#define PIN_PWM_C 6
#define PIN_ENABLE 8

// Position control parameters
#define POSITION_GAIN 0.5f
#define DAMPING_GAIN 0.01f

#endif // MAIN_CONFIG_H