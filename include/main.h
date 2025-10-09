#define SELECTED_BOARD BOARD_STM32
#include "boardConfig.h"

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// Physical parameters for gravity compensation
#define ARM_LENGTH 0.1f    // 100mm = 0.1m
#define LOAD_MASS 0.06f    // 20g = 0.02kg
#define GRAVITY 9.81f      // m/s^2
#define VOLTAGE_TO_TORQUE_RATIO 2.13f  // TUNED [Nm/V]

//Motor driver configuration
#define driver_voltage_supply 24.0f

#define BUTTON_PIN PA0

// Motor configuration 4315
#define motor_GB4315_PWM_A PC8 //IN1
#define motor_GB4315_PWM_B PC7 //In2
#define motor_GB4315_PWM_C PC6  //IN3
#define motor_GB4315_PP 14
#define motor_GB4315_R 13.84
#define motor_GB4315_KV 28
#define motor_GB4315_inductance 6.45
#define motor_GB4315_ZEAngle 5.66
#define motor_GB4315_sensor_direction Direction::CCW
#define motor_GB4315_ZeroCalibratedAngle -6.06f - M_PI
#define motor_GB4315_HomeAngle 4.0f  // Define the home position angle (in radians)
#define motor_GB4315_UPPER_LIMIT 4.96f
#define motor_GB4315_LOWER_LIMIT 1.0f


// // Motor configuration
#define motor_GB6010_PWM_A PE13 //IN1
#define motor_GB6010_PWM_B PE11 //In2
#define motor_GB6010_PWM_C PE9  //IN3
#define motor_GB6010_PP 14
#define motor_GB6010_R 10.28
#define motor_GB6010_KV 16
#define motor_GB6010_inductance 12.35e-3
#define motor_GB6010_ZEAngle 6.11f
#define motor_GB6010_sensor_direction Direction::CCW
#define motor_GB6010_ZeroCalibratedAngle -2.94f //IMPT!!!! code minus this away. so for angle use angle as is when at 0 position. PI adjustment is to avoid negative angles
#define motor_GB6010_HomeAngle 1.0f  // Define the home position angle (in radians)
#define motor_GB6010_UPPER_LIMIT 1.585f
#define motor_GB6010_LOWER_LIMIT -0.87f

// Position control parameters
#define POSITION_GAIN 0.5f
#define DAMPING_GAIN 0.01f

#define LowPassVelFilter 0.03f
#define LowPassCurrFilter 0.02f
#define LowPassTorqueFilter 0.02f

#endif // MAIN_CONFIG_H