#define SELECTED_BOARD BOARD_STM32_DISCO
#include "boardConfig.h"
#include "motorConfig.h"
#include "kinematicsConfig.h"

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

// Physical parameters for gravity compensation
// #define ARM_LENGTH 0.1f    // 100mm = 0.1m
// #define LOAD_MASS 0.06f    // 20g = 0.02kg
// #define GRAVITY 9.81f      // m/s^2
// #define VOLTAGE_TO_TORQUE_RATIO 2.13f  // TUNED [Nm/V]

//Motor driver configuration
#define driver_voltage_supply 24.0f

// Position control parameters
#define POSITION_GAIN 0.5f
#define DAMPING_GAIN 0.01f

#define LowPassVelFilter 0.03f
#define LowPassCurrFilter 0.02f
#define LowPassTorqueFilter 0.02f

#endif // MAIN_CONFIG_H