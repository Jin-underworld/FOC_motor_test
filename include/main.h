#define SELECTED_BOARD BOARD_STM32_DISCO
#include "boardConfig.h"
#include "motorConfig.h"
#include "kinematicsConfig.h"
#include "impControlConfig.h"

#ifndef MAIN_CONFIG_H
#define MAIN_CONFIG_H

//Motor driver configuration
#define driver_voltage_supply 24.0f

// Position control parameters
#define POSITION_GAIN 0.5f
#define DAMPING_GAIN 0.01f

#define LowPassVelFilter 0.03f
#define LowPassCurrFilter 0.02f
#define LowPassTorqueFilter 0.02f

#endif // MAIN_CONFIG_H