#ifndef MOTOR_HELPER_H
#define MOTOR_HELPER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "../../include/main.h"

// in the order of base_joint (joint0, sensor0) to joint 5 (total 6 joints, zero indexing/)
// gb6025, gb6010, gb4315, gb3510, gb2806, gb2806
// declarations should be in this order!

// Motor and driver declarations
extern BLDCMotor GB6025motor;
extern BLDCDriver3PWM GB6025driver;
extern BLDCMotor GB6010motor;
extern BLDCDriver3PWM GB6010driver;
extern BLDCMotor GB4315motor;
extern BLDCDriver3PWM GB4315driver;

// Target angle variables , zero indexing 
extern float target_Angle_0;
extern float target_Angle_1;
extern float target_Angle_2;

struct MotorConfig {
    BLDCMotor* motor;
    MagneticSensorI2C* sensor;
    float upper_limit;
    float lower_limit;
    float home_angle;
    const char* name;
};

// Function declarations
void initializeMotorDrivers();
void initializeMotorControllers();
void initializeMotorParameters();
void linkMotorsToSensors();
void initializeMotors();
bool checkstarting(const MotorConfig& config);
bool goToHomePosition(const MotorConfig& config, unsigned long timeout_ms = 5000);
void doTarget(char* cmd);

// Motor config instances
extern MotorConfig gb6025_config;
extern MotorConfig gb6010_config;
extern MotorConfig gb4315_config;

#endif // MOTOR_HELPER_H