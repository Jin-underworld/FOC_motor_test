#ifndef MOTOR_HELPER_H
#define MOTOR_HELPER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "../../include/main.h"

// in the order of base_joint (joint0, sensor0) to joint 5 (total 6 joints, zero indexing/)
// gb6025, gb6010, gb4315, gb3510, gb2806, gb2806
// declarations should be in this order!

// Extended ExtendedBLDCMotor class with torque constant
class ExtendedBLDCMotor : public BLDCMotor {
public:
    // Constructor - passes parameters to base ExtendedBLDCMotor class
    ExtendedBLDCMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET, float Kt = NOT_SET)
        : BLDCMotor(pp, R, KV, L), torque_constant(Kt) {
    }
    
    // Torque constant [Nm/A] - can be set/accessed directly
    float torque_constant;
    
    // Helper function to calculate torque from current
    float getCurrentTorque() {
        if (torque_constant == NOT_SET) return 0;
        return current.q * torque_constant;  // Assuming current sensing is available
    }
    
    // Helper function to calculate required current for desired torque
    float getTorqueToCurrentRatio(float desired_torque) {
        if (torque_constant == NOT_SET || torque_constant == 0) return 0;
        return desired_torque / torque_constant;
    }
};

// Motor and driver declarations
extern ExtendedBLDCMotor GB6025motor;
extern BLDCDriver3PWM GB6025driver;
extern ExtendedBLDCMotor GB6010motor;
extern BLDCDriver3PWM GB6010driver;
extern ExtendedBLDCMotor GB4315motor;
extern BLDCDriver3PWM GB4315driver;

// Arrays for iteration - CHANGED: Added these
extern ExtendedBLDCMotor* Joint_motor[NUMBER_OF_JOINTS];
extern float* target_angles[NUMBER_OF_JOINTS];

// Target angle variables , zero indexing 
extern float target_Angle_0;
extern float target_Angle_1;
extern float target_Angle_2;

struct MotorConfig {
    ExtendedBLDCMotor* motor;
    MagneticSensorI2C* sensor;
    float upper_limit;
    float lower_limit;
    float home_angle;
    const char* name;
};

extern LowPassFilter LPF_angle0;

// Function declarations
void initializeMotorDrivers();
void initializeMotorControllers();
void initializeMotorParameters();
void linkMotorsToSensors();
void initializeMotors();
bool checkstarting(const MotorConfig& config);
bool goToHomePosition(const MotorConfig& config, unsigned long timeout_ms = 5000);
void doTarget(char* cmd);
void printRawSensorAngles(void);
float getMotorRawAngle(ExtendedBLDCMotor* motor);


// Motor config instances
extern MotorConfig gb6025_config;
extern MotorConfig gb6010_config;
extern MotorConfig gb4315_config;





#endif // MOTOR_HELPER_H