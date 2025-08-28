#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <SimpleFOC.h>
#include "../../../include/main.h"

class MotorControl {
public:
    MotorControl();
    ~MotorControl();

    // Initialize motor, driver, and sensor
    void init();
    
    // Main FOC algorithm function
    void loopFOC();
    
    // Run gravity compensation
    void compensateGravity();
    
    // Add monitoring support
    void useMonitoring(Stream &serial);
    
    // Print current state
    void monitor();

private:
    // Hardware components
    MagneticSensorI2C sensor;
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    
    // Gravity compensation functions
    float calculateGravityTorque(float angle);
    float torqueToVoltage(float torque);
    
    // State variables
    float max_torque;
    float current_angle;
    float last_angle;
};

#endif // MOTOR_CONTROL_H