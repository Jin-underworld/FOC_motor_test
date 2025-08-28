#pragma once
#ifndef MOTOR_CONFIG_HPP
#define MOTOR_CONFIG_HPP

#include <SimpleFOC.h>

// Motor type enumeration
enum class MotorType {
    GB2806,
    GB3510,
    GB4315,
    GB6010,
    GB6025,
    NONE  // For joints without motors
};

// Motor configuration struct
struct MotorConfig {
    int polePairs;
    float phaseResistance;
    float voltageLimit;
    float voltageToTorqueRatio;
    float pidVelocityP;
    float pidVelocityI;
    float pidVelocityD;
    float velocityRamp;
    float pAngle;
    float velocityLimit;
    float zeroElectricAngle;
    Direction sensorDirection;
};

// Default motor configurations
namespace DefaultMotors {
    constexpr MotorConfig GB2806 = {
        .polePairs = 7,
        .phaseResistance = 3.0f,
        .voltageLimit = 12.0f,
        .voltageToTorqueRatio = 0.018870f,
        .pidVelocityP = 0.01f,
        .pidVelocityI = 0.1f,
        .pidVelocityD = 0.0f,
        .velocityRamp = 1000.0f,
        .pAngle = 2.0f,
        .velocityLimit = 20.0f,
        .zeroElectricAngle = 2.21f,
        .sensorDirection = Direction::CCW
    };

    constexpr MotorConfig GB3510 = {
        .polePairs = 10,
        .phaseResistance = 2.5f,
        .voltageLimit = 12.0f,
        .voltageToTorqueRatio = 0.022f,
        .pidVelocityP = 0.015f,
        .pidVelocityI = 0.12f,
        .pidVelocityD = 0.0f,
        .velocityRamp = 1200.0f,
        .pAngle = 2.2f,
        .velocityLimit = 22.0f,
        .zeroElectricAngle = 2.3f,
        .sensorDirection = Direction::CCW
    };

    constexpr MotorConfig GB4315 = {
        .polePairs = 12,
        .phaseResistance = 2.0f,
        .voltageLimit = 24.0f,
        .voltageToTorqueRatio = 0.03f,
        .pidVelocityP = 0.02f,
        .pidVelocityI = 0.15f,
        .pidVelocityD = 0.001f,
        .velocityRamp = 1500.0f,
        .pAngle = 2.5f,
        .velocityLimit = 25.0f,
        .zeroElectricAngle = 2.4f,
        .sensorDirection = Direction::CCW
    };

    constexpr MotorConfig GB6010 = {
        .polePairs = 14,
        .phaseResistance = 1.5f,
        .voltageLimit = 24.0f,
        .voltageToTorqueRatio = 0.035f,
        .pidVelocityP = 0.025f,
        .pidVelocityI = 0.18f,
        .pidVelocityD = 0.001f,
        .velocityRamp = 1800.0f,
        .pAngle = 2.8f,
        .velocityLimit = 30.0f,
        .zeroElectricAngle = 2.5f,
        .sensorDirection = Direction::CCW
    };

    constexpr MotorConfig GB6025 = {
        .polePairs = 16,
        .phaseResistance = 1.2f,
        .voltageLimit = 24.0f,
        .voltageToTorqueRatio = 0.04f,
        .pidVelocityP = 0.03f,
        .pidVelocityI = 0.2f,
        .pidVelocityD = 0.001f,
        .velocityRamp = 2000.0f,
        .pAngle = 3.0f,
        .velocityLimit = 35.0f,
        .zeroElectricAngle = 2.6f,
        .sensorDirection = Direction::CCW
    };
}

// Physical parameters
constexpr float ARM_LENGTH = 0.1f;    // 100mm = 0.1m
constexpr float LOAD_MASS = 0.02f;    // 20g = 0.02kg
constexpr float GRAVITY = 9.81f;      // m/s^2

#endif // MOTOR_CONFIG_HPP