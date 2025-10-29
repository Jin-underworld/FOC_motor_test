#pragma once
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <SimpleFOC.h>
#include "kinematicsConfig.h"  // For NUMBER_OF_JOINTS


// ============================================================================
// MOTOR CHARACTERISTICS (Hardware-specific, reusable across joints)
// ============================================================================

// GB6025 Motor Characteristics
#define MOTOR_GB6025_PP 14
#define MOTOR_GB6025_R 1.675f
#define MOTOR_GB6025_KV 20
#define MOTOR_GB6025_KT 0.23f
#define MOTOR_GB6025_INDUCTANCE 1.865e-3f
#define MOTOR_GB6025_CURRENT_LIMIT 2.0f  // Maximum continuous current, derated to 2A to keep within range of driver

// GB6010 Motor Characteristics
#define MOTOR_GB6010_PP 14
#define MOTOR_GB6010_R 5.14f
#define MOTOR_GB6010_KV 16
#define MOTOR_GB6010_KT 0.61f
#define MOTOR_GB6010_INDUCTANCE 6.175e-3f
#define MOTOR_GB6010_CURRENT_LIMIT 1.3f  // Maximum continuous current, original is 1.3, only run (1.5) with cooling

// GB4315 Motor Characteristics
#define MOTOR_GB4315_PP 14
#define MOTOR_GB4315_R 6.92f
#define MOTOR_GB4315_KV 28
#define MOTOR_GB4315_KT 0.47f
#define MOTOR_GB4315_INDUCTANCE 3.225e-3f
#define MOTOR_GB4315_CURRENT_LIMIT 0.59f  // Maximum continuous current, original is 0.59, only run (0.8) with cooling

#define MOTOR_GB3510_PP 11
#define MOTOR_GB3510_R 4.265f
#define MOTOR_GB3510_KV 80
#define MOTOR_GB3510_KT 0.2f
#define MOTOR_GB3510_INDUCTANCE 0.95e-3f
#define MOTOR_GB3510_CURRENT_LIMIT 0.53f  // Maximum continuous current, original is 0.59, only run (0.8) with cooling


// ============================================================================
// JOINT CONFIGURATION (Installation-specific, varies by joint location)
// ============================================================================

// Joint 0 Configuration (GB6025 installed as base joint)
#define JOINT0_ZE_ANGLE 4.29f
#define JOINT0_SENSOR_DIRECTION Direction::CCW
#define JOINT0_ZERO_CALIBRATED_ANGLE 0.423f//-5.880f
#define JOINT0_HOME_ANGLE 0.0f
#define JOINT0_UPPER_LIMIT 0.785f
#define JOINT0_LOWER_LIMIT -0.785f

// Joint 1 Configuration (GB6010 installed as shoulder joint)
#define JOINT1_ZE_ANGLE 6.11f//1.71f
#define JOINT1_SENSOR_DIRECTION Direction::CCW
#define JOINT1_ZERO_CALIBRATED_ANGLE -2.94f
#define JOINT1_HOME_ANGLE (M_PI/3)
#define JOINT1_UPPER_LIMIT (1.601f - 0.001f)
#define JOINT1_LOWER_LIMIT -(0.007f - 0.001f)

// Joint 2 Configuration (GB4315 installed as elbow joint)
#define JOINT2_ZE_ANGLE 5.68f
#define JOINT2_SENSOR_DIRECTION Direction::CCW
#define JOINT2_ZERO_CALIBRATED_ANGLE 4.013f//(-4.331f + 2.134f)
#define JOINT2_HOME_ANGLE (0.75f * M_PI)
#define JOINT2_UPPER_LIMIT (2.628f - 0.001f)
#define JOINT2_LOWER_LIMIT (0.005f - 0.001f)

// Joint 3 Configuration (GB3510 installed as compensator joint)
#define JOINT3_ZE_ANGLE 3.45f
#define JOINT3_SENSOR_DIRECTION Direction::CCW
#define JOINT3_ZERO_CALIBRATED_ANGLE 0.444f
#define JOINT3_HOME_ANGLE 0.0f
#define JOINT3_UPPER_LIMIT -(0.752f - 0.001f)
#define JOINT3_LOWER_LIMIT -(0.752f - 0.001f)

// Joint 4 Configuration (GB3510 installed as pitch joint)
#define JOINT4_ZE_ANGLE 3.45f
#define JOINT4_SENSOR_DIRECTION Direction::CCW
#define JOINT4_ZERO_CALIBRATED_ANGLE 0.444f
#define JOINT4_HOME_ANGLE 0.0f
#define JOINT4_UPPER_LIMIT -(0.752f - 0.001f)
#define JOINT4_LOWER_LIMIT -(0.752f - 0.001f)


// ============================================================================
// MOTOR CHARACTERISTICS LOOKUP TABLE
// ============================================================================

// Motor characteristics structure
struct MotorCharacteristics {
    uint8_t pole_pairs;
    float resistance;        // Ohms
    float kv;               // RPM/V
    float torque_constant;  // Nm/A
    float inductance;       // Henry
    float current_limit;    // MAX continuous current
    const char* name;
};

// Lookup table for motor types
enum MotorType : uint8_t {
    MOTOR_GB6025 = 0,
    MOTOR_GB6010 = 1,
    MOTOR_GB4315 = 2,
    MOTOR_GB3510 = 3,
    NUM_MOTOR_TYPES
};

// Static const lookup table (inline to avoid ODR violations)
static const MotorCharacteristics MOTOR_CHARACTERISTICS[NUM_MOTOR_TYPES] = {
    // GB6025
    {
        MOTOR_GB6025_PP,
        MOTOR_GB6025_R,
        MOTOR_GB6025_KV,
        MOTOR_GB6025_KT,
        MOTOR_GB6025_INDUCTANCE,
        MOTOR_GB6025_CURRENT_LIMIT,
        "GB6025"
    },
    // GB6010
    {
        MOTOR_GB6010_PP,
        MOTOR_GB6010_R,
        MOTOR_GB6010_KV,
        MOTOR_GB6010_KT,
        MOTOR_GB6010_INDUCTANCE,
        MOTOR_GB6010_CURRENT_LIMIT,
        "GB6010"
    },
    // GB4315
    {
        MOTOR_GB4315_PP,
        MOTOR_GB4315_R,
        MOTOR_GB4315_KV,
        MOTOR_GB4315_KT,
        MOTOR_GB4315_INDUCTANCE,
        MOTOR_GB6010_CURRENT_LIMIT,
        "GB4315"
    },
    // GB3510
    {
        MOTOR_GB3510_PP,
        MOTOR_GB3510_R,
        MOTOR_GB3510_KV,
        MOTOR_GB3510_KT,
        MOTOR_GB3510_INDUCTANCE,
        MOTOR_GB3510_CURRENT_LIMIT,
        "GB3510"
    }
};

// ============================================================================
// JOINT CONFIGURATION LOOKUP TABLE
// ============================================================================

// Joint configuration structure
struct JointConfiguration {
    MotorType motor_type;           // Which motor is installed
    float zero_electric_angle;      // Zero electric angle calibration
    Direction sensor_direction;     // Sensor counting direction
    float zero_calibrated_angle;    // Mechanical zero offset
    float home_angle;               // Home position angle
    float upper_limit;              // Upper position limit
    float lower_limit;              // Lower position limit
    const char* joint_name;
};

// Joint enumeration
enum JointNumber : uint8_t {
    JOINT_0 = 0,  // Base rotation
    JOINT_1 = 1,  // Shoulder
    JOINT_2 = 2,  // Elbow
    JOINT_3 = 3,  // Wrist roll (future)
    JOINT_4 = 4,  // Wrist pitch (future)
    // JOINT_5 = 5,  // Wrist yaw (future)
    NUM_JOINTS = NUMBER_OF_JOINTS
};

// Static const lookup table (inline to avoid ODR violations)
static const JointConfiguration JOINT_CONFIGS[NUMBER_OF_JOINTS] = {
    // JOINT_0 - Base (GB6025)
    {
        MOTOR_GB6025,
        JOINT0_ZE_ANGLE,
        JOINT0_SENSOR_DIRECTION,
        JOINT0_ZERO_CALIBRATED_ANGLE,
        JOINT0_HOME_ANGLE,
        JOINT0_UPPER_LIMIT,
        JOINT0_LOWER_LIMIT,
        "Base"
    },
    // JOINT_1 - Shoulder (GB6010)
    {
        MOTOR_GB6010,
        JOINT1_ZE_ANGLE,
        JOINT1_SENSOR_DIRECTION,
        JOINT1_ZERO_CALIBRATED_ANGLE,
        JOINT1_HOME_ANGLE,
        JOINT1_UPPER_LIMIT,
        JOINT1_LOWER_LIMIT,
        "Shoulder"
    },
    // JOINT_2 - Elbow (GB4315)
    {
        MOTOR_GB4315,
        JOINT2_ZE_ANGLE,
        JOINT2_SENSOR_DIRECTION,
        JOINT2_ZERO_CALIBRATED_ANGLE,
        JOINT2_HOME_ANGLE,
        JOINT2_UPPER_LIMIT,
        JOINT2_LOWER_LIMIT,
        "Elbow"
    },
    //JOINT_3 - Compensator (GB3510_3)
    {
        MOTOR_GB3510,
        JOINT3_ZE_ANGLE,
        JOINT3_SENSOR_DIRECTION,
        JOINT3_ZERO_CALIBRATED_ANGLE,
        JOINT3_HOME_ANGLE,
        JOINT3_UPPER_LIMIT,
        JOINT3_LOWER_LIMIT,
        "Compensator"
    },
    //JOINT_4 - pitch(GB3510_4)
    {
        MOTOR_GB3510,
        JOINT4_ZE_ANGLE,
        JOINT4_SENSOR_DIRECTION,
        JOINT4_ZERO_CALIBRATED_ANGLE,
        JOINT4_HOME_ANGLE,
        JOINT4_UPPER_LIMIT,
        JOINT4_LOWER_LIMIT,
        "Pitch"
    }
};

// ============================================================================
// HELPER FUNCTIONS (inline to avoid ODR violations)
// ============================================================================

// Get motor characteristics for a joint
inline const MotorCharacteristics& getMotorCharacteristics(int joint_index) {
    return MOTOR_CHARACTERISTICS[JOINT_CONFIGS[joint_index].motor_type];
}

// Get joint configuration
inline const JointConfiguration& getJointConfig(int joint_index) {
    return JOINT_CONFIGS[joint_index];
}

// Get complete motor info (motor + joint config combined)
struct CompleteMotorInfo {
    const MotorCharacteristics& motor;
    const JointConfiguration& joint;
};

inline CompleteMotorInfo getCompleteMotorInfo(int joint_index) {
    return {
        MOTOR_CHARACTERISTICS[JOINT_CONFIGS[joint_index].motor_type],
        JOINT_CONFIGS[joint_index]
    };
}

#endif // MOTOR_CONFIG_H
