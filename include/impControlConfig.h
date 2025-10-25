#pragma once
#ifndef IMP_CONTROL_CONFIG_H
#define IMP_CONTROL_CONFIG_H

#include <stdint.h>

enum ImpedanceStateCode : uint8_t {
    IMP_FREE_AIR = 0x00,           // Free movement in air
    IMP_SKIN_CONTACT = 0x01,       // Initial contact with skin
    IMP_SKIN_PIERCING = 0x02,      // Piercing through skin (high force)
    IMP_SHALLOW_CUT = 0x03,        // Shallow cut (medium resistance)
    IMP_DEEP_CUT = 0x04,           // Deep cut (low resistance)
    IMP_BONE_CONTACT = 0x05,       // Contact with bone (very high resistance)
    IMP_INVALID = 0xFF             // Invalid/error state
};

// Impedance parameter structure
struct ImpedanceParams {
    float k_gain;    // Stiffness gain
    float b_gain;    // Damping gain
    const char* description;
};

// Lookup table for impedance parameters
// Maps ImpedanceStateCode to K and B gains
static const ImpedanceParams IMP_LOOKUP_TABLE[] = {
    // K      B       Description
    {0.0f,   1.0f,   "Free air movement"},                    // IMP_FREE_AIR
    {8.0f,   0.8f,   "Initial skin contact"},                 // IMP_SKIN_CONTACT
    {15.0f,  0.5f,   "Piercing skin (high resistance)"},      // IMP_SKIN_PIERCING
    {3.0f,   0.6f,   "Shallow cut"},                          // IMP_SHALLOW_CUT
    {0.5f,   0.7f,   "Deep cut (low resistance)"},            // IMP_DEEP_CUT
    {20.0f,  0.3f,   "Bone contact (very high)"},             // IMP_BONE_CONTACT
};

// Number of states in lookup table
#define IMP_NUM_STATES (sizeof(IMP_LOOKUP_TABLE) / sizeof(ImpedanceParams))

// Default impedance parameters for free air movement
#define DEFAULT_K_STIFFNESS 0.0f      // No stiffness in free air
#define DEFAULT_B_DAMPING 1.0f        // Base damping (knife inertia)

// Directional stiffness multipliers (relative to knife orientation)
// X: perpendicular to knife edge (blunt side) - HIGH resistance
// Y: along knife edge (sharp side) - LOW resistance  
// Z: perpendicular to knife flat - MEDIUM resistance
#define K_MULTIPLIER_X 5.0f   // Very high - cutting with blunt edge
#define K_MULTIPLIER_Y 0.2f    // Low - cutting along sharp edge
#define K_MULTIPLIER_Z 0.05f   // Very low - perpendicular to flat

// Directional damping multipliers
#define B_MULTIPLIER_X 1.0f
#define B_MULTIPLIER_Y 1.0f
#define B_MULTIPLIER_Z 1.0f

// Unity communication parameters
#define UNITY_BUFFER_SIZE 64
#define UNITY_TIMEOUT_MS 100

// Serial protocol format options
// Option 1: Single byte code: [0x01]
// Option 2: Hex string: "0x01" or "01"
// Option 3: Decimal string: "1"
#define UNITY_PROTOCOL_BYTE true      // true = expect single byte, false = expect string

// Torque limits (Nm)
#define MAX_JOINT_TORQUE_0 2.0f
#define MAX_JOINT_TORQUE_1 2.0f
#define MAX_JOINT_TORQUE_2 0.9f
// When NUMBER_OF_JOINTS = 6, add:
// #define MAX_JOINT_TORQUE_3 2.0f
// #define MAX_JOINT_TORQUE_4 2.0f
// #define MAX_JOINT_TORQUE_5 2.0f

// Position reference for impedance control
// When no Unity input, use current position as desired position
#define USE_CURRENT_AS_DESIRED true

#endif // IMP_CONTROL_CONFIG_H