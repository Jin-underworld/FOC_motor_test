#pragma once
#ifndef RNEA_CONFIG_HPP
#define RNEA_CONFIG_HPP

#include <math.h>
#include "kinematicsConfig.h"

// Number of links (including base and end effector)
#define NUM_LINKS 8  // Links 0-7 (world, b0, l1-l6, endEff)
#define NUM_ACTUATED_JOINTS 6  // Joints 0-6 (j0-j6)
//NUM of joints is in kinematicsConfig.h

// this is a safeguard!! jic you try making links < joints
static_assert(NUM_LINKS >= 2, "NUM_LINKS must be >= 2");
static_assert(NUM_ACTUATED_JOINTS <= NUMBER_OF_JOINTS, "NUM_ACTUATED_JOINTS must be <= NUMBER_OF_JOINTS");

// Link masses (kg) - from URDF inertial data
static const float LINK_MASS[NUM_LINKS] = {
    1.22658f,   // world/base
    1.22658f,   // b0
    0.06916f,   // l1
    0.08100f,   // l2
    0.13300f,   // l3
    0.16164f,   // l4
    0.27805f,   // l5
    0.05300f    // l6
};

// Center of mass positions in link frame [x, y, z] (meters)
// These are from URDF <origin xyz="..."> in <inertial> tags
static const float LINK_COM[NUM_LINKS][3] = {
    {0.00173f, 0.00000f, 0.03401f},    // world
    {0.00173f, 0.00000f, -0.02849f},   // b0
    {0.02474f, 0.00000f, 0.03712f},    // l1
    {0.04637f, 0.00006f, 0.05186f},    // l2
    {0.00004f, 0.06406f, 0.07141f},    // l3
    {0.10135f, -0.05707f, 0.00597f},   // l4
    {0.12739f, 0.00003f, 0.01582f},    // l5
    {0.03028f, 0.00000f, 0.03405f}     // l6
};

// Inertia tensors in link frame [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] (kg*m^2)
// Full 3x3 inertia matrix can be reconstructed from these 6 values
static const float LINK_INERTIA[NUM_LINKS][6] = {
    // world
    {0.00295475f, 0.00319055f, 0.00565514f, -1.46892e-9f, 8.36407e-5f, 6.50851e-8f},
    // b0
    {0.00295475f, 0.00319055f, 0.00565514f, -1.46892e-9f, 8.36407e-5f, 6.50851e-8f},
    // l1
    {3.44553e-5f, 5.20602e-5f, 3.04824e-5f, 6.50e-10f, 6.13976e-6f, -2.46e-10f},
    // l2
    {7.16698e-5f, 1.00178e-4f, 4.40458e-5f, 3.597e-9f, 2.74637e-5f, 2.557e-8f},
    // l3
    {2.65863e-4f, 2.00036e-4f, 9.55929e-5f, 1.30926e-7f, -8.412e-8f, 7.33917e-5f},
    // l4
    {2.84065e-4f, 5.13661e-4f, 7.03904e-4f, -2.68736e-4f, -8.62867e-5f, 5.00591e-5f},
    // l5
    {1.00863e-4f, 1.22395e-3f, 1.23984e-3f, 3.04044e-7f, -4.14680e-5f, 1.91627e-7f},
    // l6
    {3.01355e-5f, 3.77370e-5f, 1.81564e-5f, -1.80e-10f, 1.00021e-5f, -1.60e-10f}
};

// Joint axis directions in link frame (for revolute joints, always Z-axis in DH convention)
// But we keep this for flexibility
static const float JOINT_AXIS[NUM_ACTUATED_JOINTS][3] = {
    {0.0f, 0.0f, 1.0f},  // j0 - rotation about Z
    {0.0f, 0.0f, -1.0f}, // j1 - rotation about -Z (from URDF)
    {0.0f, 0.0f, -1.0f}, // j2 - rotation about -Z
    {0.0f, 0.0f, -1.0f}, // j3 - rotation about -Z
    {0.0f, 0.0f, -1.0f}, // j4 - rotation about -Z
    {0.0f, 0.0f, -1.0f}, // j5 - rotation about -Z
};

// Gravity compensation enable flags (only joints 1 and 2)
static const bool GCOMP_ENABLE[NUM_ACTUATED_JOINTS] = {
    false,  // j0 - no gcomp (swivel joint)
    true,   // j1 - gcomp enabled
    true,   // j2 - gcomp enabled
    false,  // j3 - no gcomp (gimbal)
    false,  // j4 - no gcomp (gimbal)
    false,  // j5 - no gcomp (gimbal)
};

// Motor torque constants (N*m/A) - from motorConfig.h
// Used to convert RNEA torques to motor currents
// this one i also anyhow write de
static const float MOTOR_KT[NUM_ACTUATED_JOINTS] = {
    0.61f,  // j0 - GB6025
    0.61f,  // j1 - GB6010
    0.28f,  // j2 - GB4315
    0.17f,  // j3 - GB3510
    0.17f,  // j4 - GB3510
    0.17f,  // j5 - GB3510
};

// Gear ratios (if any) - set to 1.0 if direct drive
static const float GEAR_RATIO[NUM_ACTUATED_JOINTS] = {
    1.0f,  // j0
    1.0f,  // j1
    1.0f,  // j2
    1.0f,  // j3
    1.0f,  // j4
    1.0f,  // j5
};

// Friction compensation parameters (optional)
struct FrictionParams {
    float coulomb_pos;  // Coulomb friction (positive velocity)
    float coulomb_neg;  // Coulomb friction (negative velocity)
    float viscous;      // Viscous friction coefficient
    float stiction;     // Static friction
};

static const FrictionParams JOINT_FRICTION[NUM_ACTUATED_JOINTS] = {
    {0.0f, 0.0f, 0.0f, 0.0f},  // j0
    {0.05f, -0.05f, 0.01f, 0.08f},  // j1 - tune these i anyyhow put de
    {0.05f, -0.05f, 0.01f, 0.08f},  // j2 - tune these
    {0.0f, 0.0f, 0.0f, 0.0f},  // j3
    {0.0f, 0.0f, 0.0f, 0.0f},  // j4
    {0.0f, 0.0f, 0.0f, 0.0f},  // j5
};

#endif // RNEA_CONFIG_HPP