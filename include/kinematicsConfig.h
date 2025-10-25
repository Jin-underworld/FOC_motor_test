#pragma once
#ifndef KINEM_CONFIG_HPP
#define KINEM_CONFIG_HPP

#include <math.h>

// Utility macro for degrees to radians
#define DEG2RAD(x) ((x) * M_PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f/ M_PI)

#define GRAVITY 9.81f      // m/s^2
//global frame is the centre of the base, below J0, at table level
#define NUMBER_OF_JOINTS 3

#define GLOBAL_TO_JOINT0_LENGTH 62.5f
#define LINK0_LENGTH 160.0f
#define LINK1_LENGTH 160.5f
#define LINK2_LENGTH 120.0f
#define ANGLED_OFFSET_LENGTH 80.0f
#define ANGLE_BTW_LINK1_LINK2 46.22f

// Pre-compute values needed for transforms (at compile time)
#define LINK2_Z_OFFSET (LINK2_LENGTH*cos(DEG2RAD(ANGLE_BTW_LINK1_LINK2/2)))
#define LINK2_X_OFFSET (LINK2_LENGTH*sin(DEG2RAD(ANGLE_BTW_LINK1_LINK2/2))+ANGLED_OFFSET_LENGTH)

// DH parameters for your robot
// global frame is below J0!!
static const float THETA_OFFSET[NUMBER_OF_JOINTS] = {0.0f, 0.0f, 0.0f};  // Angle around z axis (deg)
static const float ALPHA_DH[NUMBER_OF_JOINTS] = {M_PI/2, M_PI, 0.0f};      // Twist angle around X axis (deg)
static const float D_DH[NUMBER_OF_JOINTS]     = {LINK0_LENGTH, 0.0f, 0.0f}; // Link offset along Z axis (mm)
static const float R_DH[NUMBER_OF_JOINTS]     = {0.0f, LINK1_LENGTH, LINK2_LENGTH}; // Link length along X axis (mm)

// Base to first joint transform
inline static const float TG0[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, GLOBAL_TO_JOINT0_LENGTH},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

// End-effector transform
inline static const float T3E[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, -(LINK2_Z_OFFSET)},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

#endif // KINEM_CONFIG_HPP