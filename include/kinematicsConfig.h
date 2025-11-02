#pragma once
#ifndef KINEM_CONFIG_HPP
#define KINEM_CONFIG_HPP

#include <math.h>

// Utility macro for degrees to radians
#define DEG2RAD(x) ((x) * M_PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f/ M_PI)

#define GRAVITY 9.81f      // m/s^2
//global frame is the centre of the base, below J0, at table level
#define NUMBER_OF_JOINTS 7
#define ACTIVE_JOINTS 4

// translational joints
#define GLOBAL_TO_JOINT0_LENGTH 0.0625f 
#define LINK0_LENGTH 0.1603f   
#define LINK1_LENGTH 0.1605f
#define LINK2_LENGTH 0.1200f
#define LINK3_LENGTH_NO_ANGLE 0.0869f 
#define LINK3_ANGLED_OFFSET_LENGTH 0.0940f
#define LINK3_ANGLE 115.0f
#define ENDEFF_FROM_J3 0.09265f 
#define ANGLE_BTW_LINK1_LINK2 46.22f

// wrist joints. length represents the distnace of the "height" of link 4. the other links propogate
// from the height of link 4. THEREFORE, link4 breadth is the same as link5 breadth (since they are at the same joint)
// #define LINK4_LENGTH 92.65f
// #define LINK4_BREADTH 68.25f // same as link5 breadth
// #define LINK5_LENGTH 47.25f
// #define LINK5_BREADTH LINK4_BREADTH
// #define LINK6_LENGTH LINK5_LENGTH
// #define LINK6_BREADTH 25.0f
// #define ENDEFF_OFFSET LINK6_BREADTH

// DH parameters for your robot
// global frame is below J0!! 
// frames are all defined AT the joint itself. hence R, should tbe shortest distance between frames along x.
// 6 columns, 0T1, 1T2, 2T3, 3T4, 4T5, 5T6
// For wrist, only dist from j3 to end eff matters : https://robotics.stackexchange.com/questions/22613/4-axis-gimbal-nodal-robot-dh-parameters
static const float THETA_OFFSET[NUMBER_OF_JOINTS-1] = {0.0f, 0.0f, DEG2RAD(LINK3_ANGLE), 0.0f, -M_PI/2, M_PI/2};  // Angle around z axis (deg)
static const float ALPHA_DH[NUMBER_OF_JOINTS-1] = {M_PI/2, M_PI, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2};      // Twist angle around X axis (deg)
static const float D_DH[NUMBER_OF_JOINTS-1]     = {LINK0_LENGTH, 0.01525f, 0.00175f, ENDEFF_FROM_J3, 0, 0}; // Link offset along Z axis (mm)
static const float R_DH[NUMBER_OF_JOINTS-1]     = {0.00125f, LINK1_LENGTH, 0.15261f, 0, 0, 0}; // Link length along X axis (mm)

// 152.61 is the shortest distance of frame 2 to 3. cosine rule, with lengthes (link3 and link3_angled), with angle being link 3 angle. 

// Base to first joint transform UT0
inline static const float TG0[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, GLOBAL_TO_JOINT0_LENGTH},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

// End-effector transform 6TE
inline static const float T6E[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

#endif // KINEM_CONFIG_HPP