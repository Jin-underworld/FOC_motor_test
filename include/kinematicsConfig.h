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
#define GLOBAL_TO_JOINT0_LENGTH 62.5f
#define LINK0_LENGTH 160.3f
#define LINK1_LENGTH 160.5f
#define LINK2_LENGTH 120.0f
#define LINK2_DH_LENGTH 152.61f 
#define LINK3_LENGTH_NO_ANGLE 86.9f
#define LINK3_ANGLED_OFFSET_LENGTH 94.0f
#define LINK3_ANGLE 65.0f//33.93
#define ENDEFF_FROM_J3 17.0f//92.65
#define ANGLE_BTW_LINK1_LINK2 46.22f
#define fucked_Angle 31.07f

// wrist joints. length represents the distnace of the "height" of link 4. the other links propogate
// from the height of link 4. THEREFORE, link4 breadth is the same as link5 breadth (since they are at the same joint)
// #define LINK4_LENGTH 92.65f
// #define LINK4_BREADTH 68.25f // same as link5 breadth
// #define LINK5_LENGTH 47.25f
// #define LINK5_BREADTH LINK4_BREADTH
// #define LINK6_LENGTH LINK5_LENGTH
// #define LINK6_BREADTH 25.0f
// #define ENDEFF_OFFSET LINK6_BREADTH

// DH parameters for your robot (old one)
// global frame is below J0!! 
// frames are all defined AT the joint itself. hence R, should tbe shortest distance between frames along x.
// 5 columns, 0T1, 1T2, 2T3, 3T4, 4T5, 5T6
// For wrist, only dist from j3 to end eff matters : https://robotics.stackexchange.com/questions/22613/4-axis-gimbal-nodal-robot-dh-parameters
// static const float THETA_OFFSET[NUMBER_OF_JOINTS-1] = {0.0f        , 0.0f        , DEG2RAD(LINK3_ANGLE), 0.0f          ,-M_PI/2, M_PI/2}; // Angle around z axis (deg)
// static const float ALPHA_DH[NUMBER_OF_JOINTS-1] =     {M_PI/2      , M_PI        , -M_PI/2             , M_PI/2        , M_PI/2,-M_PI/2}; // Twist angle around X axis (deg)
// static const float D_DH[NUMBER_OF_JOINTS-1]     =     {LINK0_LENGTH, 15.25f      , 1.75f               , ENDEFF_FROM_J3, 0     , 0     }; // Link offset along Z axis (mm)
// static const float R_DH[NUMBER_OF_JOINTS-1]     =     {1.25f       , LINK1_LENGTH, 152.61f             , 0             , 0     , 0     }; // Link length along X axis (mm)

// DH parameters for your robot (Jin bro science)
// global frame is below J0!! 
// frames are all defined AT the joint itself. hence R, should tbe shortest distance between frames along x.
// 5 columns, 0T1, 1T2, 2T3, 3T4, 4T5, 5T6
// For wrist, only dist from j3 to end eff matters : https://robotics.stackexchange.com/questions/22613/4-axis-gimbal-nodal-robot-dh-parameters
static const float D_DH[NUMBER_OF_JOINTS-1]         = {LINK0_LENGTH, 0.0f        , 0.0f                , ENDEFF_FROM_J3, 0.0f     , 0.0f     }; // D is Link offset along Z axis (mm)
static const float THETA[NUMBER_OF_JOINTS-1]        = {0.0f        , 0.0f        ,-DEG2RAD(LINK3_ANGLE), 0.0f          , 0.0f  ,    M_PI/2}; // theta Angle around z axis (rad)
static const float R_DH[NUMBER_OF_JOINTS-1]         = {0.0f        , LINK1_LENGTH, LINK2_DH_LENGTH     , 0.0f          , 0.0f     , 0.0f     }; // R/A Link length along X axis (mm)
static const float ALPHA_DH[NUMBER_OF_JOINTS-1]     = {M_PI/2      ,  M_PI        , M_PI/2             ,-M_PI/2        , M_PI/2   , M_PI/2}; // Alpha Twist angle around X axis (rad)
static const float DH_ZERO_OFFSET[NUMBER_OF_JOINTS-1]={0.0f        , 0.0f        , 0.0f                ,0.0f        , M_PI     , M_PI/2}; // Offset between DH ZERO(the x axis) and the motor calibrated zero (rad)


//GPT changes to try fix
// ALPHA_DH[1] = -M_PI (should be 0.0f for a standard planar shoulder→elbow link)
// DH_ZERO_OFFSET[3] = -M_PI/2 (should be 0.0f unless that joint is physically mounted 90° rotated)

//saved, translation okay but pitch getting tied to yaw
// static const float D_DH[NUMBER_OF_JOINTS-1]         = {LINK0_LENGTH, 0.0f        , 0.0f                , ENDEFF_FROM_J3, 0.0f     , 0.0f     }; // Link offset along Z axis (mm)
// static const float THETA[NUMBER_OF_JOINTS-1]        = {0.0f        , 0.0f        ,-DEG2RAD(LINK3_ANGLE), 0.0f          , M_PI/2   , M_PI/2}; // Angle around z axis (rad)
// static const float R_DH[NUMBER_OF_JOINTS-1]         = {0.0f        , LINK1_LENGTH, LINK2_DH_LENGTH     , 0.0f          , 0.0f     , 0.0f     }; // Link length along X axis (mm)
// static const float ALPHA_DH[NUMBER_OF_JOINTS-1]     = {M_PI/2      , -M_PI        , M_PI/2             ,-M_PI/2        , M_PI/2   ,-M_PI/2}; // Twist angle around X axis (rad)
// static const float DH_ZERO_OFFSET[NUMBER_OF_JOINTS-1]={0.0f        , 0.0f        , 0.0f                ,-M_PI/2         , -M_PI     , -M_PI/2}; // Offset between DH ZERO(the x axis) and the motor calibrated zero (rad)


// Base to first joint transform
inline static const float TG0[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, GLOBAL_TO_JOINT0_LENGTH},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

// End-effector transform
inline static const float T6E[4][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

#endif // KINEM_CONFIG_HPP