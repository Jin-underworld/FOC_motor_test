#pragma once
#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "../../include/main.h"

// Pose structure to store position and orientation
typedef struct {
    float x, y, z;
    float roll, pitch, yaw;
} Pose6D;

float getSensorAngleKinematics(int joint);
void matMul4x4(const float A[4][4], const float B[4][4], float R[4][4]);
void dhTransform(float theta, float alpha, float d, float r, float T[4][4]);
Pose6D forwardKinematics(void);

void printPose(void);

// Overloaded function to print a specific pose
void printPose(const Pose6D &pose);

#endif // FORWARD_KINEMATICS_H