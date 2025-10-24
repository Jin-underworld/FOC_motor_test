#pragma once
#ifndef JACOBIAN_MATRIX_H
#define JACOBIAN_MATRIX_H

#include "../forwardKinematics/forwardKinematics.h"
#include <Arduino.h>

// Structure to hold linear and angular velocities
typedef struct {
    float linear[3];   // vx, vy, vz
    float angular[3];  // wx, wy, wz
} JointVelocity;

typedef struct {
    float data[6][3];
} Jacobian;

// Calculate the Jacobian matrix for the current robot configuration
// Returns a 6x3 matrix representing the Jacobian
Jacobian calculateJacobian(void);

// Extract the rotation matrix from a transformation matrix
void extractRotationMatrix(const float T[4][4], float R[3][3]);

// Extract the position vector from a transformation matrix
void extractPosition(const float T[4][4], float p[3]);

// Perform cross product of two 3D vectors
void crossProduct(const float a[3], const float b[3], float result[3]);

// Print the Jacobian matrix
void printJacobian(void);

void printJacobian(const Jacobian &jacobian);

// Get joint velocities
void getJointVelocities(float velocities[3]);


// Calculate end-effector velocity from joint velocities using the Jacobian
void calculateEndEffectorVelocity(const Jacobian &jacobian, float joint_velocities[3], float ee_velocity[6]);

//checker function
void checkJacobian(void);

// Helper function for dot product
float dotProduct(const float a[3], const float b[3]);

// Helper function for vector magnitude
float vectorMagnitude(const float vec[3]);

// Helper function for matrix determinant (3x3)
float determinant3x3(float mat[3][3]);

#endif // JACOBIAN_MATRIX_H