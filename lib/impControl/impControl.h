#include <SimpleFOC.h>
#include "../../include/main.h"

#ifndef IMP_CONTROL_H
#define IMP_CONTROL_H

#include <SimpleFOC.h>
#include "../../include/main.h"
#include "forwardKinematics.h"
#include "jacobianMatrix.h"

// Impedance control state structure
struct ImpedanceState {
    float k_gain;              // Base stiffness gain from Unity
    float b_gain;              // Base damping gain from Unity
    ImpedanceStateCode current_state;
    float desired_pos[3];      // Desired position [x, y, z] in meters
    float current_pos[3];      // Current position [x, y, z] in meters
    float current_vel[3];      // Current velocity [vx, vy, vz] in m/s
    float cartesian_force[6];  // Force in Cartesian space [Fx, Fy, Fz, Tx, Ty, Tz]
    float joint_torques[NUMBER_OF_JOINTS];    // Commanded joint torques [tau0, tau1, tau2]
    uint32_t last_unity_update; // Timestamp of last Unity message
    bool unity_connected;      // Flag for Unity connection status
};

// Global impedance state
extern ImpedanceState imp_state;

// Initialization function
void initializeImpControl();

// Main control loop function
float* loopImpControl();

// Parse Unity data from serial
void parseUnityData();

// Apply impedance parameters from lookup table
void applyImpedanceState(ImpedanceStateCode state_code);

// Get impedance parameters for a given state code
ImpedanceParams getImpedanceParams(ImpedanceStateCode state_code);

// Calculate Cartesian force based on impedance model
void calculateCartesianForce();

// Transform Cartesian force to joint torques using Jacobian transpose
void calculateJointTorques(const Jacobian &J);

// Apply directional stiffness/damping based on end-effector orientation
void applyDirectionalGains(float force[3], const Pose6D &pose);

// Utility: Extract rotation matrix from pose
void poseToRotationMatrix(const Pose6D &pose, float R[3][3]);

// Print debug information
void printImpedanceState();

#endif // IMP_CONTROL_H