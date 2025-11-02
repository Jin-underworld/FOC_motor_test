#pragma once
#ifndef RNEA_GRAV_COMP_HPP
#define RNEA_GRAV_COMP_HPP

#include <Arduino.h>
#include "rneaConfig.h"
#include "kinematicsConfig.h"

// 3D vector structure
struct Vec3 {
    float x, y, z;
    
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y,
                   z * v.x - x * v.z,
                   x * v.y - y * v.x);
    }
};

// 3x3 Rotation matrix
struct Mat3 {
    float m[3][3];
    
    Mat3() {
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                m[i][j] = (i == j) ? 1.0f : 0.0f;
    }
    
    Vec3 operator*(const Vec3& v) const {
        return Vec3(
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        );
    }
    
    Mat3 transpose() const {
        Mat3 result;
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                result.m[i][j] = m[j][i];
        return result;
    }
};

// Link state for RNEA recursion
struct LinkState {
    Vec3 omega;        // Angular velocity
    Vec3 omega_dot;    // Angular acceleration
    Vec3 v;            // Linear velocity
    Vec3 v_dot;        // Linear acceleration
    Vec3 v_com;        // Linear velocity of COM
    Vec3 v_com_dot;    // Linear acceleration of COM
    Vec3 force;        // Force at joint
    Vec3 torque;       // Torque at joint
    Mat3 R;            // Rotation matrix from previous frame
};

// RNEA Gravity Compensation Class
class RNEAGravComp {
public:
    RNEAGravComp();

    // Initialize RNEA (call in setup())
    void init();
    // Main gravity compensation loop (call in loop())
    // Returns pointer to 7-element array of joint torques
    float* loopGComp();
    // Set current joint positions (radians)
    void setJointPositions(const float* q_new);
    // Set current joint velocities (rad/s) - optional for friction comp
    void setJointVelocities(const float* qd_new);
    
    // Enable/disable friction compensation
    void enableFrictionComp(bool enable) { friction_comp_enabled = enable; }
    
    // Get last computed torques
    const float* getLastTorques() const { return tau_total; }
    
    // Debug print
    void printTorques();

private:
    // Joint states
    float q[NUMBER_OF_JOINTS];
    float qd[NUMBER_OF_JOINTS];
    float tau_grav[NUMBER_OF_JOINTS];
    float tau_friction[NUMBER_OF_JOINTS];
    float tau_total[NUMBER_OF_JOINTS];
    
    // Link states for forward and backward recursions
    LinkState link_states[NUM_LINKS];
    
    // Configuration flags
    bool friction_comp_enabled;
    bool initialized;
    
    // RNEA algorithm functions
    void forwardRecursion();
    void backwardRecursion();
    void computeFrictionTorques();
    
    // Helper functions
    Mat3 computeRotationMatrix(int joint_idx);
    Vec3 computeComPosition(int link_idx);
    Mat3 getInertiaMatrix(int link_idx);
    float computeFrictionTorque(int joint_idx, float velocity);
};

// Global instance
extern RNEAGravComp gravComp;

// Convenience function for main.cpp
inline float* loopGComp() {
    return gravComp.loopGComp();
}

#endif // RNEA_GRAV_COMP_HPP