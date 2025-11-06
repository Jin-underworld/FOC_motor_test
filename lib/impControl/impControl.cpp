#include "impControl.h"
#include "sensorHelper.h"
#include "motorHelper.h"

#include <math.h>
#include <stdio.h>

// Global impedance state
ImpedanceState imp_state;

void initializeImpControl() {
    // Initialize with default values
    imp_state.current_state = IMP_FREE_AIR;
    applyImpedanceState(IMP_FREE_AIR);
    
    // Initialize positions to zero
    for (int i = 0; i < 3; i++) {
        imp_state.desired_pos[i] = 0.0f;
        imp_state.current_pos[i] = 0.0f;
        imp_state.current_vel[i] = 0.0f;
    }
    
    // Initialize forces and torques to zero
    for (int i = 0; i < 6; i++) {
        imp_state.cartesian_force[i] = 0.0f;
    }
    
    for (int i = 0; i < 3; i++) {
        imp_state.joint_torques[i] = 0.0f;
    }
    
    imp_state.last_unity_update = 0;
    imp_state.unity_connected = false;
    
    Serial.println("Impedance Control Initialized");
    Serial.println("Waiting for Unity data: [K_gain, B_gain]");
}

float* loopImpControl() {
    // 1. Check for Unity data. if dh, defaults to the default k and b values of free air
    parseUnityData();
    
    // 2. Get current robot state
    Pose6D current_pose = forwardKinematics();
    imp_state.current_pos[0] = current_pose.x;
    imp_state.current_pos[1] = current_pose.y;
    imp_state.current_pos[2] = current_pose.z;
    
    // 3. Calculate current velocity from Jacobian
    Jacobian J = calculateJacobian();
    float joint_velocities[3];
    getJointVelocities(joint_velocities);
    
    float ee_velocity[6];
    calculateEndEffectorVelocity(J, joint_velocities, ee_velocity);
    
    imp_state.current_vel[0] = ee_velocity[0]; // vx
    imp_state.current_vel[1] = ee_velocity[1]; // vy
    imp_state.current_vel[2] = ee_velocity[2]; // vz
    
    // 4. Set desired position 
    if (USE_CURRENT_AS_DESIRED) {
        imp_state.desired_pos[0] = imp_state.current_pos[0];
        imp_state.desired_pos[1] = imp_state.current_pos[1];
        imp_state.desired_pos[2] = imp_state.current_pos[2];
    }
    
    // 5. Calculate Cartesian force: F = K(x_d - x) + B(0 - x_dot)
    calculateCartesianForce();
    
    // 6. Apply directional gains based on orientation
    float force_linear[3] = {
        imp_state.cartesian_force[0],
        imp_state.cartesian_force[1],
        imp_state.cartesian_force[2]
    };
    applyDirectionalGains(force_linear, current_pose);
    
    imp_state.cartesian_force[0] = force_linear[0];
    imp_state.cartesian_force[1] = force_linear[1];
    imp_state.cartesian_force[2] = force_linear[2];
    
    calculateJointTorques(J);
    
    // 10. Return pointer to joint torques array
    return imp_state.joint_torques;
}

void parseUnityData() {
    // Non-blocking check for serial data
    if (Serial.available() > 0) {
        uint8_t state_code = 0x00; //used only for testing, comment out if Unity connected
        //uint8_t state_code = 0xFF; //uncomment if Unity connected
        
        #if UNITY_PROTOCOL_BYTE
            // Read single byte directly
            state_code = Serial.read();
            // Clear any remaining newline characters
            while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
                Serial.read();
            }
        #else
            // Read as string (hex or decimal)
            String input = Serial.readStringUntil('\n');
            input.trim();
            
            // Try to parse as hex (0x01) or decimal (1)
            if (input.startsWith("0x") || input.startsWith("0X")) {
                // Hex format
                state_code = (uint8_t)strtol(input.c_str(), NULL, 16);
            } else {
                // Decimal format
                state_code = (uint8_t)input.toInt();
            }
        #endif
        
        // Validate state code
        if (state_code < IMP_NUM_STATES) {
            applyImpedanceState((ImpedanceStateCode)state_code);
            imp_state.last_unity_update = millis();
            imp_state.unity_connected = true;
            
            Serial.print("State changed to: 0x");
            Serial.print(state_code, HEX);
            Serial.print(" - ");
            Serial.println(IMP_LOOKUP_TABLE[state_code].description);
        } else {
            Serial.print("Invalid state code: 0x");
            Serial.println(state_code, HEX);
        }
    }
}

void applyImpedanceState(ImpedanceStateCode state_code) {
    ImpedanceParams params = getImpedanceParams(state_code);
    imp_state.k_gain = params.k_gain;
    imp_state.b_gain = params.b_gain;
    imp_state.current_state = state_code;
}

ImpedanceParams getImpedanceParams(ImpedanceStateCode state_code) {
    // Bounds check
    if (state_code >= IMP_NUM_STATES) {
        // Return default free air parameters
        ImpedanceParams default_params = {
            DEFAULT_K_STIFFNESS,
            DEFAULT_B_DAMPING,
            "Default (Invalid State)"
        };
        return default_params;
    }
    
    return IMP_LOOKUP_TABLE[state_code];
}

void calculateCartesianForce() {
    // F = K(x_d - x) + B(0 - x_dot)
    // Linear forces only (first 3 elements)
    for (int i = 0; i < 3; i++) {
        float pos_error = imp_state.desired_pos[i] - imp_state.current_pos[i];
        float vel_error = 0.0f - imp_state.current_vel[i]; // Desired velocity is 0
        
        imp_state.cartesian_force[i] = 
            imp_state.k_gain * pos_error + 
            imp_state.b_gain * vel_error;
    }
    
    // No torques for now (angular components)
    imp_state.cartesian_force[3] = 0.0f;
    imp_state.cartesian_force[4] = 0.0f;
    imp_state.cartesian_force[5] = 0.0f;
}

void applyDirectionalGains(float force[3], const Pose6D &pose) {
    // Get rotation matrix from end-effector orientation
    float R[3][3];
    poseToRotationMatrix(pose, R);
    
    // Transform force to end-effector frame
    float force_ee[3];
    for (int i = 0; i < 3; i++) {
        force_ee[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            force_ee[i] += R[j][i] * force[j]; // R^T * F
        }
    }
    
    // Apply directional gains in end-effector frame
    force_ee[0] *= K_MULTIPLIER_X; // Perpendicular to edge (blunt)
    force_ee[1] *= K_MULTIPLIER_Y; // Along edge (sharp)
    force_ee[2] *= K_MULTIPLIER_Z; // Perpendicular to flat
    
    // Transform back to global frame
    for (int i = 0; i < 3; i++) {
        force[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            force[i] += R[i][j] * force_ee[j]; // R * F_ee
        }
    }
}

void poseToRotationMatrix(const Pose6D &pose, float R[3][3]) {
    // ZYX Euler angle convention
    float cr = cosf(pose.roll);
    float sr = sinf(pose.roll);
    float cp = cosf(pose.pitch);
    float sp = sinf(pose.pitch);
    float cy = cosf(pose.yaw);
    float sy = sinf(pose.yaw);
    
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

void calculateJointTorques(const Jacobian &J) {
    // tau = J^T * F
    // Only use linear part of Jacobian (3x3 for now, will be 6x3 or 6x6 later) and force (3x1)
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        imp_state.joint_torques[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            // J.data is [6xN], we want J^T which is [Nx6]
            // We only use the linear velocity part (first 3 rows)
            imp_state.joint_torques[i] += J.data[j][i] * imp_state.cartesian_force[j];
        }
    }
}

void printImpedanceState() {
    Serial.println("=== Impedance Control State ===");
    Serial.print("Unity: "); 
    Serial.println(imp_state.unity_connected ? "CONNECTED" : "DISCONNECTED");
    Serial.print("K: "); Serial.print(imp_state.k_gain, 3);
    Serial.print(" B: "); Serial.println(imp_state.b_gain, 3);
    
    Serial.print("Pos error: [");
    for (int i = 0; i < 3; i++) {
        Serial.print(imp_state.desired_pos[i] - imp_state.current_pos[i], 4);
        if (i < 2) Serial.print(", ");
    }
    Serial.println("]");
    
    Serial.print("Force: [");
    for (int i = 0; i < 3; i++) {
        Serial.print(imp_state.cartesian_force[i], 4);
        if (i < 2) Serial.print(", ");
    }
    Serial.println("]");
    
    Serial.print("Torques: [");
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        Serial.print(imp_state.joint_torques[i], 4);
        if (i < NUMBER_OF_JOINTS - 1) Serial.print(", ");
    }
    Serial.println("]");
}