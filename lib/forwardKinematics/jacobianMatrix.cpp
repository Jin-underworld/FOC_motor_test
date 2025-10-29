#include "jacobianMatrix.h"
#include "../forwardKinematics/forwardKinematics.h"
#include "../sensorHelper/sensorHelper.h"
#include "../motorHelper/motorHelper.h"
#include <math.h>

// Unit vectors for each axis
const float Z_AXIS[3] = {0.0f, 0.0f, 1.0f};

// Extract rotation matrix (3x3) from transformation matrix (4x4)
void extractRotationMatrix(const float T[4][4], float R[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = T[i][j];
        }
    }
}

// Extract position vector from transformation matrix
void extractPosition(const float T[4][4], float p[3]) {
    p[0] = T[0][3]; // x
    p[1] = T[1][3]; // y
    p[2] = T[2][3]; // z
}

// Vector cross product
void crossProduct(const float a[3], const float b[3], float result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// Get joint angular velocities in rad/s
void getJointVelocities(float velocities[NUMBER_OF_JOINTS]) {
    velocities[0] = GB6025motor.shaftVelocity();
    velocities[1] = GB6010motor.shaftVelocity();
    velocities[2] = GB4315motor.shaftVelocity();
    // When you have 6 joints, add:
    // velocities[3] = GB3510motor.shaftVelocity();
    // velocities[4] = GB3510motor.shaftVelocity();
    // velocities[5] = JC2804motor.shaftVelocity();
}

// Calculate the Jacobian matrix
Jacobian calculateJacobian(void) {
    Jacobian ee;
    // Transformation matrices
    float T01[4][4], T12[4][4], T23[4][4];
    float T02[4][4], T03[4][4];
    
    // Get current joint angles
    float theta[3];
    for (int i = 0; i < 3; ++i) {
        theta[i] = getSensorAngleKinematics(i);
    }
    
    // Compute transformation matrices
    dhTransform(theta[0], ALPHA_DH[0], D_DH[0], R_DH[0], T01);
    dhTransform(theta[1], ALPHA_DH[1], D_DH[1], R_DH[1], T12);
    dhTransform(theta[2], ALPHA_DH[2], D_DH[2], R_DH[2], T23);
    
    // Calculate composite transforms
    matMul4x4(T01, T12, T02);
    matMul4x4(T02, T23, T03);
    
    // TG0 is the base transform
    // T03 is the final transform to end effector
    float TGE[4][4];
    matMul4x4(TG0, T03, TGE);
    
    // Extract rotation matrices
    float R01[3][3], R12[3][3], R23[3][3];
    extractRotationMatrix(T01, R01);
    extractRotationMatrix(T12, R12);
    extractRotationMatrix(T23, R23);
    
    // Calculate positions relative to base
    float pG[3] = {0.0f, 0.0f, 0.0f};  // Base position
    float pJ0[3], pJ1[3], pJ2[3];
    
    
    //Temp matrix to be used to calculate position and the z axes of rotation
    //float Ttemp0[4][4];
    float TG1[4][4];
    float TG2[4][4];
    
    // Compute Z axes for each joint in global frame
    float zG[3] = {0.0f, 0.0f, 1.0f};  // Z axis of base in global frame
    float zJ0[3], zJ1[3], zJ2[3];
    
    
    extractPosition(TG0, pJ0);
    // Z axes are the 3rd column of rotation matrices in global frame
    for (int i = 0; i < 3; i++) {
        zJ0[i] = TG0[i][2];  // Z axis of joint 0 in global frame
    }
    
    matMul4x4(TG0, T01, TG1);
    extractPosition(TG1, pJ1);
    for (int i = 0; i < 3; i++) {
        zJ1[i] = TG1[i][2];  // Z axis of joint 1 in global frame
    }
    
    matMul4x4(TG0, T02, TG2);
    extractPosition(TG2, pJ2);
    for (int i = 0; i < 3; i++) {
        zJ2[i] = TG2[i][2];  // Z axis of joint 2 in global frame
    }
    
    // // ADD THIS CODE before calculating r1e, r2e, r3e
    // // Calculate the actual end effector position including the offset
    // float T_03E[4][4];      // Transform from joint 0 to end effector
    // float T_G_EE[4][4];     // Global to end effector

    // // First combine T03 with T3E to get transform to actual end effector
    // matMul4x4(T03, T3E, T_03E);

    // // Then transform to global frame
    // matMul4x4(TG0, T_03E, T_G_EE);

    // Extract the true end effector position
    float p_Ge[3];
    extractPosition(TGE, p_Ge);

    // Vectors from joint to end effector
    float rJ0_e[3], rJ1_e[3], rJ2_e[3];
    
    // r1e = p3 - p1 (vector from joint 1 to end effector)
    for (int i = 0; i < 3; i++) {
        rJ0_e[i] = p_Ge[i] - pJ0[i];
        rJ1_e[i] = p_Ge[i] - pJ1[i];
        rJ2_e[i] = p_Ge[i] - pJ2[i]; 
    }
    
    // Calculate Jacobian columns
    // Linear velocity component: zi-1 × ri,e (cross product)
    float j0_linear[3], j1_linear[3], j2_linear[3];
    crossProduct(zJ0, rJ0_e, j0_linear);
    crossProduct(zJ1, rJ1_e, j1_linear);
    crossProduct(zJ2, rJ2_e, j2_linear);

    // Angular velocity component: simply the z axes
    float j0_angular[3], j1_angular[3], j2_angular[3];
    for (int i = 0; i < 3; i++) {
        j0_angular[i] = zJ0[i];
        j1_angular[i] = zJ1[i];
        j2_angular[i] = zJ2[i];
    }
    
    // Fill the Jacobian matrix [6×3]
    // Linear part (top 3 rows)
    for (int i = 0; i < 3; i++) {
        ee.data[i][0] = j0_linear[i];
        ee.data[i][1] = j1_linear[i];
        ee.data[i][2] = j2_linear[i];
    }
    
    // Angular part (bottom 3 rows)
    for (int i = 0; i < 3; i++) {
        ee.data[i+3][0] = j0_angular[i];
        ee.data[i+3][1] = j1_angular[i];
        ee.data[i+3][2] = j2_angular[i];
    }

    return ee;
}

void printJacobian(void) {
    Jacobian j = calculateJacobian();
    printJacobian(j);
}

// Print Jacobian matrix
void printJacobian(const Jacobian &jacobian) {
    // Linear part (first 3 rows)
    Serial.print("J_lin: [");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Serial.print(jacobian.data[i][j], 2);
            if (j < 2) Serial.print(", ");
        }
        if (i < 2) Serial.println(" | ");
    }
    Serial.println("]");
    
    // Angular part (last 3 rows)
    // Serial.print("J_ang: [");
    // for (int i = 3; i < 6; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         Serial.print(jacobian.data[i][j], 2);
    //         if (j < 2) Serial.print(", ");
    //     }
    //     if (i < 5) Serial.print(" | ");
    // }
    // Serial.println("]");
}



// Calculate end-effector velocity from joint velocities using the Jacobian
void calculateEndEffectorVelocity(const Jacobian &jacobian, float joint_velocities[NUMBER_OF_JOINTS], float ee_velocity[6]) {
    // Initialize ee_velocity to zero
    for (int i = 0; i < 6; i++) {
        ee_velocity[i] = 0.0f;
    }
    
    // Multiply Jacobian by joint velocities
    // Jacobian is [6 x NUMBER_OF_JOINTS]
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < NUMBER_OF_JOINTS; j++) {
            ee_velocity[i] += jacobian.data[i][j] * joint_velocities[j];
        }
    }
}

// Helper function for dot product
float dotProduct(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// Helper function for vector magnitude
float vectorMagnitude(const float vec[3]) {
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

// Helper function for matrix determinant (3x3)
float determinant3x3(float mat[3][3]) {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) -
           mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

void checkJacobian(void){
    Jacobian J = calculateJacobian();
    return checkJacobian(J);
}

void checkJacobian(const Jacobian &J) {
    Serial.println("=== JACOBIAN VALIDATION ===");
    
    
    // Get current joint angles for reference
    float theta[3];
    for (int i = 0; i < 3; ++i) {
        theta[i] = getSensorAngleKinematics(i); //in rad
    }
    
    Serial.print("Current joint angles: [");
    Serial.print(theta[0], 3); Serial.print(", ");
    Serial.print(theta[1], 3); Serial.print(", ");
    Serial.print(theta[2], 3); Serial.println("]");
    
    // === 1. ORTHOGONALITY TEST ===
    Serial.println("\n1. Orthogonality Test: z · J_v ≈ 0");
    
    // Extract z-axes (same calculation as in calculateJacobian)
    float T01[4][4], T12[4][4], T23[4][4], T02[4][4], T03[4][4], TGE[4][4];
    dhTransform(theta[0], ALPHA_DH[0], D_DH[0], R_DH[0], T01);
    dhTransform(theta[1], ALPHA_DH[1], D_DH[1], R_DH[1], T12);
    dhTransform(theta[2], ALPHA_DH[2], D_DH[2], R_DH[2], T23);
    matMul4x4(T01, T12, T02);
    matMul4x4(T02, T23, T03);
    matMul4x4(TG0, T03, TGE);

    float TG1[4][4], TG2[4][4];
    float zG[3] = {0.0f, 0.0f, 1.0f};
    float z0[3], z1[3], z2[3];
    
    
    for (int i = 0; i < 3; i++) z0[i] = TG0[i][2];
    
    matMul4x4(TG0, T01, TG1);
    for (int i = 0; i < 3; i++) z1[i] = TG1[i][2];
    
    matMul4x4(TG0, T02, TG2);
    for (int i = 0; i < 3; i++) z2[i] = TG2[i][2];
    
    // Extract linear velocity columns from Jacobian
    float j0_linear[3], j1_linear[3], j2_linear[3];
    for (int i = 0; i < 3; i++) {
        j0_linear[i] = J.data[i][0];
        j1_linear[i] = J.data[i][1];
        j2_linear[i] = J.data[i][2];
    }
    
    float dot1 = dotProduct(z0, j0_linear);
    float dot2 = dotProduct(z1, j1_linear);
    float dot3 = dotProduct(z2, j2_linear);
    
    Serial.print("z0 · j1_linear = "); Serial.print(dot1, 6);
    Serial.println((abs(dot1) < 0.001) ? " ✓ PASS" : " ✗ FAIL");
    Serial.print("z1 · j2_linear = "); Serial.print(dot2, 6);
    Serial.println((abs(dot2) < 0.001) ? " ✓ PASS" : " ✗ FAIL");
    Serial.print("z2 · j3_linear = "); Serial.print(dot3, 6);
    Serial.println((abs(dot3) < 0.001) ? " ✓ PASS" : " ✗ FAIL");
    
    // === 2. FINITE DIFFERENCE VALIDATION ===
    Serial.println("\n2. Finite-Difference Validation");

    // small perturbation in radians
    const float delta = 0.001f;   // ~0.057 degrees


    //Helper: compute EE position from theta[] via manual DH chain
    auto fk_position_from_theta = [&](const float th[3], float p_out[3]) {
        float T01[4][4], T12[4][4], T23[4][4], T02[4][4], T03[4][4],TGE[4][4];
        dhTransform(th[0], ALPHA_DH[0], D_DH[0], R_DH[0], T01);
        dhTransform(th[1], ALPHA_DH[1], D_DH[1], R_DH[1], T12);
        dhTransform(th[2], ALPHA_DH[2], D_DH[2], R_DH[2], T23);
        matMul4x4(T01, T12, T02);
        matMul4x4(T02, T23, T03);
        matMul4x4(TG0, T03, TGE);
        //for now 3 is endeffector, add on as joints come in
        extractPosition(TGE, p_out);
    };

    // Baseline EE position (unperturbed)
    float p_base[3];
    extractPosition(TGE, p_base);
    //fk_position_from_theta(theta, p_base);

    // Assume J is your analytical Jacobian already computed for 'theta'

    for (int joint = 0; joint < 3; ++joint) {
        Serial.print("Joint "); Serial.print(joint); Serial.print(": ");

        // Forward (one-sided) difference
        float theta_forward[3] = {theta[0], theta[1], theta[2]};
        theta_forward[joint] += delta;

        float p_forward[3];
        fk_position_from_theta(theta_forward, p_forward);

        float numerical_fd[3];
        numerical_fd[0] = (p_forward[0] - p_base[0]) / delta;
        numerical_fd[1] = (p_forward[1] - p_base[1]) / delta;
        numerical_fd[2] = (p_forward[2] - p_base[2]) / delta;

        // Optional: central difference (uncomment to use)
        /*
        float theta_backward[3] = {theta[0], theta[1], theta[2]};
        theta_backward[joint] -= delta;

        float p_backward[3];
        fk_position_from_theta(theta_backward, p_backward);

        float numerical_cd[3];
        numerical_cd[0] = (p_forward[0] - p_backward[0]) / (2.0f * delta);
        numerical_cd[1] = (p_forward[1] - p_backward[1]) / (2.0f * delta);
        numerical_cd[2] = (p_forward[2] - p_backward[2]) / (2.0f * delta);
        const float *numerical = numerical_cd;
        */

        // Use forward difference by default
        const float *numerical = numerical_fd;

        // Analytical Jacobian column (linear part)
        float analytical[3] = {
            J.data[0][joint],
            J.data[1][joint],
            J.data[2][joint]
        };

        // L1 error
        float error = 0.0f;
        for (int i = 0; i < 3; ++i) {
            error += fabsf(numerical[i] - analytical[i]);
        }

        Serial.print("Error = "); Serial.print(error, 6);
        Serial.println((error < 0.3f) ? " ✓ PASS" : " ✗ FAIL");
    }
    
    // === 3. RANK AND CONDITIONING ===
    Serial.println("\n3. Rank and Conditioning");
    
    // Extract translational Jacobian (3x3)
    float J_v[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            J_v[i][j] = J.data[i][j];
        }
    }
    
    float det = determinant3x3(J_v);
    Serial.print("det(J_v) = "); Serial.print(det, 6);
    
    if (abs(det) < 0.0001) {
        Serial.println(" ✗ SINGULAR - Robot at singularity!");
    } else if (abs(det) < 0.01) {
        Serial.println(" ⚠ NEAR-SINGULAR - Poor conditioning");
    } else {
        Serial.println(" ✓ WELL-CONDITIONED");
    }
    
    // // === 4. ZERO CONFIGURATION SANITY CHECK ===
    // Serial.println("\n4. Zero Configuration Check");
    
    // bool at_zero = true;
    // for (int i = 0; i < 3; i++) {
    //     if (abs(theta[i]) > 0.1) {  // 0.1 rad tolerance
    //         at_zero = false;
    //         break;
    //     }
    // }
    
    // if (at_zero) {
    //     Serial.println("Robot is at zero configuration - manual verification needed");
    //     Serial.print("j1_linear should align with z0 × r_base_to_ee");
        
    //     // Calculate expected j1_linear at zero config
    //     float base_to_ee[3];
    //     base_to_ee[0] = pose_current.x - 0.0f;
    //     base_to_ee[1] = pose_current.y - 0.0f;
    //     base_to_ee[2] = pose_current.z - GLOBAL_TO_JOINT0_LENGTH;
        
    //     float expected_j1[3];
    //     crossProduct(z0, base_to_ee, expected_j1);
        
    //     Serial.print("\nExpected j1_linear: [");
    //     Serial.print(expected_j1[0], 3); Serial.print(", ");
    //     Serial.print(expected_j1[1], 3); Serial.print(", ");
    //     Serial.print(expected_j1[2], 3); Serial.println("]");
        
    //     Serial.print("Actual j1_linear:   [");
    //     Serial.print(j1_linear[0], 3); Serial.print(", ");
    //     Serial.print(j1_linear[1], 3); Serial.print(", ");
    //     Serial.print(j1_linear[2], 3); Serial.println("]");
    // } else {
    //     Serial.println("Robot not at zero configuration - skipping manual check");
    // }
    
    Serial.println("=== VALIDATION COMPLETE ===\n");
}
