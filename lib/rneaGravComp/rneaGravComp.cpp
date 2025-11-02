#include "rneaGravComp.hpp"
#include "motorHelper.h"
#include "forwardKinematics.h"

// Global instance
RNEAGravComp gravComp;

RNEAGravComp::RNEAGravComp() 
    : friction_comp_enabled(false), initialized(false) {
    for(int i = 0; i < NUMBER_OF_JOINTS; i++) {
        q[i] = 0.0f;
        qd[i] = 0.0f;
        tau_grav[i] = 0.0f;
        tau_friction[i] = 0.0f;
        tau_total[i] = 0.0f;
    }
}

void RNEAGravComp::init() {
    Serial2.println("Initializing RNEA Gravity Compensation...");
    
    // Verify DH parameters are loaded
    if(NUMBER_OF_JOINTS < 6) {
        Serial2.println("ERROR: Invalid NUMBER_OF_JOINTS in kinematicsConfig.h");
        return;
    }
    
    initialized = true;
    Serial2.println("RNEA Gravity Compensation initialized");
    Serial2.println("Active joints for grav comp: J1, J2 (vertical plane only)");
}

void RNEAGravComp::setJointPositions(const float* q_new) {
    for(int i = 0; i < NUMBER_OF_JOINTS; i++) {
        q[i] = q_new[i];
    }
}

void RNEAGravComp::setJointVelocities(const float* qd_new) {
    for(int i = 0; i < NUMBER_OF_JOINTS; i++) {
        qd[i] = qd_new[i];
    }
}

Mat3 RNEAGravComp::computeRotationMatrix(int joint_idx) {
    // Build DH transform of (i-1)->i using existing FK function in FK.h
    float T_i[4][4];
    dhTransform(
        q[joint_idx] + THETA_OFFSET[joint_idx+1], // note: DH row indexing if row0 is Global
        ALPHA_DH[joint_idx+1],
        D_DH[joint_idx+1],
        R_DH[joint_idx+1],
        T_i
    );
    // Rotation parent->current is top-left 3x3 of T_i
    // Our forward pass uses R_T = R.transpose() to rotate parent->current,
    // so store R as current->parent = (parent->current)^T
    Mat3 R;
    R.m[0][0] = T_i[0][0]; R.m[0][1] = T_i[1][0]; R.m[0][2] = T_i[2][0];
    R.m[1][0] = T_i[0][1]; R.m[1][1] = T_i[1][1]; R.m[1][2] = T_i[2][1];
    R.m[2][0] = T_i[0][2]; R.m[2][1] = T_i[1][2]; R.m[2][2] = T_i[2][2];
    return R;
}

Vec3 RNEAGravComp::computeComPosition(int link_idx) {
    // COM position in link frame (from URDF)
    return Vec3(LINK_COM[link_idx][0], 
                LINK_COM[link_idx][1], 
                LINK_COM[link_idx][2]);
}

Mat3 RNEAGravComp::getInertiaMatrix(int link_idx) {
    // Construct 3x3 inertia matrix from 6 parameters
    Mat3 I;
    I.m[0][0] = LINK_INERTIA[link_idx][0];  // Ixx
    I.m[1][1] = LINK_INERTIA[link_idx][1];  // Iyy
    I.m[2][2] = LINK_INERTIA[link_idx][2];  // Izz
    I.m[0][1] = I.m[1][0] = LINK_INERTIA[link_idx][3];  // Ixy
    I.m[0][2] = I.m[2][0] = LINK_INERTIA[link_idx][4];  // Ixz
    I.m[1][2] = I.m[2][1] = LINK_INERTIA[link_idx][5];  // Iyz
    
    return I;
}

void RNEAGravComp::forwardRecursion() {
    // Base link (world)
    link_states[0].omega = Vec3(0, 0, 0);
    link_states[0].omega_dot = Vec3(0, 0, 0);
    link_states[0].v = Vec3(0, 0, 0);
    link_states[0].v_dot = Vec3(0, 0, -GRAVITY);  // gravity in base

    for(int i = 0; i < NUMBER_OF_JOINTS; i++) {
        int link_idx = i + 1;

        Mat3 R = computeRotationMatrix(i);
        link_states[link_idx].R = R;
        Mat3 R_T = R.transpose();

        // Revolute joint axis is z in the joint frame 
        const Vec3 z(0.0f, 0.0f, 1.0f);

        Vec3 omega_prev_rot = R_T * link_states[i].omega;
        link_states[link_idx].omega = omega_prev_rot + z * qd[i];

        Vec3 omega_dot_prev_rot = R_T * link_states[i].omega_dot;
        Vec3 cross_term = omega_prev_rot.cross(z * qd[i]);
        link_states[link_idx].omega_dot = omega_dot_prev_rot + cross_term /*+ z*qdd[i]*/; // qdd=0 for grav-only

        // DH translation p_i expressed in parent frame (x = a, z = d)
        Vec3 p_i_parent(R_DH[i+1], 0.0f, D_DH[i+1]);

        // Linear velocity/acc expressed in current frame
        Vec3 v_prev = link_states[i].v;
        Vec3 cross_v = link_states[i].omega.cross(p_i_parent);
        link_states[link_idx].v = R_T * (v_prev + cross_v);

        Vec3 v_dot_prev = link_states[i].v_dot;
        Vec3 cross_a1 = link_states[i].omega_dot.cross(p_i_parent);
        Vec3 cross_a2 = link_states[i].omega.cross(link_states[i].omega.cross(p_i_parent));
        link_states[link_idx].v_dot = R_T * (v_dot_prev + cross_a1 + cross_a2);

        // COM terms
        Vec3 p_com = computeComPosition(link_idx);
        link_states[link_idx].v_com = link_states[link_idx].v +
                                      link_states[link_idx].omega.cross(p_com);
        link_states[link_idx].v_com_dot = link_states[link_idx].v_dot +
                                          link_states[link_idx].omega_dot.cross(p_com) +
                                          link_states[link_idx].omega.cross(
                                              link_states[link_idx].omega.cross(p_com));
    }
}

void RNEAGravComp::backwardRecursion() {
    link_states[NUM_LINKS - 1].force = Vec3(0, 0, 0);
    link_states[NUM_LINKS - 1].torque = Vec3(0, 0, 0);

    for(int i = NUMBER_OF_JOINTS - 1; i >= 0; i--) {
        int link_idx = i + 1;

        float m = LINK_MASS[link_idx];
        Mat3 I = getInertiaMatrix(link_idx);
        Vec3 p_com = computeComPosition(link_idx);

        // Gravity-only: F = m * a_com (a_com already includes gravity via base v_dot)
        link_states[link_idx].force = link_states[link_idx].v_com_dot * m;

        // N = I*alpha + omega x (I*omega) 
        Vec3 I_omega = I * link_states[link_idx].omega;
        Vec3 I_alpha = I * link_states[link_idx].omega_dot;
        link_states[link_idx].torque = I_alpha + link_states[link_idx].omega.cross(I_omega);
        link_states[link_idx].torque = link_states[link_idx].torque + p_com.cross(link_states[link_idx].force);
        
        // Add child spatial forces
       if(i < NUMBER_OF_JOINTS - 1) {
            int child_idx = link_idx + 1;
            Mat3 R_child = link_states[child_idx].R; // rotation child->current
            Vec3 p_child(R_DH[i+2], 0.0f, D_DH[i+2]); // parent->child translation in parent frame

            Vec3 F_child = R_child * link_states[child_idx].force;
            Vec3 N_child = R_child * link_states[child_idx].torque + p_child.cross(F_child);

            link_states[link_idx].force = link_states[link_idx].force + F_child;
            link_states[link_idx].torque = link_states[link_idx].torque + N_child;
        }

        // Project torque on the configured joint axis if actuated; else +Z
        Vec3 axis = (i < NUM_ACTUATED_JOINTS)
            ? Vec3(JOINT_AXIS[i][0], JOINT_AXIS[i][1], JOINT_AXIS[i][2])
            : Vec3(0.0f, 0.0f, 1.0f);
        tau_grav[i] = link_states[link_idx].torque.dot(axis);
    }
}

float RNEAGravComp::computeFrictionTorque(int joint_idx, float velocity) {
    const FrictionParams& fp = JOINT_FRICTION[joint_idx];
    
    float tau_f = 0.0f;
    
    // Coulomb friction
    if(velocity > 0.01f) {
        tau_f += fp.coulomb_pos;
    } else if(velocity < -0.01f) {
        tau_f += fp.coulomb_neg;
    } else {
        // Stiction region - use stiction value
        tau_f += fp.stiction * (velocity / 0.01f);  // Linear interpolation
    }
    
    // Viscous friction
    tau_f += fp.viscous * velocity;
    
    return tau_f;
}

void RNEAGravComp::computeFrictionTorques() {
    if(!friction_comp_enabled) {
        for(int i = 0; i < NUMBER_OF_JOINTS; i++) {
            tau_friction[i] = 0.0f;
        }
        return;
    }
    // Only defined for actuated joints
    for(int i = 0; i < NUM_ACTUATED_JOINTS; i++) {
        tau_friction[i] = GCOMP_ENABLE[i] ? computeFrictionTorque(i, qd[i]) : 0.0f;
    }
    // Non-actuated/non-existent joints -> zero
    for(int i = NUM_ACTUATED_JOINTS; i < NUMBER_OF_JOINTS; i++) {
        tau_friction[i] = 0.0f;
    }
}

float* RNEAGravComp::loopGComp() {
    if(!initialized) {
        Serial2.println("WARNING: RNEA not initialized!");
        return tau_total;
    }
    
    // Update joint positions from motors
    float q_current[NUMBER_OF_JOINTS] = {0};
    float qd_current[NUMBER_OF_JOINTS] = {0};
    
    q_current[0] = GB6025motor.shaftAngle();
    q_current[1] = GB6010motor.shaftAngle();
    q_current[2] = GB4315motor.shaftAngle();
    q_current[3] = GB3510_3_motor.shaftAngle();
    //q_current[4] = GB3510_4_motor.shaftAngle();

    // Add remaining joints if implemented
    for(int i = 4; i < NUM_ACTUATED_JOINTS; i++) {
        q_current[i] = 0.0f;  // Placeholder
    }
    
    qd_current[0] = GB6025motor.shaftVelocity();
    qd_current[1] = GB6010motor.shaftVelocity();
    qd_current[2] = GB4315motor.shaftVelocity();
    qd_current[3] = GB3510_3_motor.shaftVelocity();
    //q_current[4] = GB3510_4_motor.shaftVelocity();

    for(int i = 5; i < NUM_ACTUATED_JOINTS; i++) {
        qd_current[i] = 0.0f;  // Placeholder
    }
    
    setJointPositions(q_current);
    setJointVelocities(qd_current);
    
    forwardRecursion();
    backwardRecursion();
    
    // Compute friction compensation
    computeFrictionTorques();
    
    // Combine torques, gate output to enabled (e.g., only J1 and J2)
    for(int i = 0; i < NUM_ACTUATED_JOINTS; i++) {
        float sum = tau_grav[i] + tau_friction[i];
        tau_total[i] = GCOMP_ENABLE[i] ? sum : 0.0f;  // J1,J2 enabled per config
    }
    for(int i = NUM_ACTUATED_JOINTS; i < NUMBER_OF_JOINTS; i++) {
        tau_total[i] = 0.0f;
    }
    return tau_total;
}

void RNEAGravComp::printTorques() {
    Serial2.println("\n=== RNEA Gravity Compensation Torques ===");
    for(int i = 0; i < NUM_ACTUATED_JOINTS; i++) {
        Serial2.print("Joint ");
        Serial2.print(i);
        Serial2.print(": τ_grav=");
        Serial2.print(tau_grav[i], 4);
        Serial2.print(" N*m, τ_fric=");
        Serial2.print(tau_friction[i], 4);
        Serial2.print(" N*m, τ_total=");
        Serial2.print(tau_total[i], 4);
        Serial2.println(" N*m");
    }
}