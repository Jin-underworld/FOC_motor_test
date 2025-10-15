#include "forwardKinematics.h"
#include "sensorHelper.h"

#include <math.h>
#include <stdio.h>

float getSensorAngleKinematics(int joint) {
    // Joint readings are in radians, convert to degrees
    float angle_rad = 0.0f;
    
    switch (joint) {
        case 0: angle_rad = sensor0.getSensorAngle(); break;
        case 1: angle_rad = sensor1.getSensorAngle(); break;
        case 2: angle_rad = sensor2.getSensorAngle(); break;
        default: angle_rad = 0.0f;
    }
    
    // Convert to degrees and add the configured offset
    return (RAD2DEG(angle_rad)) + THETA_OFFSET[joint];
}

void matMul4x4(const float A[4][4], const float B[4][4], float R[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            R[i][j] = 0;
            for (int k = 0; k < 4; ++k)
                R[i][j] += A[i][k] * B[k][j];
        }
}

void dhTransform(float theta, float alpha, float d, float r, float T[4][4]) {
    float ct = cosf(DEG2RAD(theta));
    float st = sinf(DEG2RAD(theta));
    float ca = cosf(DEG2RAD(alpha));
    float sa = sinf(DEG2RAD(alpha));

    T[0][0] = ct;    T[0][1] = -st * ca;  T[0][2] = st * sa;   T[0][3] = r * ct;
    T[1][0] = st;    T[1][1] = ct * ca;   T[1][2] = -ct * sa;  T[1][3] = r * st;
    T[2][0] = 0;     T[2][1] = sa;        T[2][2] = ca;        T[2][3] = d;
    T[3][0] = 0;     T[3][1] = 0;         T[3][2] = 0;         T[3][3] = 1;
}

Pose6D forwardKinematics(void) {
    float T01[4][4], T12[4][4], T23[4][4];

    float theta[3];
    for (int i = 0; i < 3; ++i)
        theta[i] = getSensorAngleKinematics(i);

    // Compute each DH transform using parameters from kinematicsConfig.h, the offset is added
    dhTransform(theta[0], ALPHA_DH[0], D_DH[0], R_DH[0], T01);
    dhTransform(theta[1], ALPHA_DH[1], D_DH[1], R_DH[1], T12);
    dhTransform(theta[2], ALPHA_DH[2], D_DH[2], R_DH[2], T23);

    // Combine all transforms: T_GE = TG0 * T01 * T12 * T23 * T3E
    float T02[4][4], T03[4][4], Ttemp[4][4], TGE[4][4];
    matMul4x4(T01, T12, T02);
    matMul4x4(T02, T23, T03);
    matMul4x4(T03, T3E, Ttemp);
    matMul4x4(TG0, Ttemp, TGE);

    Pose6D pose;
    pose.x = TGE[0][3];
    pose.y = TGE[1][3];
    pose.z = TGE[2][3];

    // Extract Euler angles (ZYX convention)
    pose.pitch = atan2f(-TGE[2][0], sqrtf(TGE[0][0]*TGE[0][0] + TGE[1][0]*TGE[1][0]));
    pose.yaw   = atan2f(TGE[1][0], TGE[0][0]);
    pose.roll  = atan2f(TGE[2][1], TGE[2][2]);

    return pose;
}
void printPose(void) {
    Pose6D ee = forwardKinematics();
    printPose(ee);
}

void printPose(const Pose6D &pose) {
    printf("End Effector Pose:\n");
    printf("Position: X: %.2f mm, Y: %.2f mm, Z: %.2f mm\n", 
           pose.x, pose.y, pose.z);
    printf("Orientation: Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", 
           RAD2DEG(pose.roll), RAD2DEG(pose.pitch), RAD2DEG(pose.yaw));
}
