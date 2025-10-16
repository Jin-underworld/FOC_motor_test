#include "forwardKinematics.h"
#include "sensorHelper.h"
#include "motorHelper.h"

#include <math.h>
#include <stdio.h>
//verified v1
float getSensorAngleKinematics(int joint) {
    // Joint readings are in radians, convert to degrees
    float angle_rad = 0.0f;

    switch (joint) {
        case 0: 
            sensor0.update();
            angle_rad = GB6025motor.shaftAngle(); 
            // Serial.print("angle0: "); Serial.print(angle_rad, 3);
            break;
        case 1: 
            sensor1.update();
            angle_rad = GB6010motor.shaftAngle(); 
            //Serial.print(" angle1: "); Serial.print(angle_rad, 3);
            break;
        case 2: 
            sensor2.update();
            angle_rad = GB4315motor.shaftAngle(); 
            //Serial.print(" angle2: "); Serial.println(angle_rad, 3);
            break;
        default: 
            angle_rad = 0.0f;
            // Serial.println("Invalid joint number!");
    }
    
    // Convert to degrees and add the configured offset
    float result = RAD2DEG(angle_rad) + THETA_OFFSET[joint];
    // Serial.print("Joint ");
    // Serial.print(joint);
    // Serial.print(" final angle (deg): ");
    // Serial.println(result, 2);
    return result;
}
//shoiuldnt be wrong v1
void matMul4x4(const float A[4][4], const float B[4][4], float R[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            R[i][j] = 0;
            for (int k = 0; k < 4; ++k)
                R[i][j] += A[i][k] * B[k][j];
        }
}
//verfied v1
void dhTransform(float theta, float alpha, float d, float r, float T[4][4]) {
    float ct = cosf(DEG2RAD(theta));
    float st = sinf(DEG2RAD(theta));
    float ca = cosf(DEG2RAD(alpha));
    float sa = sinf(DEG2RAD(alpha));

    T[0][0] = ct;    T[0][1] = -st * ca;  T[0][2] = st * sa;   T[0][3] = r * ct;
    T[1][0]  = st;    T[1][1] = ct * ca;   T[1][2] = -ct * sa;  T[1][3] = r * st;
    T[2][0] = 0;     T[2][1] = sa;        T[2][2] = ca;        T[2][3] = d;
    T[3][0] = 0;     T[3][1] = 0;         T[3][2] = 0;         T[3][3] = 1;
}

Pose6D forwardKinematics(void) {
    float T01[4][4], T12[4][4], T23[4][4];
    
    float temptheta[3];
    // Serial.println("--- Reading Joint Angles ---");
    for (int i = 0; i < 3; ++i) {
        temptheta[i] = getSensorAngleKinematics(i);
    }

    // Compute each DH transform using parameters from kinematicsConfig.h, the offset is added
    dhTransform(temptheta[0], ALPHA_DH[0], D_DH[0], R_DH[0], T01);
    dhTransform(temptheta[1], ALPHA_DH[1], D_DH[1], R_DH[1], T12);
    dhTransform(temptheta[2], ALPHA_DH[2], D_DH[2], R_DH[2], T23);

    // Combine all transforms: T_GE = TG0 * T01 * T12 * T23 * T3E
    float T02[4][4], T03[4][4], Ttemp[4][4], TGE[4][4];
    matMul4x4(T01, T12, T02);
    matMul4x4(T02, T23, T03);
    // matMul4x4(T03, T3E, Ttemp);
    matMul4x4(TG0, T03, TGE);

    // printMatrix(T01, "T01");
    // printMatrix(T12, "T12");
    // printMatrix(T23, "T23");
    // printMatrix(TGE, "Final Transform TGE");

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

// void lastknownpos (const Pose6D &pose){
//     vector 
//     float lastknownx = pose.x;
//     float lastknowny =pose.y;
//     float lastknownx =pose.z;
//     return 0
// }

void printPose(const Pose6D &pose) {
    
    // Position
    Serial.print("["); Serial.print(pose.x, 2);  Serial.print(", ");
    Serial.print(pose.y, 2); Serial.print(", ");
    Serial.print(pose.z, 2); Serial.print(", ");
    Serial.print(0);  Serial.print(", "); 
    Serial.print(0); Serial.print(", ");
    Serial.print(0); Serial.print(", ");
    Serial.print(0); Serial.println("]");
    // Serial.print("X: "); Serial.print(pose.x, 2);
    // Serial.print("  Y: "); Serial.print(pose.y, 2);
    // Serial.print("  Z: "); Serial.println(pose.z, 2);
    
    // // Orientation
    // Serial.print("Orientation: Roll: ");
    // Serial.print(RAD2DEG(pose.roll), 2);
    // Serial.print("°, Pitch: ");
    // Serial.print(RAD2DEG(pose.pitch), 2);
    // Serial.print("°, Yaw: ");
    // Serial.print(RAD2DEG(pose.yaw), 2);
    // Serial.println("°");
}

void printMatrix(const float mat[4][4], const char* name) {
    Serial.print(name);
    Serial.println(":");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Serial.print(mat[i][j], 4);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}