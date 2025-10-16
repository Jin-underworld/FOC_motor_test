#pragma once

// Motor configuration 6025
#define motor_GB6025_PP 14
#define motor_GB6025_R 3.35
#define motor_GB6025_KV 20
#define motor_GB6025_inductance 3.73e-3
#define motor_GB6025_ZEAngle 4.27f//tune
#define motor_GB6025_sensor_direction Direction::CCW //tune
#define motor_GB6025_ZeroCalibratedAngle -5.82f //IMPT!!!! code minus this away. so for angle use angle as is when at 0 position. PI adjustment is to avoid negative angles
#define motor_GB6025_HomeAngle 0.0f  // Define the home position angle (in radians)
#define motor_GB6025_UPPER_LIMIT 0.785f
#define motor_GB6025_LOWER_LIMIT -0.785f

// // Motor configuration 6010
#define motor_GB6010_PP 14
#define motor_GB6010_R 10.28
#define motor_GB6010_KV 16
#define motor_GB6010_inductance 12.35e-3
#define motor_GB6010_ZEAngle 6.11f
#define motor_GB6010_sensor_direction Direction::CCW
#define motor_GB6010_ZeroCalibratedAngle -2.94f //+ M_PI/3 // 60 degree initialIMPT!!!! code minus this away. so for angle use angle as is when at 0 position. PI adjustment is to avoid negative angles
#define motor_GB6010_HomeAngle M_PI/3  // Define the home position angle (in radians)
#define motor_GB6010_UPPER_LIMIT 1.601f - 0.001f
#define motor_GB6010_LOWER_LIMIT -(0.007f - 0.001f) 

// Motor configuration 4315
#define motor_GB4315_PP 14
#define motor_GB4315_R 13.84
#define motor_GB4315_KV 28
#define motor_GB4315_inductance 6.45e-3
#define motor_GB4315_ZEAngle 4.02f
#define motor_GB4315_sensor_direction Direction::CCW
#define motor_GB4315_ZeroCalibratedAngle -4.331f + 2.134f -2*M_PI//+ 2.335f
#define motor_GB4315_HomeAngle 0.75*M_PI  // Define the home position angle (in radians)
#define motor_GB4315_UPPER_LIMIT 2.628f - 0.001f
#define motor_GB4315_LOWER_LIMIT 0.005f - 0.001f
