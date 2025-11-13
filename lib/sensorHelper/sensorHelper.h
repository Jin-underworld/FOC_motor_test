#ifndef SENSOR_HELPER_H
#define SENSOR_HELPER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "DFRobot_I2C_Multiplexer.h"
#include "../../include/main.h"

// I2C configuration
extern TwoWire Wire2;
extern DFRobot_I2C_Multiplexer multiplexer_tran;
//extern DFRobot_I2C_Multiplexer multiplexer_wrist;

// Custom class that wraps MagneticSensorI2C to work with I2C multiplexer
class MultiplexedMagneticSensorI2C : public MagneticSensorI2C {
public:
    MultiplexedMagneticSensorI2C(DFRobot_I2C_Multiplexer *_multiplexer,
                                 uint8_t _port, MagneticSensorI2CConfig_s _config);
    
    // Override the getSensorAngle method to select the correct port before reading
    float getSensorAngle() override;


    // Provide filtered, continuous (unwrapped) angle and velocity to the motor
    float getAngle() override;
    float getVelocity() override;

    // Tuning setters (per-sensor)
    void setMaxAngleRate(float rad_per_s) { max_rate_rad_s = rad_per_s; }
    void setAngleFilterTf(float Tf) { angle_lpf.Tf = Tf; }

    // Ensure update still selects correct port
    void update() override;
    
private:
    DFRobot_I2C_Multiplexer *multiplexer;
    uint8_t port;

    // Glitch filter state (unwrapped domain)
    float max_rate_rad_s = 150.0f;     // cap real slew [rad/s] (tune)
    LowPassFilter angle_lpf{0.002f};   // LPF on unwrapped angle
    float prev_raw_mech = NAN;         // last raw 0..2π
    float filt_unwrapped = 0.0f;       // filtered continuous angle
    float last_unwrapped = 0.0f;       // last filtered angle for velocity
    uint32_t prev_ts = 0;              // us
    uint32_t vel_prev_ts = 0;          // us

    // helpers
    static inline float wrap2pi(float a) {
        a = fmodf(a, _2PI);
        if (a < 0.0f) a += _2PI;
        return a;
    }
    static inline float angleDiff(float a, float b) {
        float d = a - b;
        while (d >  M_PI) d -= _2PI;
        while (d < -M_PI) d += _2PI;
        return d;
    }
    };

// Sensor instances declarations
extern MultiplexedMagneticSensorI2C sensor0;
extern MultiplexedMagneticSensorI2C sensor1;
extern MultiplexedMagneticSensorI2C sensor2;
extern MultiplexedMagneticSensorI2C sensor3;
extern MultiplexedMagneticSensorI2C sensor4;
extern MultiplexedMagneticSensorI2C sensor5;
extern MultiplexedMagneticSensorI2C sensor6;

// Function declarations
void initializeI2CMultiplexer();
void initializeSensors();
void updateAllEncoders();
void printAllEncoders();
void printAllMechEncoders();
float getAllJointAngles(int sensornumber);
void printAllJointAngles();
float getAllMechAngles(int sensornumber);
void printAllMechAngles();

bool recoverI2CBus();
void restartI2CSystem();
bool safeUpdateSensor(MultiplexedMagneticSensorI2C& sensor, uint8_t port, uint32_t timeout_ms);


#endif // SENSOR_HELPER_H