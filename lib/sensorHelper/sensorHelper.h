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
    
    // Override update method
    void update() override;
    
private:
    DFRobot_I2C_Multiplexer *multiplexer;
    uint8_t port;
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