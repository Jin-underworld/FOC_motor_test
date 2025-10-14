#include "sensorHelper.h"

// I2C multiplexer initialization
TwoWire Wire2(PB11, PB10);
DFRobot_I2C_Multiplexer multiplexer(&Wire2, 0x70);

// Sensor instances definitions
MultiplexedMagneticSensorI2C sensor0(&multiplexer, 0, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor1(&multiplexer, 1, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor2(&multiplexer, 2, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor3(&multiplexer, 3, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor4(&multiplexer, 4, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor5(&multiplexer, 5, AS5600_I2C);

// Constructor implementation - fix the parameter names
MultiplexedMagneticSensorI2C::MultiplexedMagneticSensorI2C(DFRobot_I2C_Multiplexer *_multiplexer, uint8_t _port, MagneticSensorI2CConfig_s _config)
    : MagneticSensorI2C(_config), multiplexer(_multiplexer), port(_port) {
}

// Override getSensorAngle method
float MultiplexedMagneticSensorI2C::getSensorAngle() {
    multiplexer->selectPort(port);
    return MagneticSensorI2C::getSensorAngle();
}

// Override update method
void MultiplexedMagneticSensorI2C::update() {
    multiplexer->selectPort(port);
    MagneticSensorI2C::update();
}

// Initialize I2C multiplexer
void initializeI2CMultiplexer() {
    Wire2.begin();
    multiplexer.begin();
    Serial.println("I2C Multiplexer initialized");
}

// Initialize all sensors
void initializeSensors() {
    sensor0.init(&Wire2);
    sensor1.init(&Wire2);
    sensor2.init(&Wire2);
    sensor3.init(&Wire2);
    sensor4.init(&Wire2);
    sensor5.init(&Wire2);

    Serial.println("All sensors initialized");
}