#include "sensorHelper.h"

// I2C multiplexer initialization
TwoWire Wire2(PIN_SDA, PIN_SCL);
DFRobot_I2C_Multiplexer multiplexer_tran(&Wire2, 0x70);
//DFRobot_I2C_Multiplexer multiplexer_wrist(&Wire2, 0x77);

// Sensor instances definitions
MultiplexedMagneticSensorI2C sensor0(&multiplexer_tran, 0, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor1(&multiplexer_tran, 1, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor2(&multiplexer_tran, 2, AS5600_I2C);
// MultiplexedMagneticSensorI2C sensorBarrier(&multiplexer_tran, 3, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor3(&multiplexer_tran, 3, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor4(&multiplexer_tran, 4, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor5(&multiplexer_tran, 5, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor6(&multiplexer_tran, 6, AS5600_I2C);

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
    Wire2.setClock(5000);
    multiplexer_tran.begin();
    //multiplexer_wrist.begin();
    Serial.println("I2C Multiplexer initialized");
}

// Initialize all sensors
void initializeSensors() {
    sensor0.init(&Wire2);
    sensor1.init(&Wire2);
    sensor2.init(&Wire2);
    //sensorBarrier.init(&Wire2);
    sensor3.init(&Wire2);
    sensor4.init(&Wire2);
    sensor5.init(&Wire2);
    sensor6.init(&Wire2);

    Serial.println("All sensors initialized");
}

// Update all sensors
void updateAllEncoders() {
    sensor0.update();
    sensor1.update();
    sensor2.update();
    //sensorBarrier.update();
    sensor3.update();
    sensor4.update();
    sensor5.update();
    sensor6.update();
}

void printAllEncoders() {
    updateAllEncoders();
    
    Serial.print("[");
    Serial.print(sensor0.getAngle(), 3);
    Serial.print(",");
    Serial.print(sensor1.getAngle(), 3);
    Serial.print(",");
    Serial.print(sensor2.getAngle(), 3);
    Serial.print(",");
    Serial.print(sensor3.getAngle(), 3);
    Serial.print(",");
    Serial.print(sensor4.getAngle(), 3);
    Serial.print(",");
    Serial.print(sensor5.getAngle(), 3);
    Serial.print(",");
    Serial.print(sensor6.getAngle(), 3);
    Serial.println("]");
}

void printAllMechEncoders() {
    updateAllEncoders();
    
    Serial2.print("[");
    Serial2.print(sensor0.getMechanicalAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor1.getMechanicalAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor2.getMechanicalAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor3.getMechanicalAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor4.getMechanicalAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor5.getMechanicalAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor6.getMechanicalAngle(), 3);
    Serial2.println("]");
}


LowPassFilter LPFangle0 {0.1};

//for DH table, gets all sensor angles in RAD with zero offset without needing to link to a motor
float getAllMechAngles(int sensornumber){

    float angle_rad = 0.0f;
    switch (sensornumber) {
        case 0: 
            sensor0.update();
            // Get raw angle (0-2π) and apply direction and offset
            angle_rad = JOINT0_SENSOR_DIRECTION * sensor0.getMechanicalAngle() - JOINT0_ZERO_CALIBRATED_ANGLE;
            break;
        case 1: 
            sensor1.update();
            angle_rad = JOINT1_SENSOR_DIRECTION * sensor1.getMechanicalAngle() - JOINT1_ZERO_CALIBRATED_ANGLE;
            break;
        case 2: 
            sensor2.update();
            angle_rad = JOINT2_SENSOR_DIRECTION * sensor2.getMechanicalAngle() - JOINT2_ZERO_CALIBRATED_ANGLE;
            break;
        case 3:
            sensor3.update();
            angle_rad = JOINT3_SENSOR_DIRECTION * sensor3.getMechanicalAngle() - JOINT3_ZERO_CALIBRATED_ANGLE;
            break;
        case 4:
            sensor4.update();
            angle_rad = JOINT4_SENSOR_DIRECTION * sensor4.getMechanicalAngle() - JOINT4_ZERO_CALIBRATED_ANGLE;
            break;
        case 5:
            sensor5.update();
            angle_rad = JOINT5_SENSOR_DIRECTION * sensor5.getMechanicalAngle() - JOINT5_ZERO_CALIBRATED_ANGLE;
            break;
        case 6:
            sensor6.update();
            angle_rad = JOINT0_SENSOR_DIRECTION * sensor6.getMechanicalAngle() - JOINT6_ZERO_CALIBRATED_ANGLE;
            break;
        default: 
            angle_rad = 0.0f;
            // Serial2.println("Invalid joint number!");
    }

    return _normalizeAngle(angle_rad);
}




//print for checking, should be all zero on power on
void printAllMechAngles() {
    updateAllEncoders();
    
    Serial2.print("[");
    for (int i =0;i<NUMBER_OF_JOINTS;i++){
    Serial2.print(getAllMechAngles(i), 3);
    Serial2.print(",");
    }
    Serial2.println("]");
}
