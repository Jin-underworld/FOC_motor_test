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

float MultiplexedMagneticSensorI2C::getAngle() {
    multiplexer->selectPort(port);
    float raw = MagneticSensorI2C::getSensorAngle(); // 0..2π

    uint32_t now = micros();
    if (prev_ts == 0 || isnan(prev_raw_mech)) {
        prev_ts = now;
        prev_raw_mech = raw;
        filt_unwrapped = raw;
        last_unwrapped = filt_unwrapped;
        vel_prev_ts = now;
        return filt_unwrapped;
    }

    float dt = (now == prev_ts) ? 1e-4f : (now - prev_ts) * 1e-6f;
    prev_ts = now;

    // Shortest diff (-π,π] across wrap
    float da = angleDiff(raw, prev_raw_mech);
    prev_raw_mech = raw;

    // 1) Reject impossible big jumps (>0.5 rotation)
    if (fabsf(da) > (_2PI * 0.5f)) {
        da = 0.0f;
    }

    // 2) Rate-cap the remaining delta
    float max_da = max_rate_rad_s * dt;
    if (fabsf(da) > max_da) da = copysignf(max_da, da);

    // Integrate in UNWRAPPED domain, then LPF in UNWRAPPED domain
    float unwrapped = filt_unwrapped + da;
    filt_unwrapped = angle_lpf(unwrapped);

    return filt_unwrapped; // continuous angle for FOC commutation
}

float MultiplexedMagneticSensorI2C::getVelocity() {
    uint32_t now = micros();
    float Ts = (now == vel_prev_ts) ? 1e-4f : (now - vel_prev_ts) * 1e-6f;
    if (Ts <= 0.0f) Ts = 1e-4f;
    vel_prev_ts = now;

    float a = getAngle(); // filtered continuous
    float v = (a - last_unwrapped) / Ts;

    // Clip to reasonable range
    if (v >  max_rate_rad_s) v =  max_rate_rad_s;
    if (v < -max_rate_rad_s) v = -max_rate_rad_s;

    last_unwrapped = a;
    return v;
}

// Override update method
void MultiplexedMagneticSensorI2C::update() {
    multiplexer->selectPort(port);
    MagneticSensorI2C::update();
}

// Initialize I2C multiplexer
void initializeI2CMultiplexer() {
    Wire2.setClock(50000);
    Wire2.begin();

    
    //Wire2.setTimeout(2);
    
    
    multiplexer_tran.begin();
    //multiplexer_wrist.begin();
    Serial2.println("I2C Multiplexer initialized");
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

    // Common tuning (per sensor)
    const float maxRate = 150.0f;  // rad/s (raise if you need more speed)
    const float Tf      = 0.002f;  // s

    sensor0.setMaxAngleRate(maxRate); sensor0.setAngleFilterTf(Tf);
    sensor1.setMaxAngleRate(maxRate); sensor1.setAngleFilterTf(Tf);
    sensor2.setMaxAngleRate(maxRate); sensor2.setAngleFilterTf(Tf);
    sensor3.setMaxAngleRate(maxRate); sensor3.setAngleFilterTf(Tf);
    sensor4.setMaxAngleRate(maxRate); sensor4.setAngleFilterTf(Tf);
    sensor5.setMaxAngleRate(maxRate); sensor5.setAngleFilterTf(Tf);
    sensor6.setMaxAngleRate(maxRate); sensor6.setAngleFilterTf(Tf);


    Serial2.println("All sensors initialized");
}

// Update all sensors
// void updateAllEncoders() {
//     // multiplexer_tran.selectPort(0);
//     // delayMicroseconds(200);
//     // sensor0.update();
    
//     multiplexer_tran.selectPort(1);
//     delayMicroseconds(200);
//     sensor1.update();
    
//     // multiplexer_tran.selectPort(2);
//     // delayMicroseconds(200);
//     // sensor2.update();
    
//     // sensorBarrier.update();
//     multiplexer_tran.selectPort(3);
//     delayMicroseconds(200);
//     sensor3.update();
    
//     multiplexer_tran.selectPort(4);
//     delayMicroseconds(200);
//     sensor4.update();
    
//     multiplexer_tran.selectPort(5);
//     delayMicroseconds(200);
//     sensor5.update();
    
//     multiplexer_tran.selectPort(6);
//     delayMicroseconds(200);
//     sensor6.update();
    
// }

void printAllEncoders() {
    updateAllEncoders();
    
    Serial2.print("[");
    Serial2.print(sensor0.getAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor1.getAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor2.getAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor3.getAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor4.getAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor5.getAngle(), 3);
    Serial2.print(",");
    Serial2.print(sensor6.getAngle(), 3);
    Serial2.println("]");
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


LowPassFilter LPFangle0 {0.01};
LowPassFilter LPFangle1 {0.01};
LowPassFilter LPFangle2 {0.01};
LowPassFilter LPFangle3 {0.01};
LowPassFilter LPFangle4 {0.01};
LowPassFilter LPFangle5 {0.01};
LowPassFilter LPFangle6 {0.01};

//for DH table, gets all sensor angles in RAD with zero offset without needing to link to a motor
float getAllMechAngles(int sensornumber) {
    float angle_raw = 0.0f;
    float angle_rad = 0.0f;

    switch (sensornumber) {
        case 0:
            sensor0.update();
            angle_raw = JOINT0_SENSOR_DIRECTION * sensor0.getMechanicalAngle() - JOINT0_ZERO_CALIBRATED_ANGLE;
            angle_rad = LPFangle0(angle_raw);
            break;
        case 1:
            sensor1.update();
            angle_raw = JOINT1_SENSOR_DIRECTION * sensor1.getMechanicalAngle() - JOINT1_ZERO_CALIBRATED_ANGLE;
            angle_rad = LPFangle1(angle_raw);
            break;
        case 2:
            sensor2.update();
            angle_raw = JOINT2_SENSOR_DIRECTION * sensor2.getMechanicalAngle() - JOINT2_ZERO_CALIBRATED_ANGLE;
            angle_rad = LPFangle2(angle_raw);
            break;
        case 3:
            sensor3.update();
            angle_raw = JOINT3_SENSOR_DIRECTION * sensor3.getMechanicalAngle() - JOINT3_ZERO_CALIBRATED_ANGLE;
            angle_rad = LPFangle3(angle_raw);
            break;
        case 4:
            sensor4.update();
            angle_raw = JOINT4_SENSOR_DIRECTION * sensor4.getMechanicalAngle() - JOINT4_ZERO_CALIBRATED_ANGLE;
            angle_rad = LPFangle4(angle_raw);
            break;
        case 5:
            sensor5.update();
            angle_raw = JOINT5_SENSOR_DIRECTION * sensor5.getMechanicalAngle() - JOINT5_ZERO_CALIBRATED_ANGLE;
            angle_rad = LPFangle5(angle_raw);
            break;
        case 6:
            sensor6.update();
            // Fixed direction macro to JOINT6_SENSOR_DIRECTION (was JOINT0_SENSOR_DIRECTION)
            angle_raw = JOINT6_SENSOR_DIRECTION * sensor6.getMechanicalAngle() - JOINT6_ZERO_CALIBRATED_ANGLE;
            angle_rad = LPFangle6(angle_raw);
            break;
        default:
            angle_rad = 0.0f;
            break;
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

// ...existing includes...

bool recoverI2CBus() {
    // 1) Release I2C peripheral
    Wire2.end();
    
    // 2) Configure pins as GPIO open-drain
    pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_SDA, INPUT_PULLUP);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(10);
    
    // 3) Check if SCL is stuck low (slave holding it)
    pinMode(PIN_SCL, INPUT_PULLUP);
    bool scl_ok = (digitalRead(PIN_SCL) == HIGH);
    pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_SCL, HIGH);
    
    if (!scl_ok) {
        //Serial22.println("I2C: SCL stuck low - hardware issue");
        // Continue anyway - might still recover
    }
    
    // 4) Clock SCL up to 9 times to let slave finish byte transmission
    for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; i++) {
        digitalWrite(PIN_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_SCL, HIGH);
        delayMicroseconds(5);
    }
    
    // 5) Send STOP condition: SDA low->high while SCL high
    pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_SDA, HIGH);
    delayMicroseconds(10);
    
    // 6) Release pins and restart I2C
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    
    // 7) Reinitialize I2C
    Wire2.begin();
    Wire2.setClock(50000); // Keep your current speed
    
    //Serial22.println("I2C bus recovery attempted");
    return true;
}

void restartI2CSystem() {
    //Serial22.println("Restarting I2C system...");
    
    // Recover the bus first
    recoverI2CBus();
    
    // Test if multiplexer is working by trying to select a port
    bool mux_ok = true;
    for (int port = 0; port < 8; port++) {
        Wire2.beginTransmission(0x70); // multiplexer address
        if (Wire2.endTransmission() != 0) {
            mux_ok = false;
            break;
        }
    }
    
    if (mux_ok) {
        //Serial22.println("Multiplexer communication OK");
    } else {
        //Serial22.println("Multiplexer communication failed");
    }
    
    // Reinitialize sensors that are active
    sensor0.init(&Wire2);
    sensor1.init(&Wire2);
    sensor2.init(&Wire2);
    sensor3.init(&Wire2);
    sensor4.init(&Wire2);
    sensor5.init(&Wire2);
    sensor6.init(&Wire2);
    
    //Serial22.println("I2C system restart complete");
}

// Add timeout-protected sensor update
bool safeUpdateSensor(MultiplexedMagneticSensorI2C& sensor, uint8_t port, uint32_t timeout_ms = 1) {
    multiplexer_tran.selectPort(port);
    delayMicroseconds(50);
    
    uint32_t start = millis();
    sensor.update();
    
    // Check if update took too long (indicates I2C timeout/hang)
    if (millis() - start > timeout_ms) {
        //Serial22.print("I2C timeout on port "); Serial22.println(port);
        return false;
    }
    return true;
}

// Modified update function with recovery
void updateAllEncoders() {
    static uint8_t error_count = 0;
    bool success = true;
    
    success &= safeUpdateSensor(sensor0, 0);
    success &= safeUpdateSensor(sensor1, 1);
    success &= safeUpdateSensor(sensor2, 2);
    success &= safeUpdateSensor(sensor3, 3); 
    success &= safeUpdateSensor(sensor4, 4);
    success &= safeUpdateSensor(sensor5, 5);
    success &= safeUpdateSensor(sensor6, 6);
    
    if (!success) {
        error_count++;
        if (error_count >= 3) {  // After 3 consecutive errors
            //Serial22.println("Multiple I2C errors - recovering bus");
            restartI2CSystem();
            error_count = 0;
        }
    } else {
        error_count = 0;  // Reset on success
    }
}
