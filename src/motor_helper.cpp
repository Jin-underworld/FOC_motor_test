#include "motor_helper.h"
#include "sensor_helper.h"
# include "main.h"

// Motor and driver definitions
BLDCMotor GB6010motor = BLDCMotor(motor_GB6010_PP, motor_GB6010_R, motor_GB6010_KV, motor_GB6010_inductance);
BLDCDriver3PWM GB6010driver = BLDCDriver3PWM(motor_GB6010_PWM_A, motor_GB6010_PWM_B, motor_GB6010_PWM_C);
BLDCMotor GB4315motor = BLDCMotor(motor_GB4315_PP, motor_GB4315_R, motor_GB4315_KV, motor_GB4315_inductance);
BLDCDriver3PWM GB4315driver = BLDCDriver3PWM(motor_GB4315_PWM_A, motor_GB4315_PWM_B, motor_GB4315_PWM_C);

// Target angle variables
float target_Angle_1 = motor_GB6010_HomeAngle;
float target_Angle_2 = motor_GB4315_HomeAngle;

// Motor configurations
MotorConfig gb6010_config = {
  &GB6010motor, 
  &sensor2, 
  motor_GB6010_UPPER_LIMIT, 
  motor_GB6010_LOWER_LIMIT, 
  motor_GB6010_HomeAngle,
  "GB6010"
};

MotorConfig gb4315_config = {
  &GB4315motor, 
  &sensor3, 
  motor_GB4315_UPPER_LIMIT, 
  motor_GB4315_LOWER_LIMIT, 
  motor_GB4315_HomeAngle,
  "GB4315"
};

// Initialize motor drivers
void initializeMotorDrivers() {
  // Initialize GB4315 driver
  GB4315driver.voltage_power_supply = driver_voltage_supply;
  GB4315driver.init();
  GB4315driver.pwm_frequency = 30000;
  
  // Initialize GB6010 driver
  GB6010driver.voltage_power_supply = driver_voltage_supply;
  GB6010driver.init();
  GB6010driver.pwm_frequency = 30000;
  
  Serial.println("Motor drivers initialized");
}

// Link motors to drivers and set control modes
void initializeMotorControllers() {
  // Link drivers to motors
  GB4315motor.linkDriver(&GB4315driver);
  GB4315motor.controller = MotionControlType::angle;
  GB4315motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  GB6010motor.linkDriver(&GB6010driver);
  GB6010motor.controller = MotionControlType::angle;
  GB6010motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  Serial.println("Motor controllers configured");
}

// Set motor control parameters
void initializeMotorParameters() {
  // GB6010 motor parameters
  GB6010motor.P_angle.P = 4.0f;
  GB6010motor.P_angle.I = 0.0f;
  GB6010motor.P_angle.D = 5.2f;
  GB6010motor.P_angle.output_ramp = 0.2f;
  GB6010motor.LPF_angle.Tf = 0.12f;
  GB6010motor.velocity_limit = 20.0f;
  
  // GB4315 motor parameters
  GB4315motor.P_angle.P = 4.0f;
  GB4315motor.P_angle.I = 0.0f;
  GB4315motor.P_angle.D = 5.2f;
  GB4315motor.P_angle.output_ramp = 0.2f;
  GB4315motor.LPF_angle.Tf = 0.12f;
  GB4315motor.velocity_limit = 20.0f;
  
  Serial.println("Motor parameters set");
}

// Link motors to sensors
void linkMotorsToSensors() {
  GB6010motor.linkSensor(&sensor2);
  GB4315motor.linkSensor(&sensor3);
  Serial.println("Motors linked to sensors");
}

// Initialize motors with FOC
void initializeMotors() {
  // Initialize GB6010 motor
  GB6010motor.init();
  GB6010motor.zero_electric_angle = motor_GB6010_ZEAngle;
  GB6010motor.sensor_direction = motor_GB6010_sensor_direction;
  GB6010motor.sensor_offset = motor_GB6010_ZeroCalibratedAngle;
  GB6010motor.initFOC();
  GB6010motor.voltage_limit = 7.0f;
  GB6010motor.voltage_sensor_align = 4.0f;
  
  // Initialize GB4315 motor
  GB4315motor.init();
  GB4315motor.zero_electric_angle = motor_GB4315_ZEAngle;
  GB4315motor.sensor_direction = motor_GB4315_sensor_direction;
  GB4315motor.sensor_offset = motor_GB4315_ZeroCalibratedAngle;
  GB4315motor.initFOC();
  GB4315motor.voltage_limit = 7.0f;
  GB4315motor.voltage_sensor_align = 3.0f;
  
  Serial.println("Motors initialized with FOC");
}

// Check if motor is in starting position
bool checkstarting(const MotorConfig& config) {
  config.sensor->update();
  float CurrentAngle = config.motor->shaftAngle();
  Serial.print("Current Angle CHECKER: ");
  Serial.println(CurrentAngle);
  if (CurrentAngle >= config.upper_limit || CurrentAngle < config.lower_limit) {
    Serial.println("Error: Motor not in starting position. Please move the motor to the home position (0 radians).");
    return 1;
  }
  else {
    Serial.println("Motor in starting position. Proceeding with homing sequence.");
    return 0;
  }
}

// Go to home position
bool goToHomePosition(const MotorConfig& config, unsigned long timeout_ms) {
    unsigned long start_time = millis();
    config.motor->target = config.home_angle;
    
    Serial.println("homing");
    Serial.println(config.name);
    while (fabs(config.motor->shaftAngle() - config.home_angle) > 0.01f){
        config.sensor->update();
        config.motor->loopFOC();
        config.motor->move();
        
        // Add timeout
        if (millis() - start_time > timeout_ms) {
            Serial.println("Homing sequence timed out!");
            return false;
        }
        
        // Optional: Add progress feedback
        if (millis() % 500 == 0) {
            Serial.print("Homing... Current angle: ");
            Serial.println(config.motor->shaftAngle());
        }
    }
    return true;
}

// Command function for target angle
void doTarget(char* cmd) { 
  extern Commander command;
  command.scalar(&target_Angle_1, cmd); 
}