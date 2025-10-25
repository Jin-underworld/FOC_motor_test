#include "motorHelper.h"
#include "sensorHelper.h"

// Motor and driver definitions

// Motor and driver definitions using the lookup tables
ExtendedBLDCMotor GB6025motor = ExtendedBLDCMotor(
    MOTOR_CHARACTERISTICS[MOTOR_GB6025].pole_pairs,
    MOTOR_CHARACTERISTICS[MOTOR_GB6025].resistance,
    MOTOR_CHARACTERISTICS[MOTOR_GB6025].kv,
    MOTOR_CHARACTERISTICS[MOTOR_GB6025].inductance,
    MOTOR_CHARACTERISTICS[MOTOR_GB6025].torque_constant
);
BLDCDriver3PWM GB6025driver = BLDCDriver3PWM(motor_GB6025_PWM_IN1, motor_GB6025_PWM_IN2, motor_GB6025_PWM_IN3, motor_GB6025_EN);

ExtendedBLDCMotor GB6010motor = ExtendedBLDCMotor(
    MOTOR_CHARACTERISTICS[MOTOR_GB6010].pole_pairs,
    MOTOR_CHARACTERISTICS[MOTOR_GB6010].resistance,
    MOTOR_CHARACTERISTICS[MOTOR_GB6010].kv,
    MOTOR_CHARACTERISTICS[MOTOR_GB6010].inductance,
    MOTOR_CHARACTERISTICS[MOTOR_GB6010].torque_constant
);
BLDCDriver3PWM GB6010driver = BLDCDriver3PWM(motor_GB6010_PWM_IN1, motor_GB6010_PWM_IN2, motor_GB6010_PWM_IN3, motor_GB6010_EN);

ExtendedBLDCMotor GB4315motor = ExtendedBLDCMotor(
    MOTOR_CHARACTERISTICS[MOTOR_GB4315].pole_pairs,
    MOTOR_CHARACTERISTICS[MOTOR_GB4315].resistance,
    MOTOR_CHARACTERISTICS[MOTOR_GB4315].kv,
    MOTOR_CHARACTERISTICS[MOTOR_GB4315].inductance,
    MOTOR_CHARACTERISTICS[MOTOR_GB4315].torque_constant
);
BLDCDriver3PWM GB4315driver = BLDCDriver3PWM(motor_GB4315_PWM_IN1, motor_GB4315_PWM_IN2, motor_GB4315_PWM_IN3, motor_GB4315_EN);

// Array of motor pointers for easy iteration
ExtendedBLDCMotor* Joint_motor[NUMBER_OF_JOINTS] = {&GB6025motor, &GB6010motor, &GB4315motor};

// Array of driver pointers
BLDCDriver3PWM* Joint_driver[NUMBER_OF_JOINTS] = {&GB6025driver, &GB6010driver, &GB4315driver};

// Target angles initialized from joint configs
float target_Angle_0 = JOINT_CONFIGS[JOINT_0].home_angle;
float target_Angle_1 = JOINT_CONFIGS[JOINT_1].home_angle;
float target_Angle_2 = JOINT_CONFIGS[JOINT_2].home_angle;

// Motor configurations (keeping for backward compatibility)
MotorConfig gb6025_config = {
  &GB6025motor, 
  &sensor0, 
  JOINT_CONFIGS[JOINT_0].upper_limit,
  JOINT_CONFIGS[JOINT_0].lower_limit,
  JOINT_CONFIGS[JOINT_0].home_angle,
  "GB6025"
};

MotorConfig gb6010_config = {
  &GB6010motor, 
  &sensor1, 
  JOINT_CONFIGS[JOINT_1].upper_limit,
  JOINT_CONFIGS[JOINT_1].lower_limit,
  JOINT_CONFIGS[JOINT_1].home_angle,
  "GB6010"
};

MotorConfig gb4315_config = {
  &GB4315motor, 
  &sensor2, 
  JOINT_CONFIGS[JOINT_2].upper_limit,
  JOINT_CONFIGS[JOINT_2].lower_limit,
  JOINT_CONFIGS[JOINT_2].home_angle,
  "GB4315"
};

// Initialize motor drivers using lookup tables
void initializeMotorDrivers() {
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    Joint_driver[i]->voltage_power_supply = driver_voltage_supply;
    Joint_driver[i]->init();
    Joint_driver[i]->pwm_frequency = 30000;
    
    Serial.print("Driver ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(getMotorCharacteristics((JointNumber)i).name);
    Serial.println(") initialized");
  }
  Serial.println("All motor drivers initialized");
}

void initializeMotorControllers() {
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    Joint_motor[i]->linkDriver(Joint_driver[i]);
    
    // Set control mode - torque control for impedance
    Joint_motor[i]->controller = MotionControlType::torque;
    
    Joint_motor[i]->foc_modulation = FOCModulationType::SpaceVectorPWM;
    
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(getMotorCharacteristics(i).name);
    Serial.println(") controller initialized");
  }
  Serial.println("All motor controllers initialized");
}
// Initialize motors with FOC using lookup tables
void initializeMotors() {
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    const JointConfiguration& joint_cfg = JOINT_CONFIGS[i];
    const MotorCharacteristics& motor_char = getMotorCharacteristics((JointNumber)i);
    
    Joint_motor[i]->init();
    Joint_motor[i]->zero_electric_angle = joint_cfg.zero_electric_angle;
    Joint_motor[i]->sensor_direction = joint_cfg.sensor_direction;
    Joint_motor[i]->sensor_offset = joint_cfg.zero_calibrated_angle;
    
    // Set voltage limits based on motor type
    if (joint_cfg.motor_type == MOTOR_GB6025) {
      Joint_motor[i]->voltage_limit = 12.0f;
      Joint_motor[i]->voltage_sensor_align = 5.0f;
    } else if (joint_cfg.motor_type == MOTOR_GB6010) {
      Joint_motor[i]->voltage_limit = 14.0f;
      Joint_motor[i]->current_limit = 0.7f;
      Joint_motor[i]->voltage_sensor_align = 4.0f;
    } else if (joint_cfg.motor_type == MOTOR_GB4315) {
      Joint_motor[i]->voltage_limit = 7.0f;
      Joint_motor[i]->voltage_sensor_align = 3.0f;
    }
    
    Joint_motor[i]->initFOC();
    
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(motor_char.name);
    Serial.print(" @ ");
    Serial.print(joint_cfg.joint_name);
    Serial.println(") initialized");
  }
  Serial.println("All motors initialized with FOC");
}

// Set motor control parameters
void initializeMotorParameters() {

  // GB6025 motor parameters
  //velocity PI controller parameters
  GB6025motor.PID_velocity.P = 0.5f;
  GB6025motor.PID_velocity.I = 3.2f;
  GB6025motor.PID_velocity.D = 0.0f;
  GB6025motor.LPF_velocity.Tf = 0.07; // filter for velocity, needs to be around 0.05s
  GB6025motor.P_angle.P = 4.0f;
  GB6025motor.P_angle.I = 0.0f;
  GB6025motor.P_angle.D = 5.2f;
  GB6025motor.P_angle.output_ramp = 0.2f;
  GB6025motor.LPF_angle.Tf = 0.12f;
  GB6025motor.velocity_limit = 20.0f;

  // GB6010 motor parameters
  //velocity PI controller parameters
  GB6010motor.PID_velocity.P = 0.5f;
  GB6010motor.PID_velocity.I = 3.2f;
  GB6010motor.PID_velocity.D = 0.0f;
  GB6010motor.LPF_velocity.Tf = 0.07; // filter for velocity, needs to be around 0.05s
  GB6010motor.P_angle.P = 6.0f;
  GB6010motor.P_angle.I = 0.01f;
  GB6010motor.P_angle.D = 5.8f;
  GB6010motor.P_angle.output_ramp = 0.2f;
  GB6010motor.LPF_angle.Tf = 0.12f;
  GB6010motor.velocity_limit = 20.0f;
  
  // GB4315 motor parameters
  //velocity PI controller parameters
  GB4315motor.PID_velocity.P = 0.5f;
  GB4315motor.PID_velocity.I = 3.2f;
  GB4315motor.PID_velocity.D = 0.0f;
  GB4315motor.LPF_velocity.Tf = 0.07; // filter for velocity, needs to be around 0.05s
  GB4315motor.P_angle.P = 2.0f;
  GB4315motor.P_angle.I = 0.0f;
  GB4315motor.P_angle.D = 4.2f;
  GB4315motor.P_angle.output_ramp = 0.2f;
  GB4315motor.LPF_angle.Tf = 0.12f;
  GB4315motor.velocity_limit = 20.0f;
  
  Serial.println("Motor parameters set");
}

// Link motors to sensors
void linkMotorsToSensors() {
  GB6025motor.linkSensor(&sensor0);
  GB6010motor.linkSensor(&sensor1);
  GB4315motor.linkSensor(&sensor2);
  Serial.println("Motors linked to sensors");
}


// Check if motor is in starting position
bool checkstarting(const MotorConfig& config) {
  config.sensor->update();
  float CurrentAngle = config.motor->shaftAngle();
  Serial.print("Current Angle CHECKER: ");
  Serial.println(CurrentAngle);
  if (CurrentAngle >= config.upper_limit || CurrentAngle < config.lower_limit) {
    Serial.print("Error: ");
    Serial.print(config.name);
    Serial.println(" not in starting position. Please move the motor to the home position.");
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
    while (fabs(config.motor->shaftAngle( ) - config.home_angle) > 0.01f){
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

void printRawSensorAngles() {
  Serial.print("Sensor angles: ");
  Serial.print(getMotorRawAngle(&GB6025motor), 4); Serial.print(", ");
  Serial.print(getMotorRawAngle(&GB6010motor), 4); Serial.print(", ");
  Serial.println(getMotorRawAngle(&GB4315motor), 4);
}

LowPassFilter LPF_angle0(0.25);

// Get motor angle (0-2π) with direction and offset but without revolution counting
float getMotorRawAngle(ExtendedBLDCMotor* motor) {
  if(!motor->sensor) return 0;
  
  // Get raw angle (0-2π) and apply direction and offset
  float raw_angle = motor->sensor_direction *  motor->sensor->getMechanicalAngle() - motor->sensor_offset;
  
  // Normalize to 0-2π range
  return _normalizeAngle(raw_angle);
}