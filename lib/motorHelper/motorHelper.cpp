#include "motorHelper.h"
#include "sensorHelper.h"
#include <SimpleFOC.h>

// Motor and driver definitions

// Motor and driver definitions using the lookup tables
ExtendedBLDCMotor GB6025motor = ExtendedBLDCMotor(
  MOTOR_CHARACTERISTICS[MOTOR_GB6025].pole_pairs,
  MOTOR_CHARACTERISTICS[MOTOR_GB6025].resistance,
  MOTOR_CHARACTERISTICS[MOTOR_GB6025].kv,
  MOTOR_CHARACTERISTICS[MOTOR_GB6025].inductance,
  MOTOR_CHARACTERISTICS[MOTOR_GB6025].torque_constant
);
BLDCDriver3PWM GB6025driver = BLDCDriver3PWM(
  M0_PWM_IN1, 
  M0_PWM_IN2, 
  M0_PWM_IN3, 
  M0_EN);

ExtendedBLDCMotor GB6010motor = ExtendedBLDCMotor(
  MOTOR_CHARACTERISTICS[MOTOR_GB6010].pole_pairs,
  MOTOR_CHARACTERISTICS[MOTOR_GB6010].resistance,
  MOTOR_CHARACTERISTICS[MOTOR_GB6010].kv,
  MOTOR_CHARACTERISTICS[MOTOR_GB6010].inductance,
  MOTOR_CHARACTERISTICS[MOTOR_GB6010].torque_constant
);
BLDCDriver3PWM GB6010driver = BLDCDriver3PWM(
  M1_PWM_IN1, 
  M1_PWM_IN2, 
  M1_PWM_IN3, 
  M1_EN);

ExtendedBLDCMotor GB4315motor = ExtendedBLDCMotor(
  MOTOR_CHARACTERISTICS[MOTOR_GB4315].pole_pairs,
  MOTOR_CHARACTERISTICS[MOTOR_GB4315].resistance,
  MOTOR_CHARACTERISTICS[MOTOR_GB4315].kv,
  MOTOR_CHARACTERISTICS[MOTOR_GB4315].inductance,
  MOTOR_CHARACTERISTICS[MOTOR_GB4315].torque_constant
);
BLDCDriver3PWM GB4315driver = BLDCDriver3PWM(
  M2_PWM_IN1, 
  M2_PWM_IN2, 
  M2_PWM_IN3, 
  M2_EN);

ExtendedBLDCMotor GB3510_3_motor = ExtendedBLDCMotor(
  MOTOR_CHARACTERISTICS[MOTOR_GB3510].pole_pairs,
  MOTOR_CHARACTERISTICS[MOTOR_GB3510].resistance,
  MOTOR_CHARACTERISTICS[MOTOR_GB3510].kv,
  MOTOR_CHARACTERISTICS[MOTOR_GB3510].inductance,
  MOTOR_CHARACTERISTICS[MOTOR_GB3510].torque_constant
);
BLDCDriver3PWM GB3510_3_driver = BLDCDriver3PWM(
  M3_PWM_IN1, 
  M3_PWM_IN2, 
  M3_PWM_IN3, 
  M3_EN);

// ExtendedBLDCMotor GB3510_4_motor = ExtendedBLDCMotor(
//   MOTOR_CHARACTERISTICS[MOTOR_GB3510].pole_pairs,
//   MOTOR_CHARACTERISTICS[MOTOR_GB3510].resistance,
//   MOTOR_CHARACTERISTICS[MOTOR_GB3510].kv,
//   MOTOR_CHARACTERISTICS[MOTOR_GB3510].inductance,
//   MOTOR_CHARACTERISTICS[MOTOR_GB3510].torque_constant
// );
// BLDCDriver3PWM GB3510_4_driver = BLDCDriver3PWM(
//   M4_PWM_IN1, 
//   M4_PWM_IN2, 
//   M4_PWM_IN3, 
//   M4_EN);
// Array of motor pointers for easy iteration
ExtendedBLDCMotor* Joint_motor[ACTIVE_JOINTS] = {
  &GB6025motor,
  &GB6010motor,
  &GB4315motor,
  &GB3510_3_motor
 };

// Array of driver pointers
BLDCDriver3PWM* Joint_driver[ACTIVE_JOINTS] = {
  &GB6025driver,
  &GB6010driver,
  &GB4315driver,
  &GB3510_3_driver
};

//Target angle constrains
float target_Angle_0 = JOINT_CONFIGS[JOINT_0].home_angle;//,JOINT0_LOWER_LIMIT,JOINT0_UPPER_LIMIT);
float target_Angle_1 = JOINT_CONFIGS[JOINT_1].home_angle;//,JOINT1_LOWER_LIMIT,JOINT1_UPPER_LIMIT);
float target_Angle_2 = JOINT_CONFIGS[JOINT_2].home_angle;//,JOINT2_LOWER_LIMIT,JOINT2_UPPER_LIMIT);
float target_Angle_3 = JOINT_CONFIGS[JOINT_3].home_angle;//,JOINT3_LOWER_LIMIT,JOINT3_UPPER_LIMIT);
//float target_Angle_4 = constrain(JOINT_CONFIGS[JOINT_4].home_angle,JOINT0_LOWER_LIMIT,JOINT0_UPPER_LIMIT);


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

MotorConfig gb3510_3_config = {
  &GB3510_3_motor, 
  &sensor3, 
  JOINT_CONFIGS[JOINT_3].upper_limit,
  JOINT_CONFIGS[JOINT_3].lower_limit,
  JOINT_CONFIGS[JOINT_3].home_angle,
  "GB3510_3"
};

// MotorConfig gb3510_4_config = {
//   &GB4315motor, 
//   &sensor4, 
//   JOINT_CONFIGS[JOINT_4].upper_limit,
//   JOINT_CONFIGS[JOINT_4].lower_limit,
//   JOINT_CONFIGS[JOINT_4].home_angle,
//   "GB3510_4"
// };


// Initialize motor drivers using lookup tables ORD1
void initializeMotorDrivers() {
  
  for (int i = 0; i < ACTIVE_JOINTS; i++) {
    Joint_driver[i]->voltage_power_supply = driver_voltage_supply;
    Joint_driver[i]->init();
    Joint_driver[i]->pwm_frequency = 30000;
    
    Serial2.print("Driver ");
    Serial2.print(i);
    Serial2.print(" (");
    Serial2.print(getMotorCharacteristics((JointNumber)i).name);
    Serial2.println(") initialized");
  }
  Serial2.println("All motor drivers initialized");
}

//ORD2
void initializeMotorControllers() {
  for (int i = 0; i < ACTIVE_JOINTS; i++) {
    Joint_motor[i]->linkDriver(Joint_driver[i]);
    
    // Set control mode - torque control for impedance
    Joint_motor[i]->controller = MotionControlType::angle;
    
    Joint_motor[i]->foc_modulation = FOCModulationType::SpaceVectorPWM;
    
    Serial2.print("Motor ");
    Serial2.print(i);
    Serial2.print(" (");
    Serial2.print(getMotorCharacteristics(i).name);
    Serial2.println(") controller initialized");
  }
  Serial2.println("All motor controllers initialized");
}

// TUNER VERY IMPORTANT //
// TUNER VERY IMPORTANT //
// TUNER VERY IMPORTANT //
bool Motor_Calib[ACTIVE_JOINTS] = {0,0,0};
// TUNER VERY IMPORTANT //
// TUNER VERY IMPORTANT //
// TUNER VERY IMPORTANT //

// Initialize motors with FOC using lookup tables ORD4
void initializeMotors() {
  for (int i = 0; i < ACTIVE_JOINTS; i++) {
    const JointConfiguration& joint_cfg = JOINT_CONFIGS[i];
    const MotorCharacteristics& motor_char = getMotorCharacteristics((JointNumber)i);
    
    //Joint_motor[i]->enableMechanicalMode();
    
    if(!Motor_Calib[i]){  
      Joint_motor[i]->zero_electric_angle = joint_cfg.zero_electric_angle;
      Joint_motor[i]->sensor_direction = joint_cfg.sensor_direction;
      Joint_motor[i]->sensor_offset = joint_cfg.zero_calibrated_angle;
    }

    Joint_motor[i]->init();
    
    // Set voltage limits based on motor type
    if (joint_cfg.motor_type == MOTOR_GB6025) {
      Joint_motor[i]->voltage_limit = 5.0f;
      Joint_motor[i]->voltage_sensor_align = 3.0f;
    } else if (joint_cfg.motor_type == MOTOR_GB6010) {
      Joint_motor[i]->voltage_limit = 13.0f; //technically limit is 8V without fan, limited by driver
      Joint_motor[i]->voltage_sensor_align = 4.0f;
    } else if (joint_cfg.motor_type == MOTOR_GB4315) {
      Joint_motor[i]->voltage_limit = 7.0f; //limited by motor current
      Joint_motor[i]->voltage_sensor_align = 3.0f;
    } else if (joint_cfg.motor_type == MOTOR_GB3510) {
      Joint_motor[i]->voltage_limit = 5.0f;
      Joint_motor[i]->voltage_sensor_align = 3.0f;
    } 
    
    
    Joint_motor[i]->initFOC();
    
    Serial2.print("Motor ");
    Serial2.print(i);
    Serial2.print(" (");
    Serial2.print(motor_char.name);
    Serial2.print(" @ ");
    Serial2.print(joint_cfg.joint_name);
    Serial2.println(") initialized");
  }
  Serial2.println("All motors initialized with FOC");
}

// Set motor control parameters ORD3
void initializeMotorParameters() {

  // GB6025 motor parameters
  //velocity PI controller parameters
  GB6025motor.PID_velocity.P = 0.59f;
  GB6025motor.PID_velocity.I = 8.9f;
  GB6025motor.PID_velocity.D = 0.001f;
  GB6025motor.LPF_velocity.Tf = 0.035; // filter for velocity
  GB6025motor.P_angle.P = 0.2f;
  GB6025motor.P_angle.I = 0.05f;
  GB6025motor.P_angle.D = 0.001f;
  GB6025motor.P_angle.output_ramp = 5000;
  GB6025motor.LPF_angle.Tf = 0.0f;
  GB6025motor.velocity_limit = 4.0f;

  // GB6010 motor parameters
  //velocity PI controller parameters
  GB6010motor.PID_velocity.P = 0.59f;
  GB6010motor.PID_velocity.I = 8.9f;
  GB6010motor.PID_velocity.D = 0.001f;
  GB6010motor.LPF_velocity.Tf = 0.035; // filter for velocity
  GB6010motor.P_angle.P = 0.2f;
  GB6010motor.P_angle.I = 0.05f;
  GB6010motor.P_angle.D = 0.001f;
  GB6010motor.P_angle.output_ramp = 5000;
  GB6010motor.LPF_angle.Tf = 0.0f;
  GB6010motor.velocity_limit = 4.0f;
  
  // GB4315 motor parameters
  //velocity PI controller parameters
  GB4315motor.PID_velocity.P = 0.33f;
  GB4315motor.PID_velocity.I = 1.2f;
  GB4315motor.PID_velocity.D = 0.001f;
  GB4315motor.LPF_velocity.Tf = 0.12f; // filter for velocity
  GB4315motor.P_angle.P = 0.18f;
  GB4315motor.P_angle.I = 0.0f;
  GB4315motor.P_angle.D = 0.001f;
  GB4315motor.P_angle.output_ramp = 2000;
  GB4315motor.LPF_angle.Tf = 0.05f;
  GB4315motor.velocity_limit = 2.0f;

  //GB3510_3 motor parameters
  //velocity PI controller parameters
  GB3510_3_motor.PID_velocity.P = 0.39f;
  GB3510_3_motor.PID_velocity.I = 7.1f;
  GB3510_3_motor.PID_velocity.D = 0.001f;
  GB3510_3_motor.LPF_velocity.Tf = 0.035f; // filter for velocity
  GB3510_3_motor.P_angle.P = 0.25f;
  GB3510_3_motor.P_angle.I = 0.0f;
  GB3510_3_motor.P_angle.D = 0.001f;
  GB3510_3_motor.P_angle.output_ramp = 2000;
  GB3510_3_motor.LPF_angle.Tf = 0.05f;
  GB3510_3_motor.velocity_limit = 3.0f;

  // //GB3510_4 motor parameters
  // //velocity PI controller parameters
  // GB3510_4_motor.PID_velocity.P = 0.5f;
  // GB3510_4_motor.PID_velocity.I = 3.2f;
  // GB3510_4_motor.PID_velocity.D = 0.0f;
  // GB3510_4_motor.LPF_velocity.Tf = 0.07; // filter for velocity, needs to be around 0.05s
  // GB3510_4_motor.P_angle.P = 2.0f;
  // GB3510_4_motor.P_angle.I = 0.0f;
  // GB3510_4_motor.P_angle.D = 4.2f;
  // GB3510_4_motor.P_angle.output_ramp = 0.2f;
  // GB3510_4_motor.LPF_angle.Tf = 0.12f;
  // GB3510_4_motor.velocity_limit = 20.0f;
  
  
  Serial2.println("Motor parameters set");
}

// Link motors to sensors ORD0
void linkMotorsToSensors() {
  GB6025motor.linkSensor(&sensor0);
  GB6010motor.linkSensor(&sensor1);
  GB4315motor.linkSensor(&sensor2);
  GB3510_3_motor.linkSensor(&sensor3);
  //GB3510_4_motor.linkSensor(&sensor4);
  Serial2.println("Motors linked to sensors");
}


// Check if motor is in starting position
bool checkstarting(const MotorConfig& config) {
  config.sensor->update();
  float CurrentAngle = config.motor->shaftAngle();
  Serial2.print("Current Angle CHECKER: ");
  Serial2.println(CurrentAngle);
  if (CurrentAngle >= config.upper_limit || CurrentAngle < config.lower_limit) {
    Serial2.print("Error: ");
    Serial2.print(config.name);
    Serial2.println(" not in starting position. Please move the motor to the home position.");
    return 1;
  }
  else {
    Serial2.println("Motor in starting position. Proceeding with homing sequence.");
    return 0;
  }
}

// Go to home position
bool goToHomePosition(const MotorConfig& config, unsigned long timeout_ms) {
    unsigned long start_time = millis();
    config.motor->target = config.home_angle;
    
    Serial2.println("homing");
    Serial2.println(config.name);
    while (fabs(config.motor->shaftAngle( ) - config.home_angle) > 0.01f){
        config.sensor->update();
        config.motor->loopFOC();
        config.motor->move();
        
        // Add timeout
        if (millis() - start_time > timeout_ms) {
            Serial2.println("Homing sequence timed out!");
            return false;
        }
        
        // Optional: Add progress feedback
        if (millis() % 500 == 0) {
            Serial2.print("Homing... Current angle: ");
            Serial2.println(config.motor->shaftAngle());
        }
    }
    return true;
}

// Command function for target angle
// void doTarget(char* cmd) { 
//   extern Commander command;
//   command.scalar(&target_Angle_1, cmd); 
// }


//get all active joint motor angles with sensor offset
void printRawMotorAngles() {
    updateAllEncoders();
    
    Serial2.print("[");
    for (int i =0;i<ACTIVE_JOINTS;i++){
    Serial2.print(getMotorRawAngle(Joint_motor[i]), 3);
    Serial2.print(",");
    }
    Serial2.println("]");
}



//LowPassFilter LPF_angle0(0.04);

// // Get motor angle (-π to π) with direction and offset but without revolution counting
// float getMotorRawAngle(ExtendedBLDCMotor* motor) {
//   if(!motor->sensor) return 0;
  
//   // Get raw angle and apply direction and offset
//   float raw_angle = motor->sensor_direction * motor->sensor->getMechanicalAngle() - motor->sensor_offset;
  
//   // Normalize to -π to π range
//   raw_angle = fmod(raw_angle, _2PI);
//   if (raw_angle > _PI) {
//     raw_angle -= _2PI;
//   } else if (raw_angle < -_PI) {
//     raw_angle += _2PI;
//   }
  
//   return raw_angle;
// }
float getMotorRawAngleNooffset(ExtendedBLDCMotor* motor) {
if(!motor->sensor) return 0;

// Get raw angle (0-2π) and apply direction and offset
float raw_angle = motor->mech_angle_lpf(motor->sensor->getMechanicalAngle()) - motor->sensor_offset;
//LPF updated here on permotor basis. MUST BE KEPT otherwise motors propagate

// Normalize to 0-2π range
return _normalizeAngle(raw_angle);
}


// use &motor, Get motor angle (0-2π) with direction and offset but without revolution counting
float getMotorRawAngle(ExtendedBLDCMotor* motor) {
if(!motor->sensor) return 0;

// Get raw angle (0-2π) and apply direction and offset
float raw_angle = motor->sensor_direction * motor->mech_angle_lpf(motor->sensor->getMechanicalAngle()) - motor->sensor_offset;
//LPF updated here on permotor basis. MUST BE KEPT otherwise motors propagate

// Normalize to 0-2π range
return _normalizeAngle(raw_angle);
}

// Override move function in ExtendedBLDCMotor
void ExtendedBLDCMotor::move(float new_target) {
    if (use_mechanical_only) {
        // MECHANICAL-ONLY MODE: Use 0-2π range, no revolution tracking
        
        // Set internal target variable
        if (_isset(new_target)) target = new_target;
        
        // Normalize target to 0-2π range
        target = _normalizeAngle(target);
        
        // Downsampling (optional)
        if (motion_cnt++ < motion_downsample) return;
        motion_cnt = 0;

        // Get current mechanical angle (0-2π range)
        float current_mechanical_angle = getMechanicalShaftAngle();
        
      
        
        // Update shaft_angle to current mechanical angle (no revolution tracking)
        shaft_angle = current_mechanical_angle;
        shaft_velocity = shaftVelocity();

        // If disabled do nothing
        if (!enabled) return;
        
        // Calculate the back-emf voltage if KV_rating available
        if (_isset(KV_rating)) voltage_bemf = shaft_velocity / (KV_rating * _SQRT3) / _RPM_TO_RADS;
        
        // Estimate the motor current if phase resistance available and current_sense not available
        if (!current_sense && _isset(phase_resistance)) current.q = (voltage.q - voltage_bemf) / phase_resistance;

        // Control logic for angle mode
        if (controller == MotionControlType::angle) {
            // Calculate shortest angular difference for position control
            float angle_error = calculateShortestAngleDifference(target, current_mechanical_angle);
            
            // optional micro-deadband to kill dithering at setpoint
            const float deadband = 0.08f; // 
            if (fabsf(angle_error) < deadband) angle_error = 0.0f;

            // Angle set point (use current + error to avoid wrapping issues)
            shaft_angle_sp = current_mechanical_angle + angle_error;
            
            // Calculate velocity set point
            shaft_velocity_sp = feed_forward_velocity + P_angle(angle_error);
            shaft_velocity_sp = _constrain(shaft_velocity_sp, -velocity_limit, velocity_limit);
            
            // Calculate the torque command
            current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);
            
            // If torque controlled through voltage
            if (torque_controller == TorqueControlType::voltage) {
                if (!_isset(phase_resistance)) voltage.q = current_sp;
                else voltage.q = _constrain(current_sp * phase_resistance + voltage_bemf, -voltage_limit, voltage_limit);
                
                if (!_isset(phase_inductance)) voltage.d = 0;
                else voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
            }
        } else {
            // For other control modes, use base class behavior but with mechanical angles
            BLDCMotor::move(new_target);
        }
        
    } else {
        // FULL ROTATION MODE: Use original SimpleFOC behavior
        BLDCMotor::move(new_target);
    }
}

float ExtendedBLDCMotor::getMechanicalShaftAngle() {
    if (!sensor) return 0;
    
    // Use your existing function to get mechanical angle in 0-2π range
    return getMotorRawAngle(this);
}

float ExtendedBLDCMotor::calculateShortestAngleDifference(float target, float current) {
    // Calculate the difference between target and current angle
    float diff = target - current;
    
    // Wrap to [-π, π] to find shortest path
    while (diff > _PI) diff -= _2PI;
    while (diff < -_PI) diff += _2PI;
    
    return diff;
}