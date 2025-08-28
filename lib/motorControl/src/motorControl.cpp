#include "motorControl.h"

MotorControl::MotorControl() 
    : sensor(AS5600_I2C),
      motor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE),
      driver(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE),
      max_torque(VOLTAGE_TO_TORQUE_RATIO * MOTOR_VOLTAGE_LIMIT),
      current_angle(0.0f),
      last_angle(0.0f)
{
    // Constructor
}

MotorControl::~MotorControl() {
    // Destructor - nothing specific to clean up
}

void MotorControl::init() {
    Serial.println("Starting gravity compensation setup...");
    
    // Initialize magnetic sensor
    sensor.init();
    motor.linkSensor(&sensor);
    
    Serial.print("Initial sensor angle: ");
    Serial.println(sensor.getAngle());
    
    // Driver config
    driver.voltage_power_supply = MOTOR_VOLTAGE_LIMIT;
    driver.init();
    
    // Link the motor and the driver
    motor.linkDriver(&driver);
    
    // Set motion control loop to be used
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;
    
    // Configure velocity PID controller
    motor.PID_velocity.P = MOTOR_PID_VELOCITY_P;
    motor.PID_velocity.I = MOTOR_PID_VELOCITY_I;
    motor.PID_velocity.D = MOTOR_PID_VELOCITY_D;
    motor.PID_velocity.output_ramp = MOTOR_PID_VELOCITY_RAMP;
    
    // Set voltage limit
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    
    // Angle P controller
    motor.P_angle.P = MOTOR_P_ANGLE;
    
    // Velocity limit
    motor.velocity_limit = MOTOR_VELOCITY_LIMIT;
    
    // Set predefined zero electric angle and sensor direction
    motor.zero_electric_angle = MOTOR_ZERO_ELECTRIC_ANGLE;
    motor.sensor_direction = MOTOR_SENSOR_DIRECTION;
    
    // Calculate maximum torque based on voltage limit
    max_torque = VOLTAGE_TO_TORQUE_RATIO * motor.voltage_limit;
    
    // Initialize motor and start FOC
    motor.init();
    motor.initFOC();
    
    Serial.println(F("Motor ready."));
    Serial.println("Starting...");
}

void MotorControl::loopFOC() {
    motor.loopFOC();
}

void MotorControl::compensateGravity() {
    // Get current angle with offset applied
    current_angle = sensor.getAngle() - MOTOR_ANGLE_OFFSET;
    
    // Calculate gravity compensation torque
    float gravity_torque = calculateGravityTorque(current_angle);
    
    // Calculate position error and correction torque
    float angle_error = 0; // This would be target - current if we had a target
    
    // Simple proportional control for position
    float position_torque = POSITION_GAIN * angle_error;
    
    // Add damping to prevent oscillation
    float velocity = (current_angle - last_angle) * 1000.0f;
    float damping_torque = 0; // DAMPING_GAIN * velocity; // Currently disabled
    last_angle = current_angle;
    
    // Total torque = gravity compensation + position control + damping
    float total_torque = gravity_torque + position_torque + damping_torque;
    
    // Limit the torque to safe values and convert to voltage
    float voltage_command = torqueToVoltage(total_torque);
    voltage_command = constrain(voltage_command, -motor.voltage_limit, motor.voltage_limit);
    
    // Apply the voltage command to the motor
    motor.move(-voltage_command);
    
    // Debug output
    Serial.print("Angle: "); Serial.print(current_angle, 3);
    Serial.print(", Gravity_T: "); Serial.print(gravity_torque, 4);
    Serial.print(", Velocity: "); Serial.print(velocity, 4);
    Serial.print(", Damping_T: "); Serial.print(damping_torque, 4);
    Serial.print(", Total_T: "); Serial.print(total_torque, 4);
    Serial.print(", Voltage: "); Serial.println(voltage_command, 4);
}

float MotorControl::calculateGravityTorque(float angle) {
    // Torque = mass * gravity * arm_length * sin(angle)
    // This gives the torque needed to counteract gravity
    return LOAD_MASS * GRAVITY * ARM_LENGTH * sin(angle);
}

float MotorControl::torqueToVoltage(float torque) {
    return torque / VOLTAGE_TO_TORQUE_RATIO;
}

void MotorControl::useMonitoring(Stream &serial) {
    motor.useMonitoring(serial);
}

void MotorControl::monitor() {
    motor.monitor();
}