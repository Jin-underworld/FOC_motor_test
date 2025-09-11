#include <SimpleFOC.h>
#include "main.h"

MagneticSensorPWM sensor = MagneticSensorPWM(PIN_ENCODER, 3, 909);

void doPWM() {
    sensor.handlePWM();
}

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE); // 7 pole pairs, 3.0 ohm phase resistance
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C);//arduino uno

float current_angle = 0;
float compensate_angle = 1.95f + 0.166;
float cleaned_angle = 0;
float angle_error = 0;
float position_gain = 0.5f;  
float position_torque = 0;//position_gain * angle_error;
float damping_torque = 0;   
int mode = -1; //1 or -1 to change direction of compensation
float lift_ratio = 1.3f; // to slightly overcompensate gravity

// Constants for homing
#define HOMING_VOLTAGE 12.0f      // Lower voltage for safe homing
#define HOMING_TIMEOUT 10000     // 10 seconds timeout for homing
#define STALL_THRESHOLD 0.4f    // Threshold to detect when motor stalls at endstop
#define STALL_TIME 500  

// Properly declare global variables for angle filtering
float last_valid_angle = 0.0f;
float previous_angle = 0.0f;
int zero_count = 0;

// Function declarations
bool performHoming();
float calculateGravityTorque(float angle);
float cleanAngle(float angle);
float torqueToVoltage(float torque);

float max_torque = 2.0; // maximum torque limit [Nm]
float current_torque = 0.0; // current torque value
float voltage_to_move = 0.0; // target angle value

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
    Serial.begin(BAUD_RATE);
    Serial.println("Starting gravity compensation setup...");

    // initialise magnetic sensor hardware
    sensor.init();
    sensor.enableInterrupt(doPWM);
    motor.linkSensor(&sensor);
    Serial.print("Initial sensor angle: ");
    Serial.println(sensor.getAngle());

    // driver config
    driver.voltage_power_supply = MOTOR_VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);
    // maximal voltage to be set to the motor
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;

    // set motion control loop to be used
    motor.torque_controller = TorqueControlType::voltage; //changed to voltage control*
    motor.controller = MotionControlType::torque; //changed to torque control*

    // velocity PI controller parameters
    motor.PID_velocity.P = MOTOR_PID_VELOCITY_P;
    motor.PID_velocity.I = MOTOR_PID_VELOCITY_I;
    motor.PID_velocity.D = MOTOR_PID_VELOCITY_D;
    motor.PID_velocity.output_ramp = MOTOR_PID_VELOCITY_RAMP; // acceleration [rad/s^2]
    motor.P_angle.P = 2.0f;
    motor.velocity_limit = 20;

    max_torque = VOLTAGE_TO_TORQUE_RATIO * motor.voltage_limit; // [Nm] set the maximum torque*

    motor.zero_electric_angle = 2.21; // 2.21 cal. you can set it to a previously found value
    motor.sensor_direction = MOTOR_SENSOR_DIRECTION;

    // initialize motor
    motor.init();
    // align sensor and start FOC
  
    motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set

    Serial.println("Motor initialized. Beginning homing procedure...");
    delay(1000);
    
    // Perform homing to establish angle references
    if (performHoming()) {
        Serial.println("Homing successful!");
        Serial.print("Reference angle set. Compensate angle: ");
        Serial.println(compensate_angle);
    } else {
        Serial.println("Homing failed! Check motor and linkage.");
        while(1) { delay(1000); } // Halt execution if homing fails
    }

    last_valid_angle = 0.0f; // At this point 0 is vertical (after compensation)
    previous_angle = 0.0f;
    zero_count = 0;

    Serial.println("System ready. Starting gravity compensation...");
    delay(1000);
}

bool performHoming() {
    unsigned long startTime = millis();
    unsigned long stallStartTime = 0;
    bool isStalled = false;
    float prev_angle = sensor.getAngle();
    
    // Correct homing direction calculation
    float homing_direction = (motor.sensor_direction == CCW) ? -1.0f : 1.0f;
    
    Serial.println("Moving to endstop...");
    Serial.print("Homing direction: ");
    Serial.println(homing_direction);
    
    // Apply voltage in proper direction until stall is detected
    while (millis() - startTime < HOMING_TIMEOUT) {
        // Ensure FOC loop runs frequently for smooth operation
        motor.loopFOC();
        
        // Apply low voltage to move toward endstop
        motor.move(homing_direction * HOMING_VOLTAGE);
        
        // Read current angle
        float current_angle = sensor.getAngle();
        
        // Debug output to monitor progress
        if (millis() % 100 == 0) {  // Print every 100ms
            Serial.print("Angle: ");
            Serial.print(current_angle);
            Serial.print(" Delta: ");
            Serial.println(abs(current_angle - prev_angle));
        }
        
        // Check if motor is stalled (angle not changing despite voltage)
        if (abs(current_angle - prev_angle) < STALL_THRESHOLD) {
            if (!isStalled) {
                // Start timing the stall
                stallStartTime = millis();
                isStalled = true;
                Serial.println("Potential stall detected!");
            } else if (millis() - stallStartTime > STALL_TIME) {
                // Stall confirmed - endstop reached
                motor.move(0); // Stop motor
                
                // Set compensate_angle so that current position is -90 degrees
                compensate_angle = current_angle + PI/2;
                Serial.print("Stall confirmed at angle: ");
                Serial.println(current_angle);
                
                return true;
            }
        } else {
            // Motor is moving, reset stall detection
            if (isStalled) {
                Serial.println("Motor still moving - not stalled");
            }
            isStalled = false;
        }
        
        prev_angle = current_angle;
        delay(5); // Shorter delay for more responsive FOC
    }
    
    // If we get here, homing timed out
    motor.move(0); // Stop motor
    Serial.println("Homing timed out!");
    return false;
}

float calculateGravityTorque(float angle) {
    // Torque = mass * gravity * arm_length * sin(angle)
    // This gives the torque needed to counteract gravity
    float gravity_torque = LOAD_MASS * GRAVITY * ARM_LENGTH * sin(angle);
    return gravity_torque;
}


float cleanAngle(float angle) {
    // First, handle zero filtering
    if (angle == 0.0f) {
        zero_count++;
        if (zero_count >= 3) {
            return last_valid_angle;  // Filter out consecutive 0s
        }
    } else {
        zero_count = 0;  // Reset zero counter
    }
    
    // Now, check for consecutive matching values (but accept if it's the first non-zero or matches)
    if (angle == previous_angle || last_valid_angle == 0.0f) {  // Accept if matches or if last_valid is still initial 0
        // Value appeared twice or is first valid, accept it
        last_valid_angle = angle;
    } else {
        // Doesn't match previous, filter out: return last valid
        return last_valid_angle;
    }
    
    // Update previous for next call
    previous_angle = angle;
    return angle;
}

float torqueToVoltage(float torque) {
    return torque / VOLTAGE_TO_TORQUE_RATIO;
}

void loop() {
    motor.loopFOC();

    current_angle = sensor.getAngle() - compensate_angle; // adjust this offset to set the "zero" position*
  
    // Calculate gravity compensation torque
    float gravity_torque = calculateGravityTorque(current_angle);
    
    position_torque = position_gain * angle_error;  // This was commented out before
    
    float total_torque = gravity_torque*lift_ratio + position_torque + damping_torque;
    voltage_to_move = torqueToVoltage(total_torque);
    voltage_to_move = constrain(voltage_to_move, -motor.voltage_limit, motor.voltage_limit);

    //MAGIC 
    //motor.move(mode * voltage_to_move); // Uncommented to actually move the motor

    Serial.print("Raw Angle: "); Serial.print(current_angle, 3);
    Serial.print(", Gravity Torque: "); Serial.println(gravity_torque, 3);
    Serial.print(", Voltage: "); Serial.println(voltage_to_move, 2);
    
    // command.run();
}