// #include <SimpleFOC.h>
// #include "main.h"

// MagneticSensorPWM sensor = MagneticSensorPWM(PIN_ENCODER, 3, 909);

// void doPWM() {sensor.handlePWM();}

// // BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_TORQUE_CONSTANT, MOTOR_PHASE_INDUCTANCE); 
// BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);

// Commander command = Commander(Serial);
// void doMotor(char* cmd) { command.motor(&motor, cmd); }

// struct AdmittanceParams {
//     float M_d = 2;    // Desired inertia [kg*m^2]
//     float B_d = 15.0;    // Desired damping [N*m*s/rad]
//     float K_d = 5.0;   // Desired stiffness [N*m/rad]
//     float dt = 0.005;   // Control loop time step [s]
// };

// // Control State Variables
// struct ControlState {
//     float pos_ref = 0;      // Position reference [rad]
//     float vel_ref = 0;      // Velocity reference [rad/s]
//     float acc_ref = 0;      // Acceleration reference [rad/s^2]
//     float pos_actual = 0;   // Actual position [rad]
//     float vel_actual = 0;   // Actual velocity [rad/s]
//     float pos_prev = 0;     // Previous position for velocity calculation
//     float vel_prev = 0;     // Previous velocity for acceleration calculation
//     float force_ext = 0;    // External force/torque [N*m]
//     float current_measured = 0;  // Measured motor current [A]
//     float hold_pos_target = 0;
//     bool hold_position = true;
// };

// struct DiagnosticData {
//     float raw_pos;        // Raw position from sensor
//     float raw_vel;        // Raw velocity before filtering
//     float filtered_vel;   // Filtered velocity
//     float motor_q_current; // Motor q-axis current
//     float target_vel;     // Target velocity
//     float loop_time;      // Control loop execution time
// };

// DiagnosticData diag;

// // Force/Torque Estimation Parameters
// volatile float Kt = MOTOR_TORQUE_CONSTANT;  // Motor torque constant [N*m/A] - calibrate for your motor
// const float current_threshold = 0.05;  // Minimum current for force detection [A]

// class MedianFilter {
// private:
//     static const int SIZE = 7;
//     float values[SIZE];
//     int index = 0;
    
// public:
//     float update(float new_value) {
//         // Store new value
//         values[index] = new_value;
//         index = (index + 1) % SIZE;
        
//         // Copy values for sorting
//         float sorted[SIZE];
//         memcpy(sorted, values, sizeof(values));
        
//         // Simple bubble sort
//         for (int i = 0; i < SIZE-1; i++) {
//             for (int j = 0; j < SIZE-i-1; j++) {
//                 if (sorted[j] > sorted[j+1]) {
//                     float temp = sorted[j];
//                     sorted[j] = sorted[j+1];
//                     sorted[j+1] = temp;
//                 }
//             }
//         }
        
//         // Return median
//         return sorted[SIZE/2];
//     }
// };

// class VelocityEstimator {
// private:

//     static const int BUFFER_SIZE = 10;
//     float pos_buffer[BUFFER_SIZE];
//     unsigned long time_buffer[BUFFER_SIZE];
//     int buffer_index = 0;
//     bool buffer_filled = false;
//     float last_velocity = 0;

// public:
//     void init(float initial_pos) {
//         for(int i = 0; i < BUFFER_SIZE; i++) {
//             pos_buffer[i] = initial_pos;
//             time_buffer[i] = micros();
//         }
//     }
    
//     float update(float new_pos) {
//         unsigned long current_time = micros();
        
//         // Store new values
//         buffer_index = (buffer_index + 1) % BUFFER_SIZE;
//         pos_buffer[buffer_index] = new_pos;
//         time_buffer[buffer_index] = current_time;
        
//         if (!buffer_filled && buffer_index == BUFFER_SIZE - 1) {
//             buffer_filled = true;
//         }
        
//         // Calculate velocity using least squares over buffer
//         int samples = buffer_filled ? BUFFER_SIZE : (buffer_index + 1);
//         if (samples < 2) return 0;
        
//         // Get oldest valid sample
//         int oldest_index = buffer_filled ? 
//             (buffer_index + 1) % BUFFER_SIZE : 0;
        
//         float delta_pos = new_pos - pos_buffer[oldest_index];
//         float delta_time = (current_time - time_buffer[oldest_index]) / 1000000.0;
        
//         if (delta_time < 0.001) return last_velocity; // Too short time
        
//         float velocity = delta_pos / delta_time;
        
//         // Apply median filter for spike rejection
//         velocity = 0.7 * last_velocity + 0.3 * velocity;
//         last_velocity = velocity;
        
//         return velocity;
//     }
// };

// // Create filter instances using SimpleFOC's LowPassFilter
// // Add a position filter before using sensor readings
// LowPassFilter position_filter = LowPassFilter(0.05); // Strong filtering for position

// LowPassFilter velocity_filter = LowPassFilter(LowPassVelFilter); // time constant (roughly 10Hz cutoff)
// LowPassFilter current_filter = LowPassFilter(LowPassCurrFilter); // time constant (roughly 50Hz cutoff)
// LowPassFilter torque_filter = LowPassFilter(LowPassTorqueFilter); //  time constant for torque smoothing
// LowPassFilter target_filter = LowPassFilter(0.005);  // 5ms for smooth target

// AdmittanceParams admittance;
// ControlState state;
// VelocityEstimator vel_estimator;
// MedianFilter pos_median_filter;

// // PID controller for position tracking
// PIDController pid_position = PIDController(MOTOR_PID_VELOCITY_P, MOTOR_PID_VELOCITY_I, MOTOR_PID_VELOCITY_D, MOTOR_PID_VELOCITY_RAMP, MOTOR_VOLTAGE_LIMIT);  // P, I, D, ramp, limit

// bool bypass_admittance = false;

// void setup() {

//     Serial.begin(BAUD_RATE);
//     Serial.println("Initializing Admittance Control...");
    
//     // Initialize sensor
//     sensor.init();
//     sensor.enableInterrupt(doPWM);
//     motor.linkSensor(&sensor);

//     // Initialize driver
//     driver.voltage_power_supply = MOTOR_VOLTAGE_LIMIT;
//     driver.init();
//     motor.linkDriver(&driver);
    
//     motor.controller = MotionControlType::angle;
//     motor.target = state.hold_pos_target;  // Use your desired position target

//     // motor.controller = MotionControlType::velocity;
//     // motor.target = state.vel_ref;  // Use your velocity reference

//     motor.velocity_limit = MOTOR_VELOCITY_LIMIT;
    
//     // Initialize motor
//     motor.init();
//     motor.initFOC();

//     // Initialize control state
//     float initial_pos = sensor.getAngle();
//     state.pos_ref = initial_pos;
//     state.pos_actual = initial_pos;
//     state.hold_pos_target = initial_pos;
//     state.hold_position = true;  // Start in hold mode
    
//     // Initialize velocity estimator
//     vel_estimator.init(initial_pos);

//     // Set initial control mode
//     motor.target = 0;

//     Serial.println("Admittance Control Initialized");
    
//     // Initialize control state
//     state.pos_ref = sensor.getAngle();
//     state.pos_actual = state.pos_ref;
    
//     command.add('M',doMotor,"motor");
//     motor.useMonitoring(Serial);
    
//     delay(1000);
// }

// // Function to estimate external torque from current
// float estimateExternalTorque() {
//     // More sophisticated torque estimation
//     static float baseline_current = 0;
//     static bool baseline_set = false;
//     static unsigned long baseline_timer = 0;
    
//     // Calibrate baseline current after startup
//     if (!baseline_set && millis() > 3000) {
//         if (abs(state.vel_actual) < 0.01) {  // Only when stationary
//             baseline_current = 0.9 * baseline_current + 0.1 * motor.current.q;
//             if (millis() - baseline_timer > 1000) {
//                 baseline_set = true;
//                 Serial.print("Baseline current set: ");
//                 Serial.println(baseline_current);
//             }
//         } else {
//             baseline_timer = millis();
//         }
//     }
    
//     // Get filtered current
//     float current = current_filter(motor.current.q);
//     state.current_measured = current;
    
//     // Calculate external torque
//     float torque_ext = Kt * (current - baseline_current);
    
//     // Apply deadband
//     if (abs(torque_ext) < current_threshold * Kt) {
//         torque_ext = 0;
//     }
    
//     // Apply torque filter
//     return torque_filter(torque_ext);
// }

// // Admittance control law implementation
// void updateAdmittanceControl() {

//     static float last_valid_pos = state.pos_actual;

//     if (abs(state.pos_actual - last_valid_pos) < 0.03) { // ~1.7 degrees deadband
//         state.pos_actual = last_valid_pos;
//     } else {
//         last_valid_pos = state.pos_actual;
//     }

//     static unsigned long last_time = micros();
//     unsigned long current_time = micros();
//     float dt = (current_time - last_time) / 1000000.0;
    
//     // Skip if dt is unreasonable
//     if (dt < 0.001 || dt > 0.1) {
//         return;
//     }
    
//     last_time = current_time;
    
//     // Get current position
//     state.pos_actual = position_filter(sensor.getAngle());
    
//     // Estimate velocity with improved estimator
//     state.vel_actual = vel_estimator.update(state.pos_actual);
    
//     // Estimate external torque
//     state.force_ext = estimateExternalTorque();
    
//     // ============= CONTROL MODE LOGIC =============
//     if (state.hold_position) {
//         // POSITION HOLD MODE - Stiff behavior with compliance under load
        
//         // If significant external force, allow compliant motion
//         if (abs(state.force_ext) > 0.5) {
//             // Temporarily reduce stiffness for compliance
//             float temp_K = admittance.K_d * 0.3;  // Softer when pushed
//             float temp_B = admittance.B_d * 1.5;  // More damping
            
//             float pos_error = state.hold_pos_target - state.pos_actual;
            
//             // Admittance dynamics
//             state.acc_ref = (state.force_ext - temp_B * state.vel_actual - 
//                            temp_K * pos_error) / admittance.M_d;
            
//             state.vel_ref = state.acc_ref * dt;  // Don't integrate velocity in hold mode
//             if (motor.controller == MotionControlType::angle) {
//                 // Apply stronger filtering to hold_pos_target
//                 static LowPassFilter hold_pos_filter = LowPassFilter(0.01); // Very strong filtering
//                 state.hold_pos_target = hold_pos_filter(state.hold_pos_target);
//             } else {        
//                 state.hold_pos_target += state.vel_ref * dt * 0.1;  
//             }

            
//         } else {
//             // No external force - maintain position strongly
//             float pos_error = state.hold_pos_target - state.pos_actual;
            
//             // Strong position feedback
//             state.vel_ref = pos_error * 5.0;  // P-control for position
            
//             // Add damping
//             state.vel_ref -= state.vel_actual * 2.0;  // D-control
            
//             // Limit velocity command
//             state.vel_ref = constrain(state.vel_ref, -2.0, 2.0);
//         }
        
//     } else {
//         // COMPLIANT MODE - Full admittance behavior
//         float pos_error = state.pos_ref - state.pos_actual;
        
//         // Standard admittance equation
//         state.acc_ref = (state.force_ext - admittance.B_d * state.vel_actual - 
//                        admittance.K_d * pos_error) / admittance.M_d;
        
//         // Integrate for velocity reference
//         state.vel_ref += state.acc_ref * dt;
//         state.pos_ref += state.vel_ref * dt;
        
//         // Update hold position to track current position
//         state.hold_pos_target = state.pos_actual;
//     }
    
//     // Apply safety limits
//     state.vel_ref = constrain(state.vel_ref, -5.0, 5.0);
//     state.pos_ref = constrain(state.pos_ref, -2*PI, 2*PI);
// }

// // Timer for consistent control loop timing
// unsigned long loop_timestamp = 0;

// // Function to receive parameter updates via Serial
// void serialReceiveCommands() {
//     if (Serial.available()) {
//         char cmd = Serial.read();
//         float value = Serial.parseFloat();
        
//         switch(cmd) {
//             case 'M':  // Set desired mass
//                 admittance.M_d = constrain(value, 0.01, 10.0);
//                 Serial.print("M_d set to: "); Serial.println(admittance.M_d);
//                 break;

//             case 'B':  // Set desired damping
//                 admittance.B_d = constrain(value, 0.1, 100.0);
//                 Serial.print("B_d set to: "); Serial.println(admittance.B_d);
//                 break;

//             case 'K':  // Set desired stiffness
//                 admittance.K_d = constrain(value, 0.1, 1000.0);
//                 Serial.print("K_d set to: "); Serial.println(admittance.K_d);
//                 break;

//             case 'P':  // Set position reference
//                 state.pos_ref = constrain(value, -PI, PI);
//                 Serial.print("Position reference set to: "); Serial.println(state.pos_ref);
//                 break;

//             case 'R':  // Reset to current position
//                 state.pos_ref = state.pos_actual;
//                 state.vel_ref = 0;
//                 Serial.println("Reset to current position");
//                 break;

//             case 'T':  // Calibrate torque constant
//                 Kt = constrain(value, 0.01, 1.0);
//                 Serial.print("Kt set to: "); Serial.println(Kt);
//                 break;

//             case 'H':  // Toggle hold position mode
//                 state.hold_position = !state.hold_position;
//                 if (state.hold_position) {
//                     state.hold_pos_target = state.pos_actual;
//                     state.vel_ref = 0;
//                     Serial.println("HOLD mode ON - Position locked");
//                 } else {
//                     state.pos_ref = state.pos_actual;
//                     Serial.println("COMPLIANT mode ON - Free movement");
//                 }
//                 break;

//             case 'Z':  // Zero velocity (emergency stop)
//                 motor.target = 0;
//                 state.vel_ref = 0;
//                 state.hold_pos_target = state.pos_actual;
//                 Serial.println("Velocity zeroed");
//                 break;

//             case 'V':  // Direct velocity command (bypass admittance)
//                 motor.target = constrain(value, -5.0, 5.0);
//                 Serial.print("Direct velocity set to: "); Serial.println(motor.target);
//                 break;

//             case 'D':  // Bypass admittance control
//                 bypass_admittance = !bypass_admittance;
//                 Serial.print("Bypass admittance: "); 
//                 Serial.println(bypass_admittance ? "ON" : "OFF");
//                 break;

//             case '?':  // Print status
//                 Serial.println("\n=== System Status ===");
//                 Serial.print("Mode: "); 
//                 Serial.println(state.hold_position ? "HOLD" : "COMPLIANT");
//                 Serial.print("Position: "); Serial.println(state.pos_actual);
//                 Serial.print("Velocity: "); Serial.println(state.vel_actual);
//                 Serial.print("Target Vel: "); Serial.println(motor.target);
//                 Serial.print("Current: "); Serial.println(state.current_measured);
//                 Serial.print("External Torque: "); Serial.println(state.force_ext);
//                 Serial.print("M_d: "); Serial.println(admittance.M_d);
//                 Serial.print("B_d: "); Serial.println(admittance.B_d);
//                 Serial.print("K_d: "); Serial.println(admittance.K_d);
//                 Serial.println("==================\n");
//                 break;
//         }
//     }
// }



// // Emergency stop function
// void emergencyStop() {
//     motor.disable();
//     while(1) {
//         Serial.println("EMERGENCY STOP - Reset required");
//         delay(1000);
//     }
// }

// void loop() {

//     motor.loopFOC();

//     float raw_pos = sensor.getAngle();

//     raw_pos = fmod(raw_pos, 2*PI);
//     if (raw_pos < 0) raw_pos += 2*PI;

//     float filtered_pos = pos_median_filter.update(raw_pos);
//     state.pos_actual = position_filter(filtered_pos); 
    
//     // Calculate filtered velocity
//     state.vel_actual = vel_estimator.update(state.pos_actual);
    
//     updateAdmittanceControl();
    
//     if (motor.controller == MotionControlType::angle) {
//         // Use the admittance-controlled hold position target for position control
//         motor.target = state.hold_pos_target;
//         motor.move();
//     } 
//     else if (motor.controller == MotionControlType::velocity) {
//         // For velocity control, use velocity reference from admittance
//         motor.target = state.vel_ref;
//         motor.move();
//     }

//     // static int print_counter = 0;
//     // if (++print_counter >= 40) {
//     //     print_counter = 0;
//     //     Serial.print("Pfilt:"); Serial.print(state.pos_actual, 2);
//     //     Serial.print(" Praw:"); Serial.print(sensor.getAngle(), 2);
//     //     Serial.print(" V:"); Serial.print(motor.shaft_velocity, 2);
//     //     Serial.print(" I:"); Serial.println(motor.current.q, 2);
//     // }

//     // User communication
//     serialReceiveCommands();

//     // //motor.monitor();
    
//     command.run();
// }

