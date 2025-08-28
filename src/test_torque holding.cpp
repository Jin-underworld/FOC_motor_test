#include <SimpleFOC.h>

/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */


// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7, 3.0f); // 7 pole pairs, 3.0 ohm phase resistance
BLDCDriver3PWM driver = BLDCDriver3PWM(3,5,6,8);//arduino uno
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

#define ARM_LENGTH 0.1f    // 100mm = 0.1m
#define LOAD_MASS 0.02f    // 20g = 0.02kg
#define GRAVITY 9.81f      // m/s^2
#define VOLTAGE_TO_TORQUE_RATIO 0.0260f  //TUNED [Nm/V] - adjust this based on your motor， theoretiacal: 0.0434/2.3

float calculateGravityTorque(float angle) {
    // Torque = mass * gravity * arm_length * sin(angle)
    // This gives the torque needed to counteract gravity
    float gravity_torque = LOAD_MASS * GRAVITY * ARM_LENGTH * sin(angle);
    return gravity_torque;
}

float torqueToVoltage(float torque) {
    return torque / VOLTAGE_TO_TORQUE_RATIO;
}

// torque set point variable
float max_torque = 2.0; // maximum torque limit [Nm]
float current_torque = 0.0; // current torque value
float voltage_to_move = 0.0; // target angle value
// instantiate the commander
Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }


void setup() {
  Serial.begin(115200);
  Serial.println("Starting gravity compensation setup...");
  // initialise magnetic sensor hardware
  sensor.init();
  motor.linkSensor(&sensor);
  Serial.print("Initial sensor angle: ");
  Serial.println(sensor.getAngle());
  // driver config
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage; //changed to voltage control*
  motor.controller = MotionControlType::torque; //changed to torque control*

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.04f;
  motor.PID_velocity.I = 0.01f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000; // acceleration [rad/s^2]
  //motor.LPF_velocity.Tf = 0.01; // filter for velocity

  // maximal voltage to be set to the motor
  motor.voltage_limit = 12;
  
  // velocity low pass filtering time constant
  // the lower the less filtered
  //motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 2.0f;
  // maximal velocity of the position control
  motor.velocity_limit = 20;

  max_torque = VOLTAGE_TO_TORQUE_RATIO * motor.voltage_limit; // [Nm] set the maximum torque*

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);


  motor.zero_electric_angle = 2.21; // 2.21 cal. you can set it to a previously found value
  motor.sensor_direction = CCW; // you can set this to a previously found value

  // initialize motor
  motor.init();
  // align sensor and start FOC
  
  motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set

  // add target command T
  //command.add('T', doTarget, "target angle");
  //motor.useMonitoring(Serial);

  Serial.println(F("Motor ready."));
  //Serial.println(F("Set the target angle using serial terminal:"));
  Serial.println("Starting...");
  _delay(1000);

}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  
  float current_angle = sensor.getAngle() - 2.54f; // adjust this offset to set the "zero" position*
  
  // Calculate gravity compensation torque
  float gravity_torque = calculateGravityTorque(current_angle);
  
  // Calculate position error and correction torque
  float angle_error = 0;
  
  // Simple proportional control for position (you can tune this)
  float position_gain = 0.5f;  // Adjust this value to tune responsiveness
  float position_torque = position_gain * angle_error;
  
  // Add damping (derivative term) to prevent oscillation
  static float last_angle = current_angle;
  float velocity = (current_angle - last_angle) * 1000.0f; // Approximate velocity
  float damping_gain = 0.01f;  // Adjust this value to tune damping* not used currently as not working
  float damping_torque = 0;   //set to 0 for now to make it work
  last_angle = current_angle;
  
  // Total torque = gravity compensation + position control + damping
  float total_torque = gravity_torque + position_torque + damping_torque;
  
  // Limit the torque to safe values
  float voltage_command = torqueToVoltage(total_torque);
  voltage_command = constrain(voltage_command, -motor.voltage_limit, motor.voltage_limit);

  //MAGIC HAPPENS HERE
  // Apply the voltage command to the motor
  motor.move(-voltage_command);
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();
  //Serial.print(current_angle);
  Serial.print("Angle: "); Serial.print(current_angle, 3);
  Serial.print(", Gravity_T: "); Serial.print(gravity_torque, 4);
  Serial.print(", Velocity: "); Serial.print(velocity, 4);
  Serial.print(", Damping_T: "); Serial.print(damping_torque, 4);
  Serial.print(", Total_T: "); Serial.print(total_torque, 4);
  Serial.print(", Voltage: "); Serial.println(voltage_command, 4);
  // user communication
  command.run();
}
