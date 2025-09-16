#include <SimpleFOC.h>

/**
 *
 * Motor gravity compensation example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) watch the motor armature with mass loaded - it should hold its position against gravity
 *
 */


// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5048_I2C);
// magnetic sensor instance - analog output
MagneticSensorPWM sensor = MagneticSensorPWM(2,3,909);// low
void doPWM() {
    sensor.handlePWM();
}
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11, 8.8f); // 7 pole pairs, 3.0 ohm phase resistance
BLDCDriver3PWM driver = BLDCDriver3PWM(3,5,6);//arduino uno
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

#define ARM_LENGTH 0.1f    // 100mm = 0.1m
#define LOAD_MASS 0.05f    // 20g = 0.02kg
#define GRAVITY 9.81f      // m/s^2
<<<<<<< HEAD
#define VOLTAGE_TO_TORQUE_RATIO 0.1983f  //TUNED [Nm/V] - adjust this based on your motor， theoretiacal: 0.0434/2.3
float compensate_angle = 1.95f + 0.166; // angle to be compensated [rad]
int mode = -1; //1 or -1 to change direction of compensation
float lift_ratio = 1.3f; // to slightly overcompensate gravity
=======
#define VOLTAGE_TO_TORQUE_RATIO 0.018870f  // [Nm/V] - adjust this based on your motor， 0.0434/2.3
>>>>>>> parent of 6f594c6 (got gravity compensation working, added comments too)

float voltageToTorque(float voltage) {
    return voltage * VOLTAGE_TO_TORQUE_RATIO;
}

float calculateRequiredTorque(float angle) {
    float torque = LOAD_MASS * GRAVITY * ARM_LENGTH * sin(angle);
    float max_torque = max_torque; // [Nm] set the maximum torqu
    return _constrain(torque, -max_torque/2, max_torque/2);  // Limit gravity compensation
}

float normalizeAngle(float angle) {
    while(angle > 2*PI) angle -= 2*PI;
    while(angle < -2*PI) angle += 2*PI;
    return angle;
}


// torque set point variable
float max_torque = 2.0; // maximum torque limit [Nm]
float current_torque = 0.0; // current torque value
float target_angle = 0.0; // target angle value
// instantiate the commander
Commander command = Commander(Serial);
<<<<<<< HEAD
//void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
=======
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
>>>>>>> parent of 6f594c6 (got gravity compensation working, added comments too)

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
<<<<<<< HEAD
  sensor.enableInterrupt(doPWM);
  motor.linkSensor(&sensor);
  Serial.print("Initial sensor angle: ");
=======
  Serial.print("Sensor angle: ");
>>>>>>> parent of 6f594c6 (got gravity compensation working, added comments too)
  Serial.println(sensor.getAngle());
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
<<<<<<< HEAD
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 0.0f;
  motor.PID_velocity.D = 0.3f;
=======
  motor.PID_velocity.P = 0.01f;
  motor.PID_velocity.I = 0.1f;
  motor.PID_velocity.D = 0.0f;
>>>>>>> parent of 6f594c6 (got gravity compensation working, added comments too)
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

  max_torque = VOLTAGE_TO_TORQUE_RATIO * motor.voltage_limit; // [Nm] set the maximum torque

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  compensate_angle = sensor.getAngle(); // initial angle to be compensated
  //command.add('M', doMotor, "motor");
  motor.zero_electric_angle = 1.96; // 2.21 cal. you can set it to a previously found value
  motor.sensor_direction = CCW; // you can set this to a previously found value

  // initialize motor
  motor.init();
  // align sensor and start FOC
  
  motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set

  // add target command T
<<<<<<< HEAD
  //command.add('T', doTarget, "target angle");
  //motor.useMonitoring(Serial);
  
=======
  command.add('T', doTarget, "target angle");
  motor.useMonitoring(Serial);

>>>>>>> parent of 6f594c6 (got gravity compensation working, added comments too)
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
  
<<<<<<< HEAD
  float current_angle = sensor.getAngle() - compensate_angle; // adjust this offset to set the "zero" position*
=======
  //Serial.println(current_torque);
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  //motor.move(target_angle);


  float current_angle = sensor.getAngle();
  
  // Calculate position error
  float angle_error = _normalizeAngle(target_angle - current_angle);
>>>>>>> parent of 6f594c6 (got gravity compensation working, added comments too)
  
  // Calculate gravity compensation torque
  float gravity_torque = calculateRequiredTorque(current_angle);
  
<<<<<<< HEAD
  // Calculate position error and correction torque
  float angle_error = current_angle - sensor.getAngle() - compensate_angle;
  
  // Simple proportional control for position (you can tune this)
  float position_gain = 0.05f;  // Adjust this value to tune responsiveness
  float position_torque = 0;//position_gain * angle_error;
  

  

  //Add damping (derivative term) to prevent oscillation
  // static float last_angle = current_angle;
  // static unsigned long last_t = micros();
  // unsigned long now = micros();
  // float dt = (now - last_t) * 1e-6f; // seconds
  // float velocity = (current_angle - last_angle) / dt;// Approximate velocity
  // last_t = now;
  // //
  // float damping_gain = 0.008f;  // Adjust this value to tune damping* not used currently as not working
  float damping_torque = 0;//-damping_gain * velocity;   //set to 0 for now to make it work
  // last_angle = current_angle;
  
  // Total torque = gravity compensation + position control + damping
  float total_torque = gravity_torque*lift_ratio + position_torque + damping_torque;
  
  // Limit the torque to safe values
  voltage_to_move = torqueToVoltage(total_torque);
  voltage_to_move = constrain(voltage_to_move, -motor.voltage_limit, motor.voltage_limit);

  //MAGIC 
  motor.move(mode * voltage_to_move); // apply the voltage command to the motor*

 
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();
  //Serial.print(current_angle);
  Serial.print("Angle: "); Serial.print(current_angle, 3);
  Serial.print(", Gravity_T: "); Serial.print(gravity_torque, 4);
  Serial.print(", Velocity: "); Serial.print(velocity, 4);
  Serial.print(", Damping_T: "); Serial.print(damping_torque, 4);
  Serial.print(", Total_T: "); Serial.print(total_torque, 4);
  Serial.print(", Voltage: "); Serial.println(voltage_to_move, 4);
=======
  // Calculate total torque (position correction + gravity compensation)
  float position_torque = motor.P_angle.P * angle_error;
  current_torque = position_torque + gravity_torque;
  
  // Limit the torque
  current_torque = _constrain(current_torque, -max_torque, max_torque);
  
  // Apply torque
  motor.move(current_torque);
  

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
  Serial.print(current_torque);
  //Serial.print(max_torque);
>>>>>>> parent of 6f594c6 (got gravity compensation working, added comments too)
  // user communication
  //command.run();
}
