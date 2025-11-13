#include "main.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include "motorHelper.h"
#include "sensorHelper.h"
#include "forwardKinematics.h"
#include "jacobianMatrix.h"
#include "impControl.h"

bool useFakeImpedance = false; //Set if you want to use the fake Impedance controller


//void doMotor(char *cmd) { command.motor(&GB4315motor, cmd); }

// Control State Variables
struct ControlState {
  float pos_ref = 0;          // Position reference [rad]
  float vel_ref = 0;          // Velocity reference [rad/s]
  float acc_ref = 0;          // Acceleration reference [rad/s^2]
  float pos_actual = 0;       // Actual position [rad]
  float vel_actual = 0;       // Actual velocity [rad/s]
  float pos_prev = 0;         // Previous position for velocity calculation
  float vel_prev = 0;         // Previous velocity for acceleration calculation
  float force_ext = 0;        // External force/torque [N*m]
  float current_measured = 0; // Measured motor current [A]
  float hold_pos_target = 0;
  bool hold_position = true;
};

// Force/Torque Estimation Parameters
//volatile float Kt = MOTOR_TORQUE_CONSTANT; // Motor torque constant [N*m/A] -
                                           // calibrate for your motor
const float current_threshold = 0.05; // Minimum current for force detection [A]

LowPassFilter position_filter =
    LowPassFilter(0.05); // Strong filtering for position
LowPassFilter velocity_filter =
    LowPassFilter(LowPassVelFilter); // time constant (roughly 10Hz cutoff)
LowPassFilter current_filter =
    LowPassFilter(LowPassCurrFilter); // time constant (roughly 50Hz cutoff)
LowPassFilter torque_filter =
    LowPassFilter(LowPassTorqueFilter); //  time constant for torque smoothing
LowPassFilter target_filter = LowPassFilter(0.005); // 5ms for smooth target

ControlState state;

volatile bool buttonPressed = false;

void ButtonReset (void) {
  buttonPressed = false;
}

void ButtonChecker (void) {
  buttonPressed = !HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);
}

extern "C" void __attribute__((weak)) HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_debounce_time = 0;
  uint32_t current_time = HAL_GetTick();
  
  if(GPIO_Pin == BUTTON_PIN && (current_time - last_debounce_time) > 200)
  {
    Serial2.println("Button Interrupt!");
    
  }
}


static char inbuf[64];
static uint8_t inlen = 0;
static volatile bool serial_command_flag = false;

static void blinkBlueTwice() {
  digitalWrite(LED_BLUE, HIGH); delay(50);
  digitalWrite(LED_BLUE, LOW);  delay(50);
  digitalWrite(LED_BLUE, HIGH); delay(50);
  digitalWrite(LED_BLUE, LOW);
}

static void processLine() {
  // trim trailing spaces/tabs
  while (inlen && (inbuf[inlen-1] == ' ' || inbuf[inlen-1] == '\t')) inlen--;
  inbuf[inlen] = '\0';

  if (inlen == 0) return; // ignore empty

  if (strcmp(inbuf, "0x01") == 0) {
    serial_command_flag = true;
    blinkBlueTwice();
    Serial2.println("Command received: Flag set to TRUE");

  } else if (strcmp(inbuf, "0x00") == 0 || strcmp(inbuf, "reset") == 0) {
    serial_command_flag = false;
    Serial2.println("Command received: Flag set to FALSE");

  } else {
    Serial2.print("Unknown command: "); Serial2.println(inbuf);

  }
  inlen = 0; // reset buffer
}

static void readFrom(Stream& s) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n' || c == '\r') {
      if (inlen) processLine(); // handle CR, LF, or CRLF
      else inlen = 0;           // eat consecutive line endings
    } else if (inlen + 1 < sizeof(inbuf)) {
      inbuf[inlen++] = c;
    } else {
      // overflow: reset buffer
      inlen = 0;
    }
  }
}



void setup() {
  
  // Enable GPIOC clock
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  // Enable SYSCFG clock
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  // Configure GPIO pin
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Try falling edge first
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);
  
  // Configure NVIC
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  pinMode(LED_BLUE, OUTPUT);
  Serial2.setTx(SERIAL2_TX);  // USART2_TX on PD5
  Serial2.setRx(SERIAL2_RX);  // USART2_RX on PD6
  Serial2.begin(BAUD_RATE);
  delay(100);
  Serial2.println("Serial2 initialised!");
  SimpleFOCDebug::enable(&Serial2);
  //command.add('T', doTarget, "set target (T=ang or T idx ang)");
  
   
  //initialise sensors
  initializeI2CMultiplexer();
  delay(1000);
  initializeSensors(); 

//   //initialise motors
//   linkMotorsToSensors();
//   initializeMotorDrivers();
//   initializeMotorControllers();
//   initializeMotorParameters();
//   initializeMotors();

  GB6025motor.linkSensor(&sensor0);
  GB6025driver.voltage_power_supply = driver_voltage_supply;
  GB6025driver.pwm_frequency = 20000;
  GB6025motor.linkDriver(&GB6025driver);
  GB6025driver.init();
  GB6025motor.controller = MotionControlType::angle;
  GB6025motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  GB6025motor.torque_controller = TorqueControlType::voltage;
  //GB6025 motor parameters
  //velocity PI controller parameters
  GB6025motor.PID_velocity.P = 0.59f;
  GB6025motor.PID_velocity.I = 8.9f;
  GB6025motor.PID_velocity.D = 0.001f;
  GB6025motor.LPF_velocity.Tf = 0.035; // filter for velocity, needs to be around 0.05s
  GB6025motor.P_angle.P = 0.2f;
  GB6025motor.P_angle.I = 0.05f;
  GB6025motor.P_angle.D = 0.001f;
  GB6025motor.P_angle.output_ramp = 5000;
  GB6025motor.LPF_angle.Tf = 0.0f;
  GB6025motor.velocity_limit = 4.0f;
  GB6025motor.enableFullRotationMode();
  GB6025motor.zero_electric_angle =JOINT0_ZE_ANGLE;
  GB6025motor.sensor_direction=    JOINT0_SENSOR_DIRECTION;
  GB6025motor.sensor_offset=       JOINT0_ZERO_CALIBRATED_ANGLE;
  GB6025motor.enable();
  GB6025motor.initFOC();


  GB6010motor.linkSensor(&sensor1);
  GB6010driver.voltage_power_supply = 24.0f;//driver_voltage_supply;
  GB6010driver.pwm_frequency = 20000;
  GB6010motor.linkDriver(&GB6010driver);
  GB6010driver.init();
  GB6010motor.controller = MotionControlType::torque;
  GB6010motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  GB6010motor.torque_controller = TorqueControlType::voltage;
  // M1_Current_sense.linkDriver(&GB6010driver);
  // M1_Current_sense.init();
  // GB6010motor.linkCurrentSense(&M1_Current_sense);

  //GB6010 motor parameters
  //velocity PI controller parameters
  GB6010motor.PID_velocity.P = 0.42f;
  GB6010motor.PID_velocity.I = 7.1f;
  GB6010motor.PID_velocity.D = 0.001f;
  GB6010motor.LPF_velocity.Tf = 0.035; // filter for velocity, needs to be around 0.05s
  GB6010motor.P_angle.P = 0.32f;
  GB6010motor.P_angle.I = 0.03f;
  GB6010motor.P_angle.D = 0.001f;
  GB6010motor.P_angle.output_ramp = 5000;
  GB6010motor.LPF_angle.Tf = 0.0f;
  GB6010motor.velocity_limit = 4.0f;
  GB6010motor.enableFullRotationMode();
  GB6010motor.zero_electric_angle =JOINT1_ZE_ANGLE;
  GB6010motor.sensor_direction=    JOINT1_SENSOR_DIRECTION;
  GB6010motor.sensor_offset=       JOINT1_ZERO_CALIBRATED_ANGLE;
  GB6010motor.current_limit= 2.1f;
  GB6010motor.voltage_limit = 17.0f;
  GB6010motor.PID_current_q.P = 1.2f;
  GB6010motor.PID_current_q.I = 6.2f;
  GB6010motor.PID_current_q.D = 0.0f;
  GB6010motor.PID_current_q.output_ramp = 500;
  GB6010motor.PID_current_d.P = 1.2;
  GB6010motor.PID_current_d.I = 6.2f;
  GB6010motor.PID_current_d.D = 0.0f;
  GB6010motor.PID_current_d.output_ramp = 500;

  GB6010motor.enable();
  GB6010motor.init();
  GB6010motor.initFOC();
  
  GB4315motor.linkSensor(&sensor2);
  GB4315driver.voltage_power_supply = driver_voltage_supply;
  GB4315driver.pwm_frequency = 20000;
  GB4315motor.linkDriver(&GB4315driver);
  GB4315driver.init();
  GB4315motor.controller = MotionControlType::torque;
  GB4315motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  GB4315motor.torque_controller = TorqueControlType::voltage;

  // GB4315 motor parameters
  //velocity PI controller parameters
  GB4315motor.PID_velocity.P = 0.33f;
  GB4315motor.PID_velocity.I = 1.2f;
  GB4315motor.PID_velocity.D = 0.001f;
  GB4315motor.LPF_velocity.Tf = 0.12f; // filter for velocity, needs to be around 0.05s
  GB4315motor.P_angle.P = 0.5f;
  GB4315motor.P_angle.I = 0.03f;
  GB4315motor.P_angle.D = 0.001f;
  GB4315motor.P_angle.output_ramp = 2000;
  GB4315motor.LPF_angle.Tf = 0.05f;
  //set limits
  GB4315motor.voltage_sensor_align = 3.0f;
  GB4315motor.voltage_limit = 7.0f;
  GB4315motor.velocity_limit = 2.0f;
  GB4315motor.enableFullRotationMode();
  GB4315motor.zero_electric_angle =JOINT2_ZE_ANGLE;
  GB4315motor.sensor_direction=    JOINT2_SENSOR_DIRECTION;
  GB4315motor.sensor_offset=       JOINT2_ZERO_CALIBRATED_ANGLE;
  GB4315motor.enable();
  GB4315motor.initFOC();

  GB3510_3_motor.linkSensor(&sensor3);
  GB3510_3_driver.voltage_power_supply = driver_voltage_supply;
  GB3510_3_driver.pwm_frequency = 20000;
  GB3510_3_motor.linkDriver(&GB3510_3_driver);
  GB3510_3_driver.init();
  GB3510_3_motor.controller = MotionControlType::torque;
  GB3510_3_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  GB3510_3_motor.torque_controller = TorqueControlType::voltage;

  // GB3510_3 motor parameters
  //velocity PI controller parameters
 // VELOCITY LOOP - Reduce gains for high inertia
  GB3510_3_motor.PID_velocity.P = 0.6f;      // Reduced from 1.5 (lower P = less aggressive)
  GB3510_3_motor.PID_velocity.I = 2.5f;      // Reduced from 6.5 (prevents integral windup)
  GB3510_3_motor.PID_velocity.D = 0.0f;      // Remove D term (adds noise with high inertia)
  GB3510_3_motor.LPF_velocity.Tf = 0.1f;    // Increased filtering (was 0.035)
  GB3510_3_motor.PID_velocity.output_ramp = 30.0f;  // Limit voltage rate of change
  
  // POSITION LOOP - Gentle position control for smooth motion
  GB3510_3_motor.P_angle.P = 0.05f;          // Reduced from 0.3 (softer position response)
  GB3510_3_motor.P_angle.I = 0.005f;           // Remove integral (was 0.02 - can cause overshoot)
  GB3510_3_motor.P_angle.D = 0.003f;        // Small D for damping oscillations
  GB3510_3_motor.P_angle.output_ramp = 300;  // MUCH lower ramp (was 2000 - too aggressive)
  GB3510_3_motor.LPF_angle.Tf = 0.2f;       // More angle filtering (was 0.05)
  
  GB3510_3_motor.velocity_limit = 1.5f;      // Reduce max velocity (was 3.0)
  GB3510_3_motor.voltage_limit = 8.0f;
  GB3510_3_motor.enableFullRotationMode();
  GB3510_3_motor.zero_electric_angle =JOINT3_ZE_ANGLE;
  GB3510_3_motor.sensor_direction=    JOINT3_SENSOR_DIRECTION;
  GB3510_3_motor.sensor_offset=       JOINT3_ZERO_CALIBRATED_ANGLE;
  GB3510_3_motor.enable();
  GB3510_3_motor.initFOC();

  


  //command.add('M',doMotor,"motor");

  //Wait for button press with debounce
  Serial2.println("Waiting for button press to start homing sequence...");
//   while (!buttonPressed) {
//     // Optional: Add watchdog or timeout
//     ButtonChecker();
//     //printPose();
//     // updateAllEncoders();
//     //Serial2.print("Motorshaft: \n");
//     //printRawMotorAngles();  //with sensor zero offset
//     Serial2.print("Raw Mech ENC: \n");
//     printAllEncoders();
//     //printAllMechEncoders(); //no sensor offset
//     // Serial2.print("Current Angle: \n");
//     // Serial2.println(getMotorRawAngle(Joint_motor[0]),3);
//     // Serial2.println(getMotorRawAngle(Joint_motor[1]),3);
//     // Serial2.println(getMotorRawAngle(Joint_motor[2]),3);
//     delay(10); // Small delay to prevent tight loop
//   }

  //updateAllEncoders();
  
//   if (checkstarting(GB6010_config)) {
//     Serial2.println("Enter Error Loop: Motor not in starting position!");
//     while(1); // Halt execution
//   }

//   if (checkstarting(gb6010_config)) {
//     Serial2.println("Enter Error Loop: Motor not in starting position!");
//     while(1); // Halt execution
//   }

//   if (checkstarting(gb4315_config)) {
//     Serial2.println("Enter Error Loop: GB4315 Motor not in starting position!");
//     while(1); // Halt execution
// }
//   // Execute homing sequence
  
//   delay(1000);
  Serial2.println("Starting homing sequence...");
  
  //goToHomePosition(gb6010_config,5000);
  // goToHomePosition(gb6010_config);
  // goToHomePosition(gb4315_config);
  
  Serial2.println("Homing sequence completed. Motor is at home position.");
  //ButtonReset(); // Reset button press flag

  //initializeImpControl();
  //For testing motors
  //Joint_motor[0]->disable();
  //Joint_motor[1]->disable();
  //Joint_motor[2]->disable();
  //Joint_motor[3]->disable();

  pinMode(LED_BUILTIN, OUTPUT);
}




float K_spring = 0.68f; //spring Constant (Nm/rad)
float D_damper = 0.19f; //Damping constant (Nm/(rad/s))
float K_spring_2 = 0.12f; //spring Constant (Nm/rad)
float D_damper_2 = 0.05f; //Damping constant (Nm/(rad/s))
float K_spring_3 = 0.00f; //spring Constant (Nm/rad) 0.001 if fail
float D_damper_3 = 0.035f; //Damping constant (Nm/(rad/s)) 0.003 if fail
float last_z = 0.0f;

LowPassFilter theta_1_filter = LowPassFilter(0.05f);
LowPassFilter dtheta_2_filter = LowPassFilter(0.014f);  // 
LowPassFilter theta_2_filter = LowPassFilter(0.005f);  // 
LowPassFilter dtheta_3_filter = LowPassFilter(0.06f);  // 
LowPassFilter theta_3_filter = LowPassFilter(0.04f);  // 

//motor 1 helpers for haptic feedback
//static bool     contact_1     = false;
static float    tau_skin_1    = 0.0f;        // Nm ramped 0→tau_skin_max
static uint32_t t_skin_prev_1 = 0;

const float angle_enter_1 = 0.65f;           // enter threshold [rad]
const float angle_exit_1  = 0.67f;           // exit threshold [rad] (hysteresis)
const float tau_skin_max  = 0.25f;           // target “pop-in” Nm
const float attack_rate   = 80.0f;           // Nm/s (fast rise)
const float release_rate  = 15.0f;           // Nm/s (slower fall)
const float depth_gain    = 0.01f;    // Nm per mm (tune)
const float ff_tau_limit  = 1.0f;            // absolute clamp Nm

//motor 2 helper for smoother x translation
const float e_db        = 0.15f;            // deadband half-width [rad]
const float e_smooth_2  = 0.60f;            // linear ramp width [rad] (make this as you like)
const float w_cap_2     = 20.0f;            // velocity cap [rad/s]
const float tau_max_2   = 0.7f;             // torque clamp [Nm]
const float theta_ref_2 = 2.63f;


//motor 3 helper for damper to prevent slop
static float tau_prev_3 = 0.0f;
static uint32_t t_prev_3 = 0;
const float tau_slew_3 = 0.5f; // Nm per second




static uint32_t loop_timeout = 0;

void loop() {

  readFrom(Serial2);

  // if (serial_command_flag) {
  //   // Do something when flag is true
  //   Serial2.println("Flag is TRUE - executing special behavior");
  //   // Your code here...
  // }
  

  loop_timeout = micros() - loop_timeout;
  if (loop_timeout>= 2000){
    restartI2CSystem();
  }

  
  // Joint_motor[0]->loopFOC();
  // Joint_motor[0]->move(target_Angle_0);

  
  Pose6D instant_pose = forwardKinematics();



  float angle1 = getMotorRawAngle(&GB6010motor);

  // Hysteresis contact detection
  // if (!contact_1 && angle1 <= angle_enter_1) contact_1 = true;
  // else if (contact_1 && angle1 > angle_exit_1) contact_1 = false;

  // Update surface reference only when NOT in contact
  if (!serial_command_flag) last_z = instant_pose.z;

  // Depth (positive penetration only)
  float current_depth = serial_command_flag ? (last_z - instant_pose.z) : 0.0f;
  if (current_depth < 0.0f) current_depth = 0.0f;

  // Ramp tau_skin_1 toward target
  uint32_t now_skin = micros();
  float dt_skin = (t_skin_prev_1 == 0) ? 0.001f : (now_skin - t_skin_prev_1) * 1e-6f;
  t_skin_prev_1 = now_skin;

  float tau_target = serial_command_flag ? tau_skin_max : 0.0f;
  float rate = (tau_target > tau_skin_1) ? attack_rate : release_rate;
  float max_delta1 = rate * dt_skin;
  float delta1 = tau_target - tau_skin_1;
  if (delta1 >  max_delta1) delta1 =  max_delta1;
  if (delta1 < -max_delta1) delta1 = -max_delta1;
  tau_skin_1 += delta1;

  // Combined force feedback torque (Nm)
  float force_feedback = tau_skin_1 + current_depth * depth_gain;
  force_feedback = constrain(force_feedback, -ff_tau_limit, ff_tau_limit);

  // --- Impedance control 6010, haptics also ---
  float dtheta = GB6010motor.shaftVelocity();
  float theta  = theta_1_filter(getMotorRawAngle(&GB6010motor));

  float torque_cmd =
      -K_spring * (theta - M_PI_2)
      -D_damper * dtheta
      + force_feedback;              // add haptic term

  torque_cmd /= 0.61f;               // scaling to motor command

  GB6010motor.loopFOC();
  GB6010motor.move(torque_cmd);//positive brings the joint 1 up to help player


  /////////
  ////4315 MOTOR START
  //////////



  float dtheta_2 = dtheta_2_filter(GB4315motor.shaftVelocity());
  float theta_2  = theta_2_filter(getMotorRawAngle(&GB4315motor));

  float e2 = -theta_ref_2 + theta_2;          // error

  // Smooth the 4315 impedance near 2.0 rad and slew-limit torque
  // Zone shaping to suppress the spring near the problematic sensor wrap
  const float zone_c_2      = 2.0f;   // center [rad]
  const float zone_half_2   = 0.06f;  // flat dead-zone half-width [rad]
  const float zone_ramp_2   = 0.20f;  // ramp width to full gain [rad]
  const float extra_damp_2  = 1.5f;   // extra damping multiplier inside zone

  float dx2 = theta_2 - zone_c_2;
  float a2  = fabsf(dx2) - zone_half_2;

  // kK in [0..1] scales spring; kD >=1 scales damping in the dead/ramp zone
  float kK = 1.0f;
  float kD = 1.0f;
  if (a2 <= 0.0f) {
    kK = 0.0f;                 // kill spring at the center band
    kD = 1.0f + extra_damp_2;  // boost damping to ride through the wrap
  } else if (a2 < zone_ramp_2) {
    float ramp = a2 / zone_ramp_2;   // 0→1
    kK = ramp;                       // spring ramps up
    kD = 1.0f + (1.0f - ramp) * extra_damp_2; // damping ramps down
  }

  // Impedance with zone-shaped gains
  float tau_spring_2 = -K_spring_2 * kK * e2;
  float tau_damp_2   = -D_damper_2 * kD * dtheta_2;
  float tau_cmd_2    = tau_spring_2 + tau_damp_2;

  // Clamp torque
  tau_cmd_2 = constrain(tau_cmd_2, -tau_max_2, tau_max_2);

  // Slew-limit torque to avoid sudden steps (Nm/s)
  static float    tau_prev_2 = 0.0f;
  static uint32_t t_prev_2   = 0;
  const float tau_slew_rate_2 = 1.5f; // Nm per second (tune)

  uint32_t now2 = micros();
  float dt2 = (t_prev_2 == 0) ? 0.001f : (now2 - t_prev_2) * 1e-6f;
  t_prev_2 = now2;

  float max_delta_tau = tau_slew_rate_2 * dt2;
  float d_tau = tau_cmd_2 - tau_prev_2;
  if (d_tau >  max_delta_tau) d_tau =  max_delta_tau;
  if (d_tau < -max_delta_tau) d_tau = -max_delta_tau;
  float tau_slewed_2 = tau_prev_2 + d_tau;
  tau_prev_2 = tau_slewed_2;

  // Scale to motor command (torque constant equivalent 0.47)
  float torque_cmd_4315 = tau_slewed_2 / 0.47f;

  GB4315motor.loopFOC();
  GB4315motor.move(torque_cmd_4315); // negative brings joint 2 in to help player

  ///////////
  ////3510////
  ///////////

  float dtheta_3 = dtheta_3_filter(GB3510_3_motor.shaftVelocity());
  float theta_3  = theta_3_filter(getMotorRawAngle(&GB3510_3_motor));
  const float theta_ref_3 = M_PI;

  // Impedance PD with clear sign
  float e3    = theta_ref_3 - theta_3;
  float torque_cmd_3510 = -K_spring_3 * e3
              - D_damper_3 * dtheta_3;


  
  float tau_raw_3 = torque_cmd_3510;

  //testing
  uint32_t now = micros();
  float dt = (t_prev_3 == 0) ? 0.0005f : (now - t_prev_3) * 1e-6f; // fallback if first run
  t_prev_3 = now;

  float max_delta = tau_slew_3 * dt;
  float delta = tau_raw_3 - tau_prev_3;
  if (delta >  max_delta) delta =  max_delta;
  if (delta < -max_delta) delta = -max_delta;
  float tau_slewed = tau_prev_3 + delta;
  tau_prev_3 = tau_slewed;

  torque_cmd_3510 = tau_slewed/0.2f;

  Joint_motor[3]->loopFOC();
  Joint_motor[3]->move(torque_cmd_3510);

  digitalWrite(LED_BUILTIN, LOW);

  //float* torque_imp = loopImpControl();

  //Impedance J0 to J2
//   for (int i = 0; i<3;i++ ){

//     float torque_cmd = torque_imp[i];
      
//     float current_cmd = torque_cmd/Joint_motor[i]->torque_constant;

//     float current_max = getMotorCharacteristics(i).current_limit;

//     current_cmd = constrain(current_cmd,-current_max,current_max);

//     Joint_motor[i]->loopFOC();
//     Joint_motor[i]->move(current_cmd);

//   }

    // float target_torque = 2.0f;

    // float current_cmd = target_torque/Joint_motor[1]->torque_constant;

    // Joint_motor[1]->loopFOC();

    // Joint_motor[1]->move(current_cmd);







  
  static uint32_t last_debug = 0;

  if(micros() - last_debug > 10000) {  // Print every 100 miilisecond
    //prints pos endeffector in Joint_motor space for Unity controller(rate limited)
    printPose(instant_pose);
    //printAllEncoders();
    //printAllMechAngles();
    //printAllJointAngles();
    //additional print
    //Jacobian J = calculateJacobian();
    //printJacobian(J);
    //checkJacobian(J);
    last_debug = micros();
  }

  // if (millis() - last_debug > 2) {
  //   //Serial2.print("Target torque: "); Serial2.print(torque_cmd,3);
  //   Serial2.print(", Motor Angle: "); Serial2.print(getMotorRawAngle(Joint_motor[2]),3);
  //   Serial2.print(", Variable: "); Serial2.print(e2,3);
  //   //Serial2.print(", Raw Sensor Angle: "); Serial2.print(sensor1.getAngle(),3);
  //   //Serial2.print(", Velocity: "); Serial2.print(Joint_motor[1]->shaftVelocity(),3);
  //   Serial2.print(", Motor Voltage Uq: "); Serial2.print(Joint_motor[2]->voltage.q,3);
  //   Serial2.print(", Motor Current Iq: "); Serial2.print(Joint_motor[2]->current.q,3);
  //   //Serial.print(", Current Command: "); Serial.print(current_cmd,3);
  //   Serial2.println();
  //   last_debug = millis();
  // }

  // command.run();

  ButtonChecker();
  if (buttonPressed && useFakeImpedance){
    delay(50); // Debounce delay
    
    Serial2.println("In homing mode");
    GB6025motor.controller = MotionControlType::angle;
    GB6025motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    GB6010motor.controller = MotionControlType::angle;
    GB6010motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    GB4315motor.controller = MotionControlType::angle;
    GB4315motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    GB3510_3_motor.controller = MotionControlType::angle;
    GB3510_3_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    while(buttonPressed){
      ButtonChecker();
 
      GB6025motor.loopFOC();
      GB6025motor.move(JOINT_CONFIGS[0].home_angle);
      GB6010motor.loopFOC();
      GB6010motor.move(JOINT_CONFIGS[1].home_angle);
      GB6010motor.loopFOC();
      GB6010motor.move(JOINT_CONFIGS[2].home_angle);
      GB6010motor.loopFOC();
      GB6010motor.move(JOINT_CONFIGS[3].home_angle);
    }
    Serial2.println("Exiting homing mode, returning to play");
      


  }
  if (buttonPressed && !useFakeImpedance) {
    delay(50); // Debounce delay
    
    Serial2.println("Button pressed! Entering error loop.");
    Joint_motor[0]->disable();
    Joint_motor[1]->disable();
    Joint_motor[2]->disable();
    Joint_motor[3]->disable();

    while (buttonPressed) {
      // Wait for button release
      ButtonChecker();
      delay(10);
      //printPose();
      // Serial2.print("press button again to exit error loop.");
    }

    Serial2.println("Exiting error loop. Resuming operation.");
    //printPose();
    sensor0.update();
    target_Angle_0 = Joint_motor[0]->shaftAngle(); // Set target to current position

    sensor1.update();
    target_Angle_1 = Joint_motor[1]->shaftAngle(); // Set target to current position

    sensor2.update();
    target_Angle_2 = Joint_motor[2]->shaftAngle();//->shaftAngle(); // Set target to current position

    sensor3.update();
    target_Angle_3 = Joint_motor[3]->shaftAngle(); // Set target to current position

    Joint_motor[0]->enable();
    Joint_motor[1]->enable();
    Joint_motor[2]->enable();
    Joint_motor[3]->enable();

  }

  // Serial2.println("\n=== I2C Scanner ===");
  // Serial2.println("Scanning main I2C bus...");
  // byte nDevices = 0;

  // delay(2000);

  // // Scan the main I2C bus first
  // for (byte address = 1; address < 127; address++) {
  //   Wire2.beginTransmission(address);
  //   byte error = Wire2.endTransmission();

  //   if (error == 0) {
  //     Serial2.print("I2C device found at address 0x");
  //     if (address < 16)
  //       Serial2.print("0");
  //     Serial2.print(address, HEX);
  //     Serial2.println();
  //     nDevices++;
  //   }
  // }

  // if (nDevices == 0) {
  //   Serial2.println("No I2C devices found on main bus");
  // }

  // // Now scan each multiplexer port
  // for (int port = 0; port < 8; port++) {
  //   Serial2.print("\nScanning multiplexer port ");
  //   Serial2.print(port);
  //   Serial2.println("...");

  //   multiplexer.selectPort(port);
  //   nDevices = 0;
  //   uint8_t *ret = multiplexer.scan(port);

  //   Serial2.println(ret[0], HEX);
  //   // Serial2.print("Addresses found: ");
  //   // for (int i = 0; ret[i] != 0; i++) { // Assuming the array is
  //   // null-terminated
  //   // }
  //   // Serial2.println();

  //   if (nDevices == 0) {
  //     Serial2.print("No I2C devices found on multiplexer port ");
  //     Serial2.println(port);
  //   }
  // }

  // Serial2.println("=== Scan Complete ===\n");

  // delay(5000); // Wait 5 seconds before next scan
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);
}
