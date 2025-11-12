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
volatile bool serial_command_flag = false;  // Flag that gets set by 0x01


void handleIncoming() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      inbuf[inlen] = '\0';
      if (inlen && inbuf[inlen - 1] == '\r') 
        inbuf[inlen - 1] = '\0';

      // Check for 0x01 command
      if (strcmp(inbuf, "0x01") == 0) {
        serial_command_flag = true;  // Set flag to true
        Serial2.println("Command received: Flag set to TRUE");
        digitalWrite(LED_BLUE, HIGH);
        delay(50);
        digitalWrite(LED_BLUE, LOW);
        delay(50);
        digitalWrite(LED_BLUE, HIGH);
        delay(50);
        digitalWrite(LED_BLUE, LOW);
      }
      // Check for reset/clear command
      else if (strcmp(inbuf, "0x00") == 0 || strcmp(inbuf, "reset") == 0) {
        serial_command_flag = false;  // Set flag to false
        Serial2.println("Command received: Flag set to FALSE");
      }
      // Echo unknown commands
      else if (inlen > 0) {
        Serial2.print("Unknown command: ");
        Serial2.println(inbuf);
      }

      inlen = 0;  // Reset buffer
    }
    else if (c != '\r' && inlen + 1 < sizeof(inbuf)) {
      inbuf[inlen++] = c;
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
  GB6010motor.current_limit= 1.8f;
  GB6010motor.voltage_limit = 13.0f;
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




float K_spring = 0.71f; //spring Constant (Nm/rad)
float D_damper = 0.19f; //Damping constant (Nm/(rad/s))
float K_spring_2 = 0.3f; //spring Constant (Nm/rad)
float D_damper_2 = 0.16f; //Damping constant (Nm/(rad/s))
float K_spring_3 = 0.001f; //spring Constant (Nm/rad)
float D_damper_3 = 0.003f; //Damping constant (Nm/(rad/s))
float last_z = 0.0f;

LowPassFilter theta_1_filter = LowPassFilter(0.1f);
LowPassFilter dtheta_2_filter = LowPassFilter(0.2f);  // 
LowPassFilter theta_2_filter = LowPassFilter(0.2f);  // 
LowPassFilter dtheta_3_filter = LowPassFilter(0.04f);  // 
LowPassFilter theta_3_filter = LowPassFilter(0.04f);  // 


static const float tau_max_3 = 0.8f;     // max virtual torque [Nm]
static const float tau_slew_3 = 1.5f;     // Nm/s limit on torque change
static const float tau_fric_3 = 0.01f;    // Coulomb friction compensation [Nm]


static uint32_t loop_timeout = 0;

void loop() {

  handleIncoming(); //SERIAL input from unity

  if (serial_command_flag) {
    // Do something when flag is true
    Serial2.println("Flag is TRUE - executing special behavior");
    // Your code here...
  }
  

  loop_timeout = micros() - loop_timeout;
  if (loop_timeout>= 2000){
    restartI2CSystem();
  }
  //Target angle constrains
//   float target_Angle_0 = constrain(target_Angle_0,JOINT0_LOWER_LIMIT,JOINT0_UPPER_LIMIT);
//   float target_Angle_1 = constrain(target_Angle_1,JOINT1_LOWER_LIMIT,JOINT1_UPPER_LIMIT);
//   float target_Angle_2 = constrain(target_Angle_2,JOINT2_LOWER_LIMIT,JOINT2_UPPER_LIMIT);
//   float target_Angle_3 = constrain(target_Angle_3,JOINT3_LOWER_LIMIT,JOINT3_UPPER_LIMIT);
  
  
//   ButtonChecker();
//   if(buttonPressed&&useFakeImpedance){
//     target_Angle_0 = GB6010motor.shaftAngle();
//     target_Angle_1 = GB6010motor.shaftAngle();
//     target_Angle_2 = GB4315motor.shaftAngle();

//     //maybe add part to soften up all the controllers, then add else statement to flop the controller
//     //to harder settings under normal situation, add single counter so that else statement doesnt always slow down loop
//   }

//   //standard position control
//   digitalWrite(LED_BUILTIN, HIGH);
  float force_feedback = 0.0f;

  // Joint_motor[0]->loopFOC();
  // Joint_motor[0]->move(target_Angle_0);


  //pseudo force feedback
  if (serial_command_flag){
    float current_depth = last_z - forwardKinematics().z;
    force_feedback = current_depth * 0.3f/5.0f;//current over total allowavble depth
  }
  else{
    last_z = forwardKinematics().z;
  }

  //really fake force feedback, take away once serial working
  if (getMotorRawAngle(&GB6010motor)<=0.65f){
    force_feedback = 0.7f; //(0.65f-getMotorRawAngle(&GB6010motor))+0.1f;
  }
  else{
    force_feedback = 0.0f;
  }


  //Impedance control Master
  float dtheta =GB6010motor.shaftVelocity();
  float theta = theta_1_filter(getMotorRawAngle(&GB6010motor));

  float torque_cmd = 
      -K_spring * (theta-M_PI_2)
      -D_damper * dtheta;

  torque_cmd = torque_cmd/0.61f;

  GB6010motor.loopFOC();
  GB6010motor.move(torque_cmd);//positive brings the joint 1 up to help player


  float dtheta_2 =dtheta_2_filter(GB4315motor.shaftVelocity());
  float theta_2 = theta_2_filter(getMotorRawAngle(&GB4315motor));

  float torque_cmd_4315 = 
      -K_spring_2 * -(2.1f - theta_2)
      -D_damper_2 * dtheta_2;

  torque_cmd_4315 = torque_cmd_4315/0.47f;

  Joint_motor[2]->loopFOC();
  Joint_motor[2]->move(torque_cmd_4315);//negative brings the joint 2 in to help player

  float dtheta_3 = dtheta_3_filter(GB3510_3_motor.shaftVelocity());
  float theta_3  = theta_3_filter(getMotorRawAngle(&GB3510_3_motor));
  const float theta_ref_3 = M_PI;

  // Impedance PD with clear sign
  float e3    = theta_ref_3 - theta_3;
  float torque_cmd_3510 = -K_spring_3 * e3
              - D_damper_3 * dtheta_3;


  torque_cmd_3510 = torque_cmd_3510/0.2f;
 

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
    printPose();
    //printAllEncoders();
    //printAllMechAngles();
    //printAllJointAngles();
    
    //Serial2.print("Actual Angle: \n");
    //Serial2.println(GB6025motor.shaftAngle(),3);
    //Serial2.println(GB6025motor.shaftVelocity(),3);
    // Serial2.println(GB6010motor.shaftAngle(),3);
    // Serial2.println(GB6010motor.shaftVelocity(),3);
    // Serial2.println(GB4315motor.shaftAngle(),3);
    // Serial2.println(GB4315motor.shaftVelocity(),3);
    //Serial2.println(GB3510_3_motor.shaftAngle(),3);
    //Serial2.println(GB3510_3_motor.shaftVelocity(),3);
    //additional print
    //Jacobian J = calculateJacobian();
    //printJacobian(J);
    //checkJacobian(J);
    // Serial2.print("\n");
    //Serial2.print("Current Targets: \n");
    //Serial2.print("J0:");Serial2.println(target_Angle_0,3);
    //Serial2.print("J1:");Serial2.println(target_Angle_1,3);
    //Serial2.print("J2:");Serial2.println(target_Angle_2,3);
    //Serial2.print("J3:");Serial2.println(target_Angle_3,3);
    last_debug = micros();
  }

  // if (millis() - last_debug > 20) {
  //   //Serial2.print("Target torque: "); Serial2.print(torque_cmd,3);
  //   Serial2.print(", Motor Angle: "); Serial2.print(getMotorRawAngle(Joint_motor[3]),3);
  //   //Serial2.print(", J5 angle: "); Serial2.print(getAllMechAngles(5),3);
  //   //Serial2.print(", Raw Sensor Angle: "); Serial2.print(sensor1.getAngle(),3);
  //   //Serial2.print(", Velocity: "); Serial2.print(Joint_motor[1]->shaftVelocity(),3);
  //   Serial2.print(", Motor Voltage Uq: "); Serial2.print(Joint_motor[3]->voltage.q,3);
  //   Serial2.print(", Motor Current Iq: "); Serial2.print(Joint_motor[3]->current.q,3);
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
