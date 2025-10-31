#include "main.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include "motorHelper.h"
#include "sensorHelper.h"
#include "forwardKinematics.h"
#include "jacobianMatrix.h"
#include "impControl.h"

bool useFakeImpedance = false; //Set if you want to use the fake Impedance controller

Commander command = Commander(Serial2);

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

// Commander callback: T [idx angle] or T=angle (defaults to joint 1)
void doTarget(char* cmd) {command.scalar(&target_Angle_1,cmd);}


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


  Serial2.setTx(PD5);  // USART2_TX on PD5
  Serial2.setRx(PD6);  // USART2_RX on PD6
  Serial2.begin(BAUD_RATE);
  delay(100);
  Serial2.println("Serial2 initialised!");
  SimpleFOCDebug::enable(&Serial2);
  //command.add('T', doTarget, "set target (T=ang or T idx ang)");
  
   
  //initialise sensors
  initializeI2CMultiplexer();
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
  // GB6025 motor parameters
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
  GB6025motor.enableMechanicalMode();
  GB6025motor.zero_electric_angle =JOINT0_ZE_ANGLE;
  GB6025motor.sensor_direction=    JOINT0_SENSOR_DIRECTION;
  GB6025motor.sensor_offset=       JOINT0_ZERO_CALIBRATED_ANGLE;
  GB6025motor.enable();
  GB6025motor.initFOC();


  GB6010motor.linkSensor(&sensor1);
  GB6010driver.voltage_power_supply = driver_voltage_supply;
  GB6010driver.pwm_frequency = 20000;
  GB6010motor.linkDriver(&GB6010driver);
  GB6010driver.init();
  GB6010motor.controller = MotionControlType::angle;
  GB6010motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // GB6010 motor parameters
  //velocity PI controller parameters
  GB6010motor.PID_velocity.P = 0.59f;
  GB6010motor.PID_velocity.I = 8.9f;
  GB6010motor.PID_velocity.D = 0.001f;
  GB6010motor.LPF_velocity.Tf = 0.035; // filter for velocity, needs to be around 0.05s
  GB6010motor.P_angle.P = 0.2f;
  GB6010motor.P_angle.I = 0.05f;
  GB6010motor.P_angle.D = 0.001f;
  GB6010motor.P_angle.output_ramp = 5000;
  GB6010motor.LPF_angle.Tf = 0.0f;
  GB6010motor.velocity_limit = 4.0f;
  GB6010motor.enableMechanicalMode();
  GB6010motor.zero_electric_angle =JOINT1_ZE_ANGLE;
  GB6010motor.sensor_direction=    JOINT1_SENSOR_DIRECTION;
  GB6010motor.sensor_offset=       JOINT1_ZERO_CALIBRATED_ANGLE;
  GB6010motor.enable();
  GB6010motor.initFOC();
  
  GB4315motor.linkSensor(&sensor2);
  GB4315driver.voltage_power_supply = driver_voltage_supply;
  GB4315driver.pwm_frequency = 20000;
  GB4315motor.linkDriver(&GB4315driver);
  GB4315driver.init();
  GB4315motor.controller = MotionControlType::angle;
  GB4315motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // GB4315 motor parameters
  //velocity PI controller parameters
  GB4315motor.PID_velocity.P = 0.33f;
  GB4315motor.PID_velocity.I = 1.2f;
  GB4315motor.PID_velocity.D = 0.001f;
  GB4315motor.LPF_velocity.Tf = 0.12f; // filter for velocity, needs to be around 0.05s
  GB4315motor.P_angle.P = 0.18f;
  GB4315motor.P_angle.I = 0.0f;
  GB4315motor.P_angle.D = 0.001f;
  GB4315motor.P_angle.output_ramp = 2000;
  GB4315motor.LPF_angle.Tf = 0.05f;
  GB4315motor.velocity_limit = 2.0f;
  GB4315motor.enableMechanicalMode();
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
  GB3510_3_motor.controller = MotionControlType::velocity;
  GB3510_3_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // GB4315 motor parameters
  //velocity PI controller parameters
  GB3510_3_motor.PID_velocity.P = 0.47f;
  GB3510_3_motor.PID_velocity.I = 9.2f;
  GB3510_3_motor.PID_velocity.D = 0.001f;
  GB3510_3_motor.LPF_velocity.Tf = 0.035f; // filter for velocity
  GB3510_3_motor.P_angle.P = 0.18f;
  GB3510_3_motor.P_angle.I = 0.0f;
  GB3510_3_motor.P_angle.D = 0.001f;
  GB3510_3_motor.P_angle.output_ramp = 2000;
  GB3510_3_motor.LPF_angle.Tf = 0.05f;
  GB3510_3_motor.velocity_limit = 3.0f;
  GB3510_3_motor.enableFullRotationMode();
  GB3510_3_motor.zero_electric_angle =JOINT3_ZE_ANGLE;
  GB3510_3_motor.sensor_direction=    JOINT3_SENSOR_DIRECTION;
  GB3510_3_motor.sensor_offset=       JOINT3_ZERO_CALIBRATED_ANGLE;
  GB3510_3_motor.voltage_limit =      3.0f;
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
  
  // goToHomePosition(GB6010_config);
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

// float K_spring = 2.0f; //spring Constant (Nm/rad)
// float D_damper = 0.7f; //Damping constant (Nm/(rad/s))


void loop() {
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

//   Joint_motor[0]->loopFOC();
//   Joint_motor[0]->move(target_Angle_0);

//   GB6010motor.loopFOC();
//   GB6010motor.move(target_Angle_1);

//   Joint_motor[2]->loopFOC();
//   Joint_motor[2]->move(target_Angle_2);//target_Angle_2
 

  Joint_motor[3]->loopFOC();
  Joint_motor[3]->move(1.0f);

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


  // //test branch: impedance control only on 6010motor, used as reference for full implementation
  // float dtheta =GB6010motor.shaft_velocity;
  // float theta = GB6010motor.shaft_angle;

  // float torque_cmd = 
  //     -K_spring * (theta - target_Angle_1)
  //     -D_damper * dtheta;

  // torque_cmd = constrain(torque_cmd,-0.9f,2.0f);
  // torque_cmd = torque_cmd/0.61f;

  
  static uint32_t last_debug = 0;

  if(micros() - last_debug > 10000) {  // Print every 10 miilisecond
    //prints pos endeffector in Joint_motor space for Unity controller(rate limited)
    //printPose();
    //printRawSensorAngles();
    //printRawMotorAngles();
    //Serial2.print("Actual Angle: \n");
    //Serial2.println(GB6025motor.shaftAngle(),3);
    //Serial2.println(GB6025motor.shaftVelocity(),3);
    // Serial2.println(GB6010motor.shaftAngle(),3);
    // Serial2.println(GB6010motor.shaftVelocity(),3);
    // Serial2.println(GB4315motor.shaftAngle(),3);
    // Serial2.println(GB4315motor.shaftVelocity(),3);
    Serial2.println(GB3510_3_motor.shaftAngle(),3);
    Serial2.println(GB3510_3_motor.shaftVelocity(),3);
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


  command.run();

  ButtonChecker();
  if (buttonPressed && !useFakeImpedance) {
    delay(50); // Debounce delay
    
    Serial2.println("Button pressed! Entering error loop.");
    // Joint_motor[0]->disable();
    // Joint_motor[1]->disable();
    // Joint_motor[2]->disable();
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

    // sensor3.update();
    // target_Angle_3 = getMotorRawAngle(Joint_motor[3]); // Set target to current position

    // Joint_motor[0]->enable();
    // Joint_motor[1]->enable();
    // Joint_motor[2]->enable();
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
