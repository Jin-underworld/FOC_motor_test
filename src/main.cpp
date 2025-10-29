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

void setup() {
  
  // Enable GPIOA clock
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
  Serial2.begin(115200);
//   Serial2.begin(BAUD_RATE);
  delay(100);
  Serial2.println("Serial2 initialised!");
  SimpleFOCDebug::enable(&Serial2);
  
   
  //initialise sensors
  initializeI2CMultiplexer();
  initializeSensors(); 

  //initialise motors
  linkMotorsToSensors();
  initializeMotorDrivers();
  initializeMotorControllers();
  initializeMotorParameters();
  initializeMotors();
  if (TIM1->CR1 & TIM_CR1_CEN) {  // If TIM1 is enabled
      TIM1->BDTR |= TIM_BDTR_MOE;   // Enable Main Output
      TIM1->BDTR &= ~TIM_BDTR_BKE;  // Disable break
      Serial2.println("✓ TIM1 MOE force-enabled");
    }
  

  //command.add('M',doMotor,"motor");
  command.add('T', doTarget, "target angle");

  //Wait for button press with debounce
  Serial2.println("Waiting for button press to start homing sequence...");
  while (!buttonPressed) {
    // Optional: Add watchdog or timeout
    ButtonChecker();
    //printPose();
    // updateAllEncoders();
    //Serial2.print("Motorshaft: \n");
    //printRawMotorAngles();  //with sensor zero offset
    Serial2.print("Raw Mech ENC: \n");
    printAllMechEncoders(); //no sensor offset
    // Serial2.print("Current Angle: \n");
    // Serial2.println(getMotorRawAngle(Joint_motor[0]),3);
    // Serial2.println(getMotorRawAngle(Joint_motor[1]),3);
    // Serial2.println(getMotorRawAngle(Joint_motor[2]),3);
    delay(10); // Small delay to prevent tight loop
  }

  //updateAllEncoders();
  
//   if (checkstarting(gb6025_config)) {
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
  
  // goToHomePosition(gb6025_config);
  // goToHomePosition(gb6010_config);
  // goToHomePosition(gb4315_config);
  
  Serial2.println("Homing sequence completed. Motor is at home position.");
  ButtonReset(); // Reset button press flag

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
    
  
  
  
  //Serial2.print("heartbeat\n");
  ButtonChecker();
  if(buttonPressed&&useFakeImpedance){
    target_Angle_0 = GB6025motor.shaftAngle();
    target_Angle_1 = GB6010motor.shaftAngle();
    target_Angle_2 = GB4315motor.shaftAngle();

    //maybe add part to soften up all the controllers, then add else statement to flop the controller
    //to harder settings under normal situation, add single counter so that else statement doesnt always slow down loop
  }

  //standard position control
  digitalWrite(LED_BUILTIN, HIGH);
  Joint_motor[0]->loopFOC();
  Joint_motor[0]->move(target_Angle_0);

  Joint_motor[1]->loopFOC();
  Joint_motor[1]->move(target_Angle_1);

  Joint_motor[2]->loopFOC();
  Joint_motor[2]->move(target_Angle_2);
  digitalWrite(LED_BUILTIN, LOW);

  Joint_motor[3]->loopFOC();
  Joint_motor[3]->move(target_Angle_3);

  
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

  if(micros() - last_debug > 100) {  // Print every 10 miilisecond
    //prints pos endeffector in Joint_motor space for Unity controller(rate limited)
    //printPose();
    //printRawSensorAngles();
    printRawMotorAngles();
    //additional print
    //Jacobian J = calculateJacobian();
    //printJacobian(J);
    //checkJacobian(J);
    // Serial2.print("\n");
    Serial2.print("Current Angle: \n");
    Serial2.print("J0: \n");Serial2.println(target_Angle_0,3);
    Serial2.print("J1: \n");Serial2.println(target_Angle_1,3);
    Serial2.print("J2: \n");Serial2.println(target_Angle_2,3);
    Serial2.print("J3: \n");Serial2.println(target_Angle_3,3);
    last_debug = micros();
  }


  command.run();

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
      printPose();
      // Serial2.print("press button again to exit error loop.");
    }

    Serial2.println("Exiting error loop. Resuming operation.");
    printPose();
    sensor0.update();
    target_Angle_0 = getMotorRawAngle(Joint_motor[0]); // Set target to current position

    sensor1.update();
    target_Angle_1 = getMotorRawAngle(Joint_motor[1]); // Set target to current position
    sensor2.update();
    target_Angle_2 = getMotorRawAngle(Joint_motor[2]); // Set target to current position

    sensor3.update();
    target_Angle_3 = getMotorRawAngle(Joint_motor[3]); // Set target to current position

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
