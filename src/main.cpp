#include "main.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include "motor_helper.h"
#include "sensor_helper.h"

Commander command = Commander(Serial);
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

void readAllEncoders() {
  sensor1.update();
  sensor2.update();
  sensor3.update();
  // sensor4.update();
  // sensor5.update();
  // sensor6.update();

  Serial.print("Encoder 1: ");
  Serial.println(sensor1.getAngle(), 3);
  Serial.print("Encoder 2: ");
  Serial.println(sensor2.getAngle(), 3);
  Serial.print("Encoder 3: "); Serial.println(sensor3.getAngle(), 3);
  // Serial.print("Encoder 4: "); Serial.println(sensor4.getAngle(), 3);
  // Serial.print("Encoder 5: "); Serial.println(sensor5.getAngle(), 3);
  // Serial.print("Encoder 6: "); Serial.println(sensor6.getAngle(), 3);
}

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



#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_GPIO_PORT GPIOA

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
    Serial.println("Button Interrupt!");
    
  }
}



void setup() {
  

  // Enable GPIOA clock
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
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



  Serial.begin(BAUD_RATE);
  Serial.println("Hi");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  initializeSensors();

  SimpleFOCDebug::enable(&Serial);
  
  //initialise sensors
  initializeI2CMultiplexer();
  initializeSensors();
  SimpleFOCDebug::enable(&Serial);
  
  //initialise motors
  linkMotorsToSensors();
  initializeMotorDrivers();
  initializeMotorControllers();
  initializeMotorParameters();
  initializeMotors();

  //command.add('M',doMotor,"motor");
  command.add('T', doTarget, "target angle");
  GB6010motor.useMonitoring(Serial);

  //Wait for button press with debounce
  Serial.println("Waiting for button press to start homing sequence...");
  while (!buttonPressed) {
    // Optional: Add watchdog or timeout
    ButtonChecker();
    sensor2.update();
    sensor3.update();
    Serial.print("Current Angle: ");
    Serial.println(GB4315motor.shaftAngle());
    Serial.println(GB6010motor.shaftAngle());
    delay(10); // Small delay to prevent tight loop
  }
  
  // Update angle before checking
  sensor2.update();
  
  if (checkstarting(gb6010_config)) {
    Serial.println("Enter Error Loop: Motor not in starting position!");
    while(1); // Halt execution
  }

  if (checkstarting(gb4315_config)) {
    Serial.println("Enter Error Loop: GB4315 Motor not in starting position!");
    while(1); // Halt execution
}
  // Execute homing sequence
  
  delay(1000);
  Serial.println("Starting homing sequence...");
  
  goToHomePosition(gb6010_config);
  //delay(8000);
  //goToHomePosition(gb4315_config);
  
  Serial.println("Homing sequence completed. Motor is at home position.");
  ButtonReset(); // Reset button press flag

}

// // Function to receive parameter updates via Serial
// void serialReceiveCommands() {
//     if (Serial.available()) {
//         char cmd = Serial.read();
//         float value = Serial.parseFloat();

//         switch(cmd) {

//             case 'P':  // Set position reference
//                 state.pos_ref = constrain(value, -PI, PI);
//                 Serial.print("Position reference set to: ");
//                 Serial.println(state.pos_ref); break;

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
//                 Serial.print("Direct velocity set to: ");
//                 Serial.println(motor.target); break;

//             case '?':  // Print status
//                 Serial.println("\n=== System Status ===");
//                 Serial.print("Mode: ");
//                 Serial.println(state.hold_position ? "HOLD" : "COMPLIANT");
//                 Serial.print("Position: "); Serial.println(state.pos_actual);
//                 Serial.print("Velocity: "); Serial.println(state.vel_actual);
//                 Serial.print("Target Vel: "); Serial.println(motor.target);
//                 Serial.print("Current: ");
//                 Serial.println(state.current_measured);
//                 Serial.print("External Torque: ");
//                 Serial.println(state.force_ext);
//                 Serial.println("==================\n");
//                 break;
//         }
//     }
// }

void loop() {

  //readAllEncoders();
  
  // static uint32_t last_print = 0;
  // if(millis() - last_print > 1000) {
  //   Serial.print("Button state: ");
  //   Serial.println(HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN));
  //   Serial.print("Current target angle: ");
  //   Serial.println(target_Angle_1);
  //   last_print = millis();
  // }

  //Serial.print("PA0 state: ");
  //Serial.println(HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN));
  ////start try motor-Jin
  GB6010motor.loopFOC();
  GB6010motor.move(target_Angle_1);

  // Add GB4315 motor control
  GB4315motor.loopFOC();
  GB4315motor.move(target_Angle_2); 

  static uint32_t last_debug = 0;
  if(millis() - last_debug > 500) {  // Print every 500ms
    Serial.print("Target angle: ");
    Serial.print(target_Angle_1);
    Serial.print(" Current angle: ");
    Serial.println(GB6010motor.shaftAngle(), 3);
    last_debug = millis();
  }

  
  if(millis() - last_debug > 500) {  // Print every 500ms
    Serial.print("Target angle: ");
    Serial.print(target_Angle_2);
    Serial.print(" Current angle: ");
    Serial.println(GB6010motor.shaftAngle(), 3);
    last_debug = millis();
  }

  command.run();

  ButtonChecker();

  if (buttonPressed) {
    delay(50); // Debounce delay
    
    Serial.println("Button pressed! Entering error loop.");
    GB6010motor.disable();

    while (buttonPressed) {
      // Wait for button release
      ButtonChecker();
      delay(10);
      Serial.print("press button again to exit error loop.");
    }


    Serial.println("Exiting error loop. Resuming operation.");
    GB6010motor.enable();
    sensor2.update();
    target_Angle_1 = GB6010motor.shaftAngle(); // Set target to current position

    GB4315motor.enable();
    sensor3.update();
    target_Angle_2 = GB4315motor.shaftAngle(); // Set target to current position
   
    

  }



  // Serial.println("\n=== I2C Scanner ===");
  // Serial.println("Scanning main I2C bus...");
  // byte nDevices = 0;

  // delay(2000);

  // // Scan the main I2C bus first
  // for (byte address = 1; address < 127; address++) {
  //   Wire2.beginTransmission(address);
  //   byte error = Wire2.endTransmission();

  //   if (error == 0) {
  //     Serial.print("I2C device found at address 0x");
  //     if (address < 16)
  //       Serial.print("0");
  //     Serial.print(address, HEX);
  //     Serial.println();
  //     nDevices++;
  //   }
  // }

  // if (nDevices == 0) {
  //   Serial.println("No I2C devices found on main bus");
  // }

  // // Now scan each multiplexer port
  // for (int port = 0; port < 8; port++) {
  //   Serial.print("\nScanning multiplexer port ");
  //   Serial.print(port);
  //   Serial.println("...");

  //   multiplexer.selectPort(port);
  //   nDevices = 0;
  //   uint8_t *ret = multiplexer.scan(port);

  //   Serial.println(ret[0], HEX);
  //   // Serial.print("Addresses found: ");
  //   // for (int i = 0; ret[i] != 0; i++) { // Assuming the array is
  //   // null-terminated
  //   // }
  //   // Serial.println();

  //   if (nDevices == 0) {
  //     Serial.print("No I2C devices found on multiplexer port ");
  //     Serial.println(port);
  //   }
  // }

  // Serial.println("=== Scan Complete ===\n");

  // delay(5000); // Wait 5 seconds before next scan
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);
}
