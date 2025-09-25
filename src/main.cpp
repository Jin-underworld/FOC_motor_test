// #include <SimpleFOC.h>

// //help me package this somewhere
// #define motor_GB4315_PP 14
// #define motor_GB4315_R 13.84
// #define motor_GB4315_ZEAngle 5.57
// #define motor_GB4315_sensor_direction Direction::CW
// #define motor_GB4315_ZeroCalibratedAngle -1.784f
// #define motor_GB4315_HomeAngle 1.6f  // Define the home position angle (in radians)




// /**
//  *
//  * Position/angle motion control example
//  * Steps:
//  * 1) Configure the motor and magnetic sensor
//  * 2) Run the code
//  * 3) Set the target angle (in radians) from serial terminal
//  *
//  */
// class MedianFilter {
// private:
//     static const int SIZE = 7;
//     float values[SIZE];
//     int index = 0;
//     bool buffer_initialized = false;
    
// public:
//     MedianFilter() {

//         for (int i = 0; i < SIZE; i++) {
//             values[i] = 0;
//         }
//     }
    
//     float update(float new_value) {
//         // Store new value
//         values[index] = new_value;
//         index = (index + 1) % SIZE;
        
//         if (!buffer_initialized && index == 0) {
//             buffer_initialized = true;
//         }
        
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
    
//     // Add a method to initialize the buffer with a specific value
//     void init(float value) {
//         for (int i = 0; i < SIZE; i++) {
//             values[i] = value;
//         }
//         buffer_initialized = true;
//     }
// };


// class FilteredSensor: public Sensor {
//   public:
//     FilteredSensor(MagneticSensorPWM& _sensor) : sensor(_sensor) {
//     }
    
//     // Update the sensor and filter the angle
//     void update() override {
//       sensor.update();
//       float new_angle = sensor.getAngle();
      
//       // Use the median filter to update and get the filtered angle
//       filtered_angle = filter.update(new_angle);
//     }
    
//     // Override getAngle to return the filtered angle
//     float getAngle() override {
//       return filtered_angle;
//     }
    
//     // Pass through velocity calculation to the original sensor
//     float getVelocity() override {
//       return sensor.getVelocity();
//     }
    
//     // Enable PWM interrupt handling
//     void enableInterrupt(void (*doPWM)(void)) {
//       sensor.enableInterrupt(doPWM);
//     }
    
//     // Initialize the wrapped sensor
//     void init() {
//       sensor.init();
//       // Initialize the filter with the first sensor reading
//       float initial_angle = sensor.getAngle();
//       filter.init(initial_angle);
//       filtered_angle = initial_angle;
//     }
    
//     // Get the raw angle for debugging
//     float getRawAngle() {
//       return sensor.getAngle();
//     }

//     float getSensorAngle() override {
//       return sensor.getSensorAngle();
//     }
    
//   private:
//     MagneticSensorPWM& sensor;
//     MedianFilter filter;
//     float filtered_angle = 0;
// };



// // magnetic sensor instance - SPI
// //MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// // magnetic sensor instance - MagneticSensorI2C
// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// // magnetic sensor instance - analog output
// // MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);
// //MagneticSensorPWM sensor = MagneticSensorPWM(2, 3, 945);// low

// //FilteredSensor filteredSensor = FilteredSensor(sensor);

// // BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(motor_GB4315_PP);
// BLDCDriver3PWM driver = BLDCDriver3PWM(3,5,6,8);//arduino uno
// // Stepper motor & driver instance
// //StepperMotor motor = StepperMotor(50);
// //StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// // angle set point variable
// float target_Angle = motor_GB4315_HomeAngle; // target angle value

// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_Angle, cmd); }



// float augmented_angle = 0.0f;

// float getAugmentedAngle(float Actual_angle) {
//     augmented_angle = Actual_angle + motor_GB4315_ZeroCalibratedAngle;
//     // if (augmented_angle < 0) {
//     //     augmented_angle += 2 * M_PI; // Wrap around to [0, 2π]
//     // }
//     // else if (augmented_angle >= 2 * M_PI) {
//     //     augmented_angle -= 2 * M_PI; // Wrap around to [0, 2π]
//     // }
//     return augmented_angle;
// }

// float DisplayAngle(float Actual_angle) {
//     float display_angle = Actual_angle - motor_GB4315_ZeroCalibratedAngle;
//     // if (display_angle < 0) {
//     //     display_angle += 2 * M_PI; // Wrap around to [0, 2π]
//     // }
//     // else if (display_angle >= 2 * M_PI) {
//     //     display_angle -= 2 * M_PI; // Wrap around to [0, 2π]
//     // }
//     return display_angle;
// }

// bool checkstarting() {
//   sensor.update();
//   float CurrentAngle = DisplayAngle(sensor.getAngle());
//   Serial.print("Current Angle: ");
//   Serial.println(CurrentAngle);
//   if (CurrentAngle >= M_PI or CurrentAngle < M_PI/2) {
//     Serial.println("Error: Motor not in starting position. Please move the motor to the home position (0 radians).");
//     return 1;
//   }
//   else {
//     Serial.println("Motor in starting position. Proceeding with homing sequence.");
//     return 0;
//   }
// }

// bool goToHomePosition(unsigned long timeout_ms = 5000) {
//     unsigned long start_time = millis();
//     motor.target = getAugmentedAngle(motor_GB4315_HomeAngle);
    
//     while (fabs(DisplayAngle(sensor.getAngle()) - DisplayAngle(motor_GB4315_HomeAngle) > 0.01f)){
//         motor.loopFOC();
//         motor.move();
        
//         // Add timeout
//         if (millis() - start_time > timeout_ms) {
//             Serial.println("Homing sequence timed out!");
//             return false;
//         }
        
//         // Optional: Add progress feedback
//         if (millis() % 500 == 0) {
//             Serial.print("Homing... Current angle: ");
//             Serial.println(DisplayAngle(sensor.getAngle()));
//         }
//     }
//     return true;
// }

// // Must be volatile to share with ISR
// volatile bool buttonPressed = false;  
// const unsigned long DEBOUNCE_DELAY = 50; // milliseconds

// void onButtonISR() {
//     static unsigned long lastInterruptTime = 0;
//     unsigned long interruptTime = millis();
//     if (interruptTime - lastInterruptTime > DEBOUNCE_DELAY) {
//         buttonPressed = true;
//         lastInterruptTime = interruptTime;
//     }
// }



// void setup() {

//   // initialise magnetic sensor hardware

 
//   sensor.init();
//   //sensor.enableInterrupt(doPWM);
//   Serial.print("Sensor angle: ");
//   Serial.println(DisplayAngle(sensor.getAngle()));
//   // link the motor to the sensor
//   motor.linkSensor(&sensor);
//   // driver config
//   // power supply voltage [V]
//   driver.voltage_power_supply = 24;
//   driver.init();
//   // link the motor and the driver
//   motor.linkDriver(&driver);

//   // choose FOC modulation (optional)
//   motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

//   // set motion control loop to be used
//   motor.controller = MotionControlType::angle;

//   // contoller configuration
//   // default parameters in defaults.h

//   // velocity PI controller parameters
//   motor.PID_velocity.P = 0.5f;
//   motor.PID_velocity.I = 10.0f;
//   motor.PID_velocity.D = 0.0f;
//   //motor.LPF_velocity.Tf = 0.01; // filter for velocity

//   // maximal voltage to be set to the motor
//   motor.voltage_limit = 24.0f;

//   // velocity low pass filtering time constant
//   // the lower the less filtered
//   motor.LPF_velocity.Tf = 0.0f;

//   // angle P controller
//   motor.P_angle.P = 3.0f;
//   motor.P_angle.I = 0.0f;
//   motor.P_angle.D = 0.468f;
//   motor.P_angle.output_ramp = 0.0f; // acceleration [rad/s^2
//   // maximal velocity of the position control
//   motor.velocity_limit = 20.0f;

//   // use monitoring with serial
//   Serial.begin(115200);
//   // comment out if not needed
//   motor.useMonitoring(Serial);


//   // initialize motor
//   motor.init();
//   // align sensor and start FOC
//   motor.zero_electric_angle = motor_GB4315_ZEAngle; // 6.24 cal. you can set it to a previously found value
//   motor.sensor_direction = motor_GB4315_sensor_direction; // you can set this to a previously found value
//   motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set

//   // add target command T
//   command.add('T', doTarget, "target angle");
  

//   // command.verbose = VerboseMode::machine_readable; // can be set using the webcontroller - optional
//   Serial.println(F("Motor ready."));
//   Serial.println(F("Set the target angle using serial terminal:"));
//   Serial.println("Starting...waiting for button press to start homing sequence");

//   // Configure pin and attach ISR on FALLING edge
//   pinMode(2, INPUT_PULLUP);
//   attachInterrupt(
//     digitalPinToInterrupt(2),
//     onButtonISR,
//     FALLING
//   );

  

//   // Wait for button press with debounce
//    Serial.println("Waiting for button press to start homing sequence...");
//     while (!buttonPressed) {
//         // Optional: Add watchdog or timeout
//         sensor.update();
//         Serial.print("Current Angle: ");
//         Serial.println(DisplayAngle(sensor.getAngle()));
//         delay(10); // Small delay to prevent tight loop
//     }
    
//     // Update angle before checking
//     sensor.update();
    
//     if (checkstarting()) {
//         Serial.println("Enter Error Loop: Motor not in starting position!");
//         while(1); // Halt execution
//     }

//   // Execute homing sequence
  
//   delay(1000);
//   Serial.println("Starting homing sequence...");
//     goToHomePosition();
  
// }



// void loop() {

//   // main FOC algorithm function
//   // the faster you run this function the better
//   // Arduino UNO loop  ~1kHz
//   // Bluepill loop ~10kHz
//   sensor.update();
  
//   motor.loopFOC();
  
//   // Motion control function
//   // velocity, position or voltage (defined in motor.controller)
//   // this function can be run at much lower frequency than loopFOC() function
//   // You can also use motor.move() and set the motor.target in the code
//   motor.move(getAugmentedAngle(target_Angle));
//   //Serial.print("Angle: "); Serial.println(sensor.getAngle(), 3);
//   // function intended to be used with serial plotter to monitor motor variables
//   // significantly slowing the execution down!!!!
//   Serial.println(DisplayAngle(sensor.getAngle()), 3);
//   // user communication
//   command.run();
// }
