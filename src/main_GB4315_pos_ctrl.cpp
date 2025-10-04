#include <SimpleFOC.h>

//help me package this somewhere
#define motor_GB4315_PP 14
#define motor_GB4315_R 13.84
#define motor_GB4315_KV 28
#define motor_GB4315_inductance 6.45
#define motor_GB4315_ZEAngle 5.56
#define motor_GB4315_sensor_direction Direction::CCW
#define motor_GB4315_ZeroCalibratedAngle -(1.49f+M_PI) 
#define motor_GB4315_HomeAngle 3.6f  // Define the home position angle (in radians)
#define motor_GB4315_UPPER_LIMIT 5.0f
#define motor_GB4315_LOWER_LIMIT 2.3f

#define motor_GB6010_PP 14
#define motor_GB6010_R 10.28
#define motor_GB6010_KV 16
#define motor_GB6010_inductance 12.35e-3
#define motor_GB6010_ZEAngle 0.05f
#define motor_GB6010_sensor_direction Direction::CCW
#define motor_GB6010_ZeroCalibratedAngle 2.97f-2*M_PI //IMPT!!!! code minus this away. so for angle use angle as is when at 0 position. PI adjustment is to avoid negative angles
#define motor_GB6010_HomeAngle 1.2f  // Define the home position angle (in radians)
#define motor_6010_UPPER_LIMIT 2.0f
#define motor_6010_LOWER_LIMIT -0.5f

#define BUTTON_PIN 3  // Pin connected to the button


/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */
class MedianFilter {
private:
    static const int SIZE = 7;
    float values[SIZE];
    int index = 0;
    bool buffer_initialized = false;
    
public:
    MedianFilter() {

        for (int i = 0; i < SIZE; i++) {
            values[i] = 0;
        }
    }
    
    float update(float new_value) {
        // Store new value
        values[index] = new_value;
        index = (index + 1) % SIZE;
        
        if (!buffer_initialized && index == 0) {
            buffer_initialized = true;
        }
        
        // Copy values for sorting
        float sorted[SIZE];
        memcpy(sorted, values, sizeof(values));
        
        // Simple bubble sort
        for (int i = 0; i < SIZE-1; i++) {
            for (int j = 0; j < SIZE-i-1; j++) {
                if (sorted[j] > sorted[j+1]) {
                    float temp = sorted[j];
                    sorted[j] = sorted[j+1];
                    sorted[j+1] = temp;
                }
            }
        }
        
        // Return median
        return sorted[SIZE/2];
    }
    
    // Add a method to initialize the buffer with a specific value
    void init(float value) {
        for (int i = 0; i < SIZE; i++) {
            values[i] = value;
        }
        buffer_initialized = true;
    }
};


class FilteredSensor: public Sensor {
  public:
    FilteredSensor(MagneticSensorPWM& _sensor) : sensor(_sensor) {
    }
    
    // Update the sensor and filter the angle
    void update() override {
      sensor.update();
      float new_angle = sensor.getAngle();
      
      // Use the median filter to update and get the filtered angle
      filtered_angle = filter.update(new_angle);
    }
    
    // Override getAngle to return the filtered angle
    float getAngle() override {
      return filtered_angle;
    }
    
    // Pass through velocity calculation to the original sensor
    float getVelocity() override {
      return sensor.getVelocity();
    }
    
    // Enable PWM interrupt handling
    void enableInterrupt(void (*doPWM)(void)) {
      sensor.enableInterrupt(doPWM);
    }
    
    // Initialize the wrapped sensor
    void init() {
      sensor.init();
      // Initialize the filter with the first sensor reading
      float initial_angle = sensor.getAngle();
      filter.init(initial_angle);
      filtered_angle = initial_angle;
    }
    
    // Get the raw angle for debugging
    float getRawAngle() {
      return sensor.getAngle();
    }

    float getSensorAngle() override {
      return sensor.getSensorAngle();
    }
    
  private:
    MagneticSensorPWM& sensor;
    MedianFilter filter;
    float filtered_angle = 0;
};



// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);
//MagneticSensorPWM sensor = MagneticSensorPWM(2, 3, 945);// low

//FilteredSensor filteredSensor = FilteredSensor(sensor);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(motor_GB4315_PP, motor_GB4315_R, motor_GB4315_KV, motor_GB4315_inductance);
BLDCDriver3PWM driver = BLDCDriver3PWM(8,10,12);//arduino uno
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// angle set point variable
float target_Angle = motor_GB4315_HomeAngle; // target angle value

// instantiate the commander
//PIDController pid = PIDController(10.0f,0.0f,0.0f,1000.0f,_PI); // 10,0,0
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_Angle, cmd); }

bool checkstarting() {
  sensor.update();
  float CurrentAngle = motor.shaftAngle();
  Serial.print("Current Angle CHECKER: ");
  Serial.println(CurrentAngle);
  if (CurrentAngle >= motor_GB4315_UPPER_LIMIT or CurrentAngle < motor_GB4315_LOWER_LIMIT) {
    Serial.println("Error: Motor not in starting position. Please move the motor to the home position (0 radians).");
    return 1;
  }
  else {
    Serial.println("Motor in starting position. Proceeding with homing sequence.");
    return 0;
  }
}

bool goToHomePosition(unsigned long timeout_ms = 5000) {
    unsigned long start_time = millis();
    motor.target =motor_GB4315_HomeAngle;
    
    while (fabs(motor.shaftAngle() - motor_GB4315_HomeAngle) > 0.01f){
        motor.loopFOC();
        motor.move();
        
        // Add timeout
        if (millis() - start_time > timeout_ms) {
            Serial.println("Homing sequence timed out!");
            return false;
        }
        
        // Optional: Add progress feedback
        if (millis() % 500 == 0) {
            Serial.print("Homing... Current angle: ");
            Serial.println(motor.shaftAngle());
        }
    }
    return true;
}

// Must be volatile to share with ISR
volatile bool buttonPressed = false;
volatile bool buttonState = false;
volatile unsigned long pressStartTime = 0;
volatile unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // milliseconds
const unsigned long LONG_PRESS_DURATION = 1000;    // 1 second for long press

void onButtonISR() {
    unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime > DEBOUNCE_DELAY) {
        bool reading = (digitalRead(BUTTON_PIN) == LOW);
        if (reading != buttonState) {
            buttonState = reading;
            buttonPressed = buttonState;
            lastDebounceTime = currentTime;
        }
    }
}


void setup() {

  // initialise magnetic sensor hardware


  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0f; //IMPTTTT!!!!!
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle; // angle, velocity, torque, velocity_openloop, angle_openloop
  //motor.torque_controller = TorqueControlType::voltage; // voltage, dc_current, foc_current
  // contoller configuration
  // default parameters in defaults.h

  //velocity PI controller parameters
  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 3.2f;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.07; // filter for velocity, needs to be around 0.05s

  // maximal voltage to be set to the motor IMPTTTT!!!
  motor.voltage_limit = 7.0f; // [V]
  

  // angle P controller
  motor.P_angle.P = 4.0f;
  motor.P_angle.I = 0.0f;
  motor.P_angle.D = 5.2f;
  motor.P_angle.output_ramp = 0.2f; // acceleration [rad/s^2
  motor.LPF_angle.Tf = 0.12f; // filter for angle, needs to be around 0.1s
  // maximal velocity of the position control
  motor.velocity_limit = 20.0f;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.voltage_sensor_align = 4.0f; // voltage used for sensor alignment

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.zero_electric_angle = motor_GB4315_ZEAngle; //  you can set it to a previously found value
  motor.sensor_direction = motor_GB4315_sensor_direction; // you can set this to a previously found value
  motor.sensor_offset = motor_GB4315_ZeroCalibratedAngle; // you can set this to a previously found value
  motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set
  // add target command T
  command.add('T', doTarget, "target angle");
  //command.add('C', onPid, "pid controller");
  

  // command.verbose = VerboseMode::machine_readable; // can be set using the webcontroller - optional
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  Serial.println("Starting...waiting for button press to start homing sequence");

  // Configure pin and attach ISR on FALLING edge
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(BUTTON_PIN),
    onButtonISR,
    CHANGE
  );
  


  //Wait for button press with debounce
   Serial.println("Waiting for button press to start homing sequence...");
    while (!buttonPressed) {
        // Optional: Add watchdog or timeout
        sensor.update();
        Serial.print("Current Angle: ");
        Serial.println(motor.shaftAngle());
        delay(10); // Small delay to prevent tight loop
    }
    
    // Update angle before checking
    sensor.update();
    
    if (checkstarting()) {
        Serial.println("Enter Error Loop: Motor not in starting position!");
        while(1); // Halt execution
    }

  // Execute homing sequence
  
  delay(1000);
  Serial.println("Starting homing sequence...");
    goToHomePosition();
  
  Serial.println("Homing sequence completed. Motor is at home position.");
  buttonPressed = false; // Reset button press flag
}



void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  sensor.update();
  
  motor.loopFOC();
  
  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_Angle);
  //motor.move(target_Angle);
  //Serial.print("Target Angle: "); Serial.println(target_Angle,3);
  //Serial.print("Angle: "); Serial.println(DisplayAngle(sensor.getAngle()),3);
  Serial.print("Motor shaft: "); Serial.println(motor.shaftAngle(),3);
  // if (buttonPressed) {
  //   Serial.println("Button pressed");
  // }

  //Serial.print("Sensor angle: "); Serial.println(sensor.getAngle(),3);
  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //Serial.println(motor.shaftVelocity());
  //motor.monitor();
  // user communication
  command.run();


  if (buttonPressed) {
    delay(50); // Debounce delay
    onButtonISR(); // Update button state
    Serial.println("Button pressed! Entering error loop.");
    motor.disable();

    while (buttonPressed) {
      // Wait for button release
      delay(10);
      Serial.print("press button again to exit error loop.");
    }


    Serial.println("Exiting error loop. Resuming operation.");
    motor.enable();
    sensor.update();
    target_Angle = motor.shaftAngle(); // Set target to current position
   
    

  }
  
}
