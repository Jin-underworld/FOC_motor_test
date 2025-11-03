#include <SimpleFOC.h>
#include "custom_BLDCmotor.h"

//help me package this somewhere
// #define motor_GB4315_PP 14
// #define motor_GB4315_R 13.84
// #define motor_GB4315_KV 28
// #define motor_GB4315_Kt 0.47 //Nm/A
// #define motor_GB4315_inductance 6.45e-3
// #define motor_GB4315_ZEAngle 2.93f//4.02f
// #define motor_GB4315_sensor_direction Direction::CW
// #define motor_GB4315_ZeroCalibratedAngle 0.0f //IMPT!!!! code minus this away. so for angle use angle as is when at 0 position.
// #define motor_GB4315_HomeAngle M_PI/3 // Define the home position angle (in radians)
// #define motor_GB4315_UPPER_LIMIT 2.628f
// #define motor_GB4315_LOWER_LIMIT 0.005f

#define motor_GB4315_PP 11
#define motor_GB4315_R 2.55
#define motor_GB4315_KV 220
#define motor_GB4315_KT 0.06 //Nm/A
#define motor_GB4315_INDUCTANCE 6.45
#define motor_GB4315_ZEAngle 3.27f
#define motor_GB4315_sensor_direction Direction::CCW
#define motor_GB4315_ZeroCalibratedAngle 0.0f
#define motor_GB4315_HomeAngle 1.1f  // Define the home position angle (in radians)
#define motor_GB4315_UPPER_LIMIT 3.0f
#define motor_GB4315_LOWER_LIMIT -0.5f


#define MOTOR_GB6010_PP 14
#define MOTOR_GB6010_R 5.14
#define MOTOR_GB6010_KV 16
#define MOTOR_GB6010_KT 0.61 //Nm/A
#define MOTOR_GB6010_INDUCTANCE 6.175e-3f
#define motor_GB6010_ZEAngle 0.05f
#define motor_GB6010_sensor_direction Direction::CCW
#define motor_GB6010_ZeroCalibratedAngle 2.97f-2*M_PI //IMPT!!!! code minus this away. so for angle use angle as is when at 0 position. PI adjustment is to avoid negative angles
#define motor_GB6010_HomeAngle 1.2f  // Define the home position angle (in radians)
#define motor_6010_UPPER_LIMIT 2.0f
#define motor_6010_LOWER_LIMIT -0.5f

#define motor_GB6025_PP 14
#define motor_GB6025_R 1.675
#define motor_GB6025_KV 16
#define motor_GB6025_Kt 0.61 //Nm/A
#define motor_GB6025_inductance 1.865e-3f
#define motor_GB6025_ZEAngle 4.29f
#define motor_GB6025_sensor_direction Direction::CCW
#define motor_GB6025_ZeroCalibratedAngle -5.88f //IMPT!!!! code minus this away. so for angle use angle as is when at 0 position. PI adjustment is to avoid negative angles
#define motor_GB6025_HomeAngle 1.2f  // Define the home position angle (in radians)
#define motor_6010_UPPER_LIMIT 2.0f
#define motor_6010_LOWER_LIMIT -0.5f


#define MOTOR_GB3510_PP 11
#define MOTOR_GB3510_R 4.265f
#define MOTOR_GB3510_KV 80
#define MOTOR_GB3510_KT 0.2f
#define MOTOR_GB3510_INDUCTANCE 0.95e-3f
#define MOTOR_GB3510_CURRENT_LIMIT 0.53f  // Maximum continuous current, original is 0.59, only run (0.8) with cooling


#define JOINT1_ZE_ANGLE 6.11f
#define JOINT1_SENSOR_DIRECTION Direction::CCW
#define JOINT1_ZERO_CALIBRATED_ANGLE -2.94f

// Joint 3 Configuration (GB3510 installed as compensator joint)
#define JOINT3_ZE_ANGLE 3.45f
#define JOINT3_SENSOR_DIRECTION Direction::CCW
#define JOINT3_ZERO_CALIBRATED_ANGLE 0.444f
#define JOINT3_HOME_ANGLE 0.0f
#define JOINT3_UPPER_LIMIT -(0.752f - 0.001f)
#define JOINT3_LOWER_LIMIT -(0.752f - 0.001f)

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
new_BLDCMotor motor = new_BLDCMotor(
MOTOR_GB6010_PP, 
MOTOR_GB6010_R, 
MOTOR_GB6010_KV,
MOTOR_GB6010_KT, 
MOTOR_GB6010_INDUCTANCE
);
BLDCDriver3PWM driver = BLDCDriver3PWM(6,7,8);//arduino uno
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// angle set point variable
float target_Angle = 0.4f; // target angle value

// instantiate the commander
//PIDController pid = PIDController(10.0f,0.0f,0.0f,1000.0f,_PI); // 10,0,0
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_Angle, cmd); }

// new: add command to set velocity PID P term via serial: send e.g. "P 0.25"
void doVelP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }

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

InlineCurrentSense current_sense = InlineCurrentSense(0.01,50,A0,A1,_NC); // pin, gain, shunt resistance

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

  //link current sense
  current_sense.linkDriver(&driver);
  

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque; // angle, velocity, torque, velocity_openloop, angle_openloop
  motor.torque_controller = TorqueControlType::foc_current; // voltage, dc_current, foc_current
  // contoller configuration
  // default parameters in defaults.h

  //velocity PI controller parameters
  motor.PID_velocity.P = 0.36f;
  motor.PID_velocity.I = 8.9f;
  motor.PID_velocity.D = 0.001f;
  motor.LPF_velocity.Tf = 0.035; // filter for velocity, needs to be around 0.05s
  motor.velocity_limit = 7.0f; // [rad/s]
  //angle P controller
  motor.P_angle.P = 1.2f;
  motor.P_angle.I = 0.05f;
  motor.P_angle.D = 0.001f;
  motor.P_angle.output_ramp = 5000; // acceleration [rad/s^2]
  motor.LPF_angle.Tf = 0.0f; // filter for angle, needs to be around 0.1s
  // maximal velocity of the position control
  
  
  // maximal voltage to be set to the motor IMPTTTT!!!
  motor.voltage_limit = 5.0f; // [V]
  motor.voltage_sensor_align = 3.0f; // [V]
  //motor.current_limit = 0.5f; // [A] for current controllers


  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  current_sense.gain_a *= -1.0f; // invert current sense if needed
  current_sense.gain_b *= -1.0f; // invert current sense if needed
  motor.init();
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  
  // align sensor and start FOC
  if(true){
    motor.zero_electric_angle = JOINT1_ZE_ANGLE; //  you can set it to a previously found value
    motor.sensor_direction =    JOINT1_SENSOR_DIRECTION; // you can set this to a previously found value
    motor.sensor_offset =       JOINT1_ZERO_CALIBRATED_ANGLE; // you can set this to a previously found value
  }

  current_sense.skip_align = true; // skip current sense alignment if previously done

  motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set
  // add target command T
  command.add('T', doTarget, "target angle");
  command.add('P', doVelP, "set velocity PID P");
  //command.add('C', onPid, "pid controller");
  

  // command.verbose = VerboseMode::machine_readable; // can be set using the webcontroller - optional
  // Serial.println(F("Motor ready."));
  // Serial.println(F("Set the target angle using serial terminal:"));
  // Serial.println("Starting...waiting for button press to start homing sequence");

  // Configure pin and attach ISR on FALLING edge
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(BUTTON_PIN),
    onButtonISR,
    CHANGE
  );
  
  delay(1000);


  //Wait for button press with debounce
   Serial.println("Waiting for button press to start homing sequence...");
    // while (!buttonPressed) {
    //     // Optional: Add watchdog or timeout
    //     sensor.update();
    //     Serial.print("Current Angle: ");
    //     Serial.println(motor.shaftAngle());
    //     delay(10); // Small delay to prevent tight loop
    // }
    
    // Update angle before checking
    sensor.update();
    
}



float K_spring = 0.0f; // Spring constant (Nm/rad)
float C_damper = 1.2f; // Damping coefficient (Nm/(rad/s))
float last_debug = 0.0f;



void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  sensor.update();
  
  

  // constrain target angle within limits
  //target_Angle = constrain(target_Angle, motor_GB4315_LOWER_LIMIT, motor_GB4315_UPPER_LIMIT);

  
  float dtheta = motor.shaftVelocity();
  float theta = motor.shaftAngle();

  float ref_angle = target_Angle;
  float torque_cmd = 
       -K_spring * (theta - ref_angle)
       -C_damper * dtheta;

  float current_cmd = torque_cmd/0.47f; //Kt
  
  motor.loopFOC();

  motor.move(current_cmd);

  //current_cmd = constrain(current_cmd, -0.90f, 0.90f);

  // motor.IMP_move(target_Angle);
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();



  //motor.monitor();
  // user communication
  command.run();
  if (millis() - last_debug > 200) {
    Serial.print("Target Angle: "); Serial.print(target_Angle,3);
    Serial.print(", Motor Angle: "); Serial.print(motor.shaftAngle(),3);
    // Serial.print(", Raw Sensor Angle: "); Serial.print(sensor.getAngle(),3);
    // Serial.print(", Velocity: "); Serial.print(motor.shaftVelocity(),3);
    Serial.print(", Motor Current Ia: "); Serial.print(currents.a,3);
    Serial.print(", Motor Current Ib: "); Serial.print(currents.b,3);
    Serial.print(", Motor Current Ic: "); Serial.print(currents.c,3);
    Serial.print(", DC Current: "); Serial.print(current_magnitude,3);
    Serial.print(", Motor Voltage Uq: "); Serial.print(motor.voltage.q,3);
    Serial.print(", Motor Current Iq: "); Serial.print(motor.current.q,3);
    //Serial.print(", Current Command: "); Serial.print(current_cmd,3);
    Serial.println();
    last_debug = millis();
  }

  // if (buttonPressed) {
  //   delay(50); // Debounce delay
  //   onButtonISR(); // Update button state
  //   Serial.println("Button pressed! Entering error loop.");
  //   motor.disable();

  //   while (buttonPressed) {
  //     // Wait for button release
  //     delay(10);
  //     Serial.print("press button again to exit error loop.");
  //   }


  //   Serial.println("Exiting error loop. Resuming operation.");
  //   motor.enable();
  //   sensor.update();
  //   target_Angle = motor.shaftAngle(); // Set target to current position
   
    

}

