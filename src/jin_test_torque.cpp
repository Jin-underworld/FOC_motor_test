#include <SimpleFOC.h>
#include "main.h"
/**
 *
 * Position/angle motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target angle (in radians) from serial terminal
 *
 */

 // start copying from here

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


MagneticSensorPWM sensor = MagneticSensorPWM(PIN_ENCODER, 3, 943);// low

FilteredSensor filtered_sensor(sensor);
// end copying here

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS); 
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);

// angle set point variable
float target_angle = 0.0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void doPWM() {
    sensor.handlePWM();
}

//PIDController pid_position = PIDController(MOTOR_PID_VELOCITY_P, MOTOR_PID_VELOCITY_I, MOTOR_PID_VELOCITY_D, MOTOR_PID_VELOCITY_RAMP, MOTOR_VOLTAGE_LIMIT);  // P, I, D, ramp, limit

void setup() {
   // use monitoring with serial
  Serial.begin(BAUD_RATE);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  sensor.init();
  sensor.enableInterrupt(doPWM);
    filtered_sensor.init();  // Initialize the filtered sensor

    Serial.print("filtered angle: ");
  Serial.println(filtered_sensor.getAngle());
  motor.linkSensor(&filtered_sensor); // add this to link to the ffiltered_sensor instead of the sensor itself

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SinePWM  ;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 0.5f;
  motor.PID_velocity.D = 0.0f;
  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.0f;

  // angle P controller
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.velocity_limit = 10;


  // comment out if not needed
  motor.useMonitoring(Serial);


  // initialize motor
  motor.init();

  Serial.println(motor.zero_electric_angle);
  motor.sensor_direction = MOTOR_SENSOR_DIRECTION; 
  motor.zero_electric_angle = MOTOR_ZERO_ELECTRIC_ANGLE;
  
  motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  Serial.println("Starting...");
  _delay(1000);
  
}


void loop() {

  filtered_sensor.update();
  motor.loopFOC();

  motor.move(target_angle);
  //Serial.print("Angle: "); Serial.println(sensor.getAngle(), 3);
  Serial.print("filtered Angle: "); Serial.println(filtered_sensor.getAngle(), 3);

  command.run();
}
