#include <SimpleFOC.h>
#include "main.h"

MagneticSensorPWM sensor = MagneticSensorPWM(PIN_ENCODER,3,909);// low
void doPWM() {
    sensor.handlePWM();
}
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, 8.8f); // 7 pole pairs, 3.0 ohm phase resistance
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);


class MedianFilter {
private:
    static const int SIZE = 7;
    float values[SIZE];
    int index = 0;
    
public:
    float update(float new_value) {
        // Store new value
        values[index] = new_value;
        index = (index + 1) % SIZE;
        
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
};

MedianFilter pos_median_filter;
LowPassFilter position_filter = LowPassFilter(0.05); 

class FilteredSensor: public Sensor {
private:
    Sensor* baseSensor;
    MedianFilter medianFilter;
    LowPassFilter lowPassFilter = LowPassFilter(0.05);
    
protected:
    // This is the critical method to override - it powers the base class
    float getSensorAngle() override {
        float raw = baseSensor->getSensorAngle();
        // Normalize to 0-2π range if needed
        raw = fmod(raw, 2*PI);
        if (raw < 0) raw += 2*PI;
        
        float median_filtered = medianFilter.update(raw);
        return lowPassFilter(median_filtered);
    }
    
public:
    FilteredSensor(Sensor* _baseSensor) : baseSensor(_baseSensor) {}
    
    void init() override {
        baseSensor->init();
    }
    
    int needsSearch() override {
        return baseSensor->needsSearch();
    }
};

#define ARM_LENGTH 0.1f    // 100mm = 0.1m
#define LOAD_MASS 0.05f    // 20g = 0.02kg
#define GRAVITY 9.81f      // m/s^2
#define VOLTAGE_TO_TORQUE_RATIO 0.1983f  //TUNED [Nm/V] - adjust this based on your motor， theoretiacal: 0.0434/2.3
float compensate_angle = 1.95f + 0.166; // angle to be compensated [rad]
int mode = -1; //1 or -1 to change direction of compensation
float lift_ratio = 1.3f; // to slightly overcompensate gravity

float calculateGravityTorque(float angle) {
    // Torque = mass * gravity * arm_length * sin(angle)
    // This gives the torque needed to counteract gravity
    float gravity_torque = LOAD_MASS * GRAVITY * ARM_LENGTH * sin(angle);
    return gravity_torque;
}

float torqueToVoltage(float torque) {
    return torque / VOLTAGE_TO_TORQUE_RATIO;
}

// torque set point variable
float max_torque = 2.0; // maximum torque limit [Nm]
float current_torque = 0.0; // current torque value
float voltage_to_move = 0.0; // target angle value

Commander command = Commander(Serial);
FilteredSensor filtered_sensor = FilteredSensor(&sensor);


void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Starting gravity compensation setup...");

  sensor.init();
  sensor.enableInterrupt(doPWM);
  motor.linkSensor(&sensor);

  // driver config
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage; //changed to voltage control*
  motor.controller = MotionControlType::torque; //changed to torque control*

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 0.0f;
  motor.PID_velocity.D = 0.3f;
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

  max_torque = VOLTAGE_TO_TORQUE_RATIO * motor.voltage_limit; // [Nm] set the maximum torque*

  // comment out if not needed
  motor.useMonitoring(Serial);
  compensate_angle = filtered_sensor.getAngle(); 

  command.add('M', doMotor, "motor");
  motor.zero_electric_angle = 1.96; // 2.21 cal. you can set it to a previously found value
  motor.sensor_direction = CCW; // you can set this to a previously found value

  // initialize motor
  motor.init();
  // align sensor and start FOC
  
  motor.initFOC(); // set to false to skip the alignment if the zero_electric_angle is set
  
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
  
  float current_angle = sensor.getAngle() - compensate_angle; // adjust this offset to set the "zero" position*
  
  // Calculate gravity compensation torque
  float gravity_torque = calculateGravityTorque(current_angle);
  
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
  // user communication
  //command.run();
}