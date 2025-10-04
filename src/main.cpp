#include "main.h"
#include <Arduino.h>
#include <SimpleFOC.h>

#include "DFRobot_I2C_Multiplexer.h"
TwoWire Wire2(PB11, PB10);
DFRobot_I2C_Multiplexer multiplexer(&Wire2, 0x70);
// Custom class that wraps MagneticSensorI2C to work with I2C multiplexer
class MultiplexedMagneticSensorI2C : public MagneticSensorI2C {
public:
  MultiplexedMagneticSensorI2C(DFRobot_I2C_Multiplexer *_multiplexer,
                               uint8_t _port, MagneticSensorI2CConfig_s _config)
      : MagneticSensorI2C(_config), multiplexer(_multiplexer), port(_port) {}

  // Override the getSensorAngle method to select the correct port before
  // reading
  float getSensorAngle() override {
    multiplexer->selectPort(port);
    return MagneticSensorI2C::getSensorAngle();
  }

  // Override update method if needed
  void update() override {
    multiplexer->selectPort(port);
    MagneticSensorI2C::update();
  }

private:
  DFRobot_I2C_Multiplexer *multiplexer;
  uint8_t port;
};

MultiplexedMagneticSensorI2C sensor1(&multiplexer, 0, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor2(&multiplexer, 1, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor3(&multiplexer, 2, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor4(&multiplexer, 3, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor5(&multiplexer, 4, AS5600_I2C);
MultiplexedMagneticSensorI2C sensor6(&multiplexer, 5, AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE,
                            MOTOR_TORQUE_CONSTANT, MOTOR_PHASE_INDUCTANCE);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);

Commander command = Commander(Serial);
void doMotor(char *cmd) { command.motor(&motor, cmd); }

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
volatile float Kt = MOTOR_TORQUE_CONSTANT; // Motor torque constant [N*m/A] -
                                           // calibrate for your motor
const float current_threshold = 0.05; // Minimum current for force detection [A]

void readAllEncoders() {
  sensor1.update();
  sensor2.update();
  // sensor3.update();
  // sensor4.update();
  // sensor5.update();
  // sensor6.update();

  Serial.print("Encoder 1: ");
  Serial.println(sensor1.getAngle(), 3);
  Serial.print("Encoder 2: ");
  Serial.println(sensor2.getAngle(), 3);
  // Serial.print("Encoder 3: "); Serial.println(sensor3.getAngle(), 3);
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

void setup() {

  Serial.begin(BAUD_RATE);
  Serial.println("Hi");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire2.begin();
  Serial.println("bruh");
  multiplexer.begin();
  Serial.println("I2C Multiplexer initialized");

  // // Initialize sensor
  sensor1.init(&Wire2);
  sensor2.init(&Wire2);
  // sensor3.init();
  // sensor4.init();
  // sensor5.init();
  // sensor6.init();
  // Serial.println("how can");
  // motor.linkSensor(&sensor1);

  // Initialize driver
  driver.voltage_power_supply = MOTOR_VOLTAGE_LIMIT;

  // driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::angle;
  motor.target = state.hold_pos_target;

  // motor.controller = MotionControlType::velocity;

  motor.velocity_limit = MOTOR_VELOCITY_LIMIT;

  // Initialize motor
  motor.init();

  motor.initFOC();

  // // Set initial control mode
  // motor.target = 0;

  // command.add('M',doMotor,"motor");
  // //motor.useMonitoring(Serial);
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
  digitalWrite(LED_BUILTIN, HIGH);

  // readAllEncoders();

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
