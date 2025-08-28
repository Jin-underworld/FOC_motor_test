#include <SimpleFOC.h>
#include <motorControl.h>

// Create motor control instance
MotorControl motorController;

// Commander instance for serial commands
Commander command = Commander(Serial);

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize motor controller
    motorController.init();
    
    // Use monitoring for the motor
    motorController.useMonitoring(Serial);
    
    delay(1000);
}

void loop() {
    // Main FOC algorithm
    motorController.loopFOC();
    
    // Run gravity compensation
    motorController.compensateGravity();
    
    // Process user commands
    command.run();
}