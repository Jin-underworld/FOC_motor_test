// #include <SimpleFOC.h>

// MagneticSensorPWM sensor = MagneticSensorPWM(2,3,909);// low

// Commander command = Commander(Serial);

// void doPWM() {
//     sensor.handlePWM();
// }

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Starting setup...");
//   sensor.init();
//   sensor.enableInterrupt(doPWM);
//   Serial.print("Sensor angle: ");
//   Serial.println(sensor.getAngle());
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   sensor.update();
//   Serial.println(sensor.getAngle());
//   //delay(100);
// }