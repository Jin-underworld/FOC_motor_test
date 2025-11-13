// #include <Arduino.h>

// // Simple line buffer + parser for Serial/Serial2 commands
// static char inbuf[64];
// static uint8_t inlen = 0;
// static volatile bool serial_command_flag = false;

// static void blinkBlueTwice() {
//   digitalWrite(LED_BLUE, HIGH); delay(50);
//   digitalWrite(LED_BLUE, LOW);  delay(50);
//   digitalWrite(LED_BLUE, HIGH); delay(50);
//   digitalWrite(LED_BLUE, LOW);
// }

// static void processLine() {
//   // trim trailing spaces/tabs
//   while (inlen && (inbuf[inlen-1] == ' ' || inbuf[inlen-1] == '\t')) inlen--;
//   inbuf[inlen] = '\0';

//   if (inlen == 0) return; // ignore empty

//   if (strcmp(inbuf, "0x01") == 0) {
//     serial_command_flag = true;
//     blinkBlueTwice();
//     Serial2.println("Command received: Flag set to TRUE");
//     //if (Serial) Serial.println("Command received: Flag set to TRUE");
//   } else if (strcmp(inbuf, "0x00") == 0 || strcmp(inbuf, "reset") == 0) {
//     serial_command_flag = false;
//     Serial2.println("Command received: Flag set to FALSE");
//     //if (Serial) Serial.println("Command received: Flag set to FALSE");
//   } else {
//     Serial2.print("Unknown command: "); Serial2.println(inbuf);
//     //if (Serial) { Serial.print("Unknown command: "); Serial.println(inbuf); }
//   }
//   inlen = 0; // reset buffer
// }

// static void readFrom(Stream& s) {
//   while (s.available()) {
//     char c = s.read();
//     if (c == '\n' || c == '\r') {
//       if (inlen) processLine(); // handle CR, LF, or CRLF
//       else inlen = 0;           // eat consecutive line endings
//     } else if (inlen + 1 < sizeof(inbuf)) {
//       inbuf[inlen++] = c;
//     } else {
//       // overflow: reset buffer
//       inlen = 0;
//     }
//   }
// }

// void setup() {
//   pinMode(LED_BLUE, OUTPUT);
//   digitalWrite(LED_BLUE, LOW);

//   // USB serial (Arduino Serial Monitor)
//   //Serial.begin(115200);
//   // USART2 on PD5/PD6 (ST discovery connector)
//   Serial2.setTx(PD5);
//   Serial2.setRx(PD6);
//   Serial2.begin(115200);

//   Serial2.println("genericBlinky serial test ready. Send 0x01 or 0x00 + newline.");
//   //if (Serial) Serial.println("genericBlinky serial test ready. Send 0x01 or 0x00 + newline.");
// }

// void loop() {
//   // Poll both USB Serial and USART2
//   readFrom(Serial2);
//   //readFrom(Serial);
//   static uint32_t t1 = 0;
//   uint32_t now2 = micros();
//   if (serial_command_flag){
//     Serial2.println("gayness oe");
//   }
//   Serial2.println("flooder");

  
//   // Optional: tiny heartbeat every 1s (doesn't interfere with the 2-blink command)
//   static uint32_t t0 = 0;
//   uint32_t now = millis();
//   if (now - t0 >= 1000) {
//     t0 = now;
    
//     digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
//   }
// }
