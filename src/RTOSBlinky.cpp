// #include <Arduino.h>
// #include <STM32FreeRTOS.h>

// // Task handles
// TaskHandle_t defaultTaskHandle = NULL;
// TaskHandle_t task2Handle = NULL;

// // Global variables
// uint8_t indx = 0;

// // Task function prototypes
// void StartDefaultTask(void *pvParameters);
// void Task2Function(void *pvParameters);

// void setup() {
//   // Initialize serial at 115200 baud rate
//   Serial.begin(115200);
//   delay(1000); // Give serial port time to initialize
  
//   Serial.println("System starting...");
  
//   // Initialize GPIO (simplified from your MX_GPIO_Init)
//   pinMode(LED_BUILTIN, OUTPUT);
  
//   // Create the default task with normal priority
//   xTaskCreate(
//     StartDefaultTask,        // Function that implements the task
//     "DefaultTask",           // Text name for the task
//     128,                     // Stack size in words (not bytes)
//     NULL,                    // Task input parameter
//     1,                       // Priority (1 is low, configMAX_PRIORITIES-1 is high)
//     &defaultTaskHandle       // Task handle
//   );
  
//   // Create the second task with higher priority
//   xTaskCreate(
//     Task2Function,           // Function that implements the task
//     "Task2",                 // Text name for the task
//     128,                     // Stack size in words (not bytes)
//     NULL,                    // Task input parameter
//     2,                       // Higher priority than the default task
//     &task2Handle             // Task handle
//   );
  
//   // Start the RTOS scheduler
//   vTaskStartScheduler();
  
//   // We should never get here as control is now taken by the scheduler
//   Serial.println("Scheduler failed to start!");
//   while (1);
// }

// void loop() {
//   // Empty, as FreeRTOS takes over control
// }

// // Default task implementation
// void StartDefaultTask(void *pvParameters) {
//   (void) pvParameters;
  
//   for (;;) {
//     Serial.println("DefaultTask");
//     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
//     vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
//   }
// }

// // Task2 implementation (similar to your task2_init)
// void Task2Function(void *pvParameters) {
//   (void) pvParameters;
  
//   for (;;) {
//     Serial.print("Task2, indx = ");
//     Serial.println(indx++);
//     vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
    
//     if (indx == 3) {
//       Serial.println("Task2 waiting for 3 seconds...");
//       TickType_t xLastWakeTime = xTaskGetTickCount();
//       vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3000)); // Absolute delay
//     }
    
//     // Uncomment to demonstrate suspending/resuming tasks
//     /*
//     if (indx == 4) {
//       Serial.println("Suspending DefaultTask");
//       vTaskSuspend(defaultTaskHandle);
//     }
    
//     if (indx == 7) {
//       Serial.println("Resuming DefaultTask");
//       vTaskResume(defaultTaskHandle);
//     }
//     */
//   }
// }