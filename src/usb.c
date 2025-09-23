#include "usb_device.h"
#include "usbd_cdc_if.h"

void setup() {
    // Initialize USB communication
    MX_USB_DEVICE_Init();  // Initializes USB and CDC class

    // Wait for USB connection
    HAL_Delay(1000);
}

void loop() {
    // Send a message every 1 second over USB CDC
    uint8_t message[] = "Hello from STM32F4 via USB!\r\n";
    CDC_Transmit_FS(message, sizeof(message) - 1);  // Send message over USB
    HAL_Delay(1000);  // Delay for 1 second
}
