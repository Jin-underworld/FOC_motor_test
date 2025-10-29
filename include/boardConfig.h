#pragma once
#ifndef BOARD_CONFIG_HPP
#define BOARD_CONFIG_HPP

// Board configurations
#define BOARD_ARDUINO_UNO 1
#define BOARD_ARDUINO_MEGA 2
#define BOARD_STM32_DISCO 3
#define BOARD_STM32_NUCLEO 4

// Pin configurations
#if SELECTED_BOARD == BOARD_ARDUINO_UNO
    // Arduino Mega pins for BLDC driver
    #define PIN_PWM_A 11
    #define PIN_PWM_B 10
    #define PIN_PWM_C 9
    #define PIN_ENABLE 8

    #define PIN_ENCODER 3   
    #define BAUD_RATE 115200
    // // I2C pins (for magnetic sensor)
    // #define PIN_SDA 20
    // #define PIN_SCL 21

#elif SELECTED_BOARD == BOARD_ARDUINO_MEGA
    // Arduino Mega pins for BLDC driver
    #define PIN_PWM_A 3
    #define PIN_PWM_B 4
    #define PIN_PWM_C 5
    #define PIN_ENABLE 23

    #define PIN_ENCODER 2   
    #define BAUD_RATE 115200	

    //SPI PINS
    #define PIN_CS 53
    #define PIN_SCK 52
    #define PIN_MISO 50
    #define PIN_MOSI 51
    
#elif SELECTED_BOARD == BOARD_STM32_DISCO
    // STM32 pins
    #define M0_PWM_IN1 PB8 //IN1 TIMER 4 on F4dsicovery
    #define M0_PWM_IN2 PB7 //IN2
    #define M0_PWM_IN3 PB6 //IN3
    #define M0_EN PE2

    #define M1_PWM_IN1 PA3 //IN1 on FOC board TIMER 2 on F4discovery
    #define M1_PWM_IN2 PA1 //IN2
    #define M1_PWM_IN3 PA0 //IN3
    #define M1_EN PE3

    #define M2_PWM_IN1 PA7//IN1 TIMER 3 on F4 discovery
    #define M2_PWM_IN2 PA6//IN2
    #define M2_PWM_IN3 PB0//IN3
    #define M2_EN PE4

    #define M3_PWM_IN1 PC8 //IN1 TIMER 8 on F4 Discovery
    #define M3_PWM_IN2 PC7 //IN2
    #define M3_PWM_IN3 PC6 //IN3
    #define M3_EN PE7

    // #define M4_PWM_IN1 GPIO_AF2_TIM5  //IN1 TIMER X on F4 Discovery
    // #define M4_PWM_IN2 PB14  //IN2
    // #define M4_PWM_IN3 PB15   //IN3
    // #define M4_EN PE6

    // #define M5_PWM_IN1 //PB15//PE6 timers to test, need to use TImer5 as master or TIM2 to TIm8 CH4
    // #define M5_PWM_IN2 //PB14//PE5
    // #define M5_PWM_IN3 //PB9 //PB9
    // #define M5_EN
    
    // I2C pins
    #define PIN_SDA PB11
    #define PIN_SCL PB10
    #define BAUD_RATE 115200
    #define BUTTON_PIN GPIO_PIN_0
    #define BUTTON_GPIO_PORT GPIOC

#elif SELECTED_BOARD == BOARD_STM32_NUCLEO
    // STM32 pins
    #define motor_GB6025_PWM_IN1 PB8 //IN1 TIMER 4 on F4dsicovery
    #define motor_GB6025_PWM_IN2 PB7 //IN2
    #define motor_GB6025_PWM_IN3 PB6  //IN3
    #define motor_GB6025_EN PB9

    #define motor_GB6010_PWM_IN1 PE13 //IN1 on FOC board TIMER 1 on F4discovery
    #define motor_GB6010_PWM_IN2 PE11 //IN2
    #define motor_GB6010_PWM_IN3 PE9 //IN3
    #define motor_GB6010_EN PE7

    #define motor_GB4315_PWM_IN1 PC9 //IN1 TIMER 3 on F4 discovery
    #define motor_GB4315_PWM_IN2 PC8 //IN2
    #define motor_GB4315_PWM_IN3 PC6  //IN3
    #define motor_GB4315_EN PC11
    
    // I2C pins
    #define PIN_SDA PB11
    #define PIN_SCL PB10
    #define BAUD_RATE 115200
    #define BUTTON_PIN GPIO_PIN_0
    #define BUTTON_GPIO_PORT GPIOA

#else
    #error "Board type not defined or not supported"
#endif

#endif // BOARD_CONFIG_HPP