#pragma once
#ifndef BOARD_CONFIG_HPP
#define BOARD_CONFIG_HPP

// Board configurations
#define BOARD_ARDUINO_UNO 1
#define BOARD_ARDUINO_MEGA 2
#define BOARD_STM32 3

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
    
#elif SELECTED_BOARD == BOARD_STM32
    // STM32 pins
    #define PIN_PWM_A PA0
    #define PIN_PWM_B PA1
    #define PIN_PWM_C PA2
    #define PIN_ENABLE PA3
    // I2C pins
    #define PIN_SDA PB7
    #define PIN_SCL PB6
    #define BAUD_RATE 115200
#else
    #error "Board type not defined or not supported"
#endif

#endif // BOARD_CONFIG_HPP