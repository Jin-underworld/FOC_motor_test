#pragma once
#ifndef ENCODER_CONFIG_HPP
#define ENCODER_CONFIG_HPP

#include <SimpleFOC.h>

// Encoder type enumeration
enum class EncoderType {
    AS5600_I2C,

    // sample for future encoders
    // AS5147_SPI,
    // add more for other encoders next time
    NONE  // For joints without encoders
};

// Encoder configuration struct
struct EncoderConfig {
    EncoderType type;
    int csPin;          // Chip select pin for SPI encoders
    int analogPin;      // Pin for analog encoders
    float cpr;          // Counts per revolution
    Direction direction;// Direction of the encoder
    int i2cAddress;     // I2C address for I2C encoders
    // Add any other encoder-specific parameters here
};

// Default encoder configurations
namespace DefaultEncoders {
    constexpr EncoderConfig AS5600 = {
        .type = EncoderType::AS5600_I2C,
        .csPin = -1,
        .analogPin = -1,
        .cpr = 4096,
        .direction = Direction::CW,
        .i2cAddress = 0x36
    };

    // sample for future encoders

    // constexpr EncoderConfig AS5147 = {
    //     .type = EncoderType::AS5147_SPI,
    //     .csPin = 10,
    //     .analogPin = -1,
    //     .cpr = 4096,
    //     .direction = Direction::CW,
    //     .i2cAddress = -1
    // };

}

#endif // ENCODER_CONFIG_HPP