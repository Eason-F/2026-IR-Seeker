#pragma once

#include <Arduino.h>

namespace config {

constexpr uint8_t SENSOR_COUNT = 18;

// Direct GPIO connection for each receiver. Sensor 0 faces robot-forward and
// the sensor numbers increase clockwise in 20-degree steps. Update this table
// to match the PCB before connecting the board.
//
// GPIO 17 and 18 are deliberately omitted because they are used by UART1.
// GPIO 19 and 20 are omitted to leave native USB available.
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {
    1,  2, 3, 4,  5,  6,  7,  8,  9, 10,
    11, 12, 13, 14, 15, 16, 17, 18
};

// UART1 connects to the Teensy 4.1. Both boards use 3.3 V logic.
constexpr int PIN_UART_RX = 17;
constexpr int PIN_UART_TX = 18;
constexpr uint32_t UART_BAUD = 460800;

// UART0 test output. Each line contains the latest values for sensors 0-17.
constexpr uint32_t TEST_SERIAL_BAUD = 115200;
constexpr uint16_t TEST_PRINT_RATE_HZ = 20;

// Integrated IR receiver modules normally pull their output low on detection.
constexpr bool SENSOR_ACTIVE_LOW = true;
constexpr uint8_t SENSOR_INPUT_MODE = INPUT_PULLUP;

// Per-channel gain in Q8 format: 256 = 1.000. Adjust after calibration.
constexpr uint16_t SENSOR_GAIN_Q8[SENSOR_COUNT] = {
    256, 256, 256, 256, 256, 256, 256, 256, 256,
    256, 256, 256, 256, 256, 256, 256, 256, 256,
};

constexpr uint16_t SAMPLE_RATE_HZ = 20000;
constexpr uint16_t DEFAULT_MEASUREMENT_RATE_HZ = 250;
constexpr uint16_t DEFAULT_RAW_RATE_HZ = 0;  // Off during matches.
constexpr uint16_t STATUS_RATE_HZ = 1;

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t FIRMWARE_MAJOR = 1;
constexpr uint8_t FIRMWARE_MINOR = 0;
constexpr uint8_t FIRMWARE_PATCH = 0;
constexpr uint8_t HARDWARE_VERSION = 1;

}  // namespace config
