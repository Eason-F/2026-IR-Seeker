#pragma once

#include <Arduino.h>

namespace config {

constexpr uint8_t SENSOR_COUNT = 18;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {
    1,  2, 3, 4,  5,  6,  7,  8,  9, 10,
    11, 12, 13, 14, 15, 16, 17, 18
};

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
constexpr uint16_t SENSOR_CALIBRATION[SENSOR_COUNT] = {
    256, 256, 256, 256, 256, 256, 256, 256, 256,
    256, 256, 256, 256, 256, 256, 256, 256, 256,
};

constexpr uint8_t SIGNALS_TO_USE = 5;

constexpr uint16_t SAMPLE_PERIOD_US = 50;
constexpr uint16_t MEASUREMENT_PERIOD_US = 4000;
constexpr uint16_t DEFAULT_MEASUREMENT_RATE_HZ = 250;
constexpr uint16_t DEFAULT_RAW_RATE_HZ = 0;
constexpr uint16_t STATUS_RATE_HZ = 1;
}

