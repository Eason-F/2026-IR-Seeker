#pragma once

#include <Arduino.h>

namespace config {

constexpr uint8_t SENSOR_COUNT = 18;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {
    1,  2, 3, 4,  5,  6,  7,  8,  9, 10,
    11, 12, 13, 14, 15, 16, 17, 18
};

constexpr uint32_t UART_BAUD = 115200;

constexpr bool DEBUG_OUTPUT_ENABLED = false;
constexpr uint32_t TEST_SERIAL_BAUD = 115200;
constexpr uint16_t TEST_PRINT_RATE_HZ = 20;

constexpr bool SENSOR_ACTIVE_LOW = true;
constexpr uint8_t SENSOR_INPUT_MODE = INPUT_PULLUP;

constexpr uint16_t SENSOR_CALIBRATION[SENSOR_COUNT] = {
    171, 174, 233, 165, 165, 223, 163, 155, 219,
    165, 165, 244, 155, 154, 256, 160, 160, 256,
};

constexpr uint8_t SIGNALS_TO_USE = 5;

constexpr uint16_t SAMPLE_PERIOD_US = 50;

constexpr uint32_t MEASUREMENT_PERIOD_US = 12000;
}
