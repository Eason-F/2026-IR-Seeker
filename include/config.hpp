#pragma once

#include <Arduino.h>

// #define WITH_ODOMETRY

namespace config {

constexpr uint8_t SENSOR_COUNT = 18;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {
    1,  2, 3, 4,  5,  6,  7,  8,  9, 10,
    11, 12, 13, 14, 15, 16, 17, 18
};

constexpr bool CALIBRATION_MODE = false;
constexpr bool DEBUG_OUTPUT_ENABLED = false;

constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t TEST_SERIAL_BAUD = 115200;
constexpr uint16_t TEST_PRINT_RATE_HZ = 20;

constexpr bool SENSOR_ACTIVE_LOW = true;
constexpr uint8_t SENSOR_INPUT_MODE = INPUT_PULLUP;

#ifndef WITH_ODOMETRY
constexpr uint16_t SENSOR_CALIBRATION[SENSOR_COUNT] = {
    256, 256, 256, 256, 256, 256, 256, 256, 256,
    256, 256, 256, 256, 256, 256, 256, 256, 256
};
#else
constexpr uint16_t SENSOR_CALIBRATION[SENSOR_COUNT] = {
    256, 256, 256, 256, 256, 256, 256, 256, 256,
    256, 256, 256, 256, 256, 256, 256, 256, 256
};
#endif

constexpr uint8_t SIGNALS_TO_USE = 5;

constexpr uint16_t SAMPLE_PERIOD_US = 50;

constexpr uint32_t MEASUREMENT_PERIOD_US = 12000;
}
