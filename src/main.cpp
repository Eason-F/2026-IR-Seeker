#include <Arduino.h>
#include <math.h>

#include "config.hpp"
#include "uart_communication.hpp"
#include "vector.hpp"

uint16_t sensorCounts[config::SENSOR_COUNT] = {};
uint16_t sensorValues[config::SENSOR_COUNT] = {};
uint16_t sampleCount = 0;

ir::Vector2 irDirection;
float irBearingDegrees = 0.0F;
float irDirectionStrength = 0.0F;

uint32_t nextSampleUs = 0;
uint32_t nextDebugPrintUs = 0;
bool measurementAvailable = false;

ir::Vector2 calculateIrDirection(
    const uint16_t (&samples)[config::SENSOR_COUNT]) {
  ir::Vector2 direction;
  bool selected[config::SENSOR_COUNT] = {};
  constexpr float SENSOR_ANGLE_DEGREES =
      360.0F / static_cast<float>(config::SENSOR_COUNT);

  for (uint8_t rank = 0; rank < config::SIGNALS_TO_USE; ++rank) {
    uint8_t strongestSensor = config::SENSOR_COUNT;
    float strongestWeight = 0.0F;

    for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
      if (selected[sensor]) continue;

      // Calibration is Q8: a value of 256 represents a gain of 1.0.
      const float weight =
          static_cast<float>(samples[sensor]) *
          static_cast<float>(config::SENSOR_CALIBRATION[sensor]) / 256.0F;
      if (weight > strongestWeight) {
        strongestWeight = weight;
        strongestSensor = sensor;
      }
    }

    if (strongestSensor == config::SENSOR_COUNT) break;

    selected[strongestSensor] = true;
    const float bearing =
        static_cast<float>(strongestSensor + 1) * SENSOR_ANGLE_DEGREES;

    direction += ir::Vector2::fromBearingDegrees(bearing, strongestWeight);
  }

  return direction;
}

void printDebugValues(const uint16_t (&values)[config::SENSOR_COUNT],
                      const ir::Vector2 &direction, float bearingDegrees,
                      float strength) {
  Serial0.print("IR");
  // for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
  //   if (sensor == 17) {
  //     Serial0.print(',');
  //     Serial0.print(values[sensor]);
  //   }
  // }
  // Serial0.println();

  Serial0.print(',');
  Serial0.print(direction.x(), 3);
  Serial0.print(',');
  Serial0.print(direction.y(), 3);
  Serial0.print(',');
  Serial0.print(bearingDegrees, 2);
  Serial0.print(',');
  Serial0.println(strength, 3);
}

void sampleSensors() {
  for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
    const bool pinHigh = digitalRead(config::SENSOR_PINS[sensor]);
    const bool active = config::SENSOR_ACTIVE_LOW ? !pinHigh : pinHigh;

    if (active && sensorCounts[sensor] < UINT16_MAX) {
      ++sensorCounts[sensor];
    }
  }
  ++sampleCount;
}

void readSensorValues(uint16_t (&values)[config::SENSOR_COUNT]) {
  uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleUs) >= 0) {
    nextSampleUs += config::SAMPLE_PERIOD_US;
    sampleSensors();

    if (sampleCount >=
        config::MEASUREMENT_PERIOD_US / config::SAMPLE_PERIOD_US) {
      memcpy(values, sensorCounts, sizeof(sensorCounts));

      irDirection = calculateIrDirection(values);
      irBearingDegrees = irDirection.bearingDegrees();
      irDirectionStrength = irDirection.magnitude();
      measurementAvailable = true;

      uart_communication::sendIrMeasurement(irBearingDegrees, irDirectionStrength);

      memset(sensorCounts, 0, sizeof(sensorCounts));
      sampleCount = 0;
    }
  }
}

void setup() {
  const uint32_t uartBaud = config::DEBUG_OUTPUT_ENABLED
                                ? config::TEST_SERIAL_BAUD
                                : config::UART_BAUD;
  Serial0.begin(uartBaud, SERIAL_8N1);

  if (config::DEBUG_OUTPUT_ENABLED) {
    Serial0.print("type");
    for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
      Serial0.print(",sensor");
      Serial0.print(sensor);
    }
    Serial0.println(",direction_x,direction_y,bearing_degrees,strength");
  }

  for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
    pinMode(config::SENSOR_PINS[sensor], config::SENSOR_INPUT_MODE);
  }
  nextSampleUs = micros();
  nextDebugPrintUs = nextSampleUs;
}

void loop() {
  readSensorValues(sensorValues);

  const uint32_t now = micros();
  if (config::DEBUG_OUTPUT_ENABLED && measurementAvailable &&
      static_cast<int32_t>(now - nextDebugPrintUs) >= 0) {
    nextDebugPrintUs += 1000000UL / config::TEST_PRINT_RATE_HZ;
    printDebugValues(sensorValues, irDirection, irBearingDegrees,
                     irDirectionStrength);
  }
}
