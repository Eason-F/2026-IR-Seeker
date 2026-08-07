#include <Arduino.h>
#include <math.h>

#include "config.hpp"
#include "vector.hpp"

HardwareSerial teensyUART(0);
uint16_t sensorCounts[config::SENSOR_COUNT] = {};
uint16_t sensorValues[config::SENSOR_COUNT] = {};
uint16_t sampleCount = 0;

ir::Vector2 irDirection;
float irBearingDegrees = 0.0F;
float irDirectionStrength = 0.0F;

uint32_t nextSampleUs = 0;

ir::Vector2 calculateIrDirection(const uint16_t (&samples)[config::SENSOR_COUNT]) {
  ir::Vector2 direction;
  bool selected[config::SENSOR_COUNT] = {};
  constexpr float SENSOR_ANGLE_DEGREES = 360.0F / static_cast<float>(config::SENSOR_COUNT);

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
    const float bearing = static_cast<float>(strongestSensor) * SENSOR_ANGLE_DEGREES;

    direction += ir::Vector2::fromBearingDegrees(bearing, strongestWeight);
  }

  return direction;
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

    if (sampleCount >= config::MEASUREMENT_PERIOD_US / config::SAMPLE_PERIOD_US) {
      memcpy(values, sensorCounts, sizeof(sensorCounts));

      irDirection = calculateIrDirection(values);
      irBearingDegrees = irDirection.bearingDegrees();
      irDirectionStrength = irDirection.magnitude();

      memset(sensorCounts, 0, sizeof(sensorCounts));
      sampleCount = 0;
    }
  }
}

void setup() {
  teensyUART.begin(115200);
  for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
    pinMode(config::SENSOR_PINS[sensor], config::SENSOR_INPUT_MODE);
  }
  nextSampleUs = micros();
}

void loop() {
  readSensorValues(sensorValues);
}
