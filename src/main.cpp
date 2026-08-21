#include <Arduino.h>
#include <math.h>
#include <cstring>
#include <cstdint>

#include "config.hpp"
#include "uart_communication.hpp"
#include "vector.hpp"

uint16_t sensorCounts[config::SENSOR_COUNT] = {};
float sensorStrengths[config::SENSOR_COUNT] = {};

uint16_t sampleCount = 0;

ir::Vector2 irDirection;
float irBearingDegrees = 0.0F;
float irDirectionConfidence = 0.0F;

float irSignalStrength = 0.0F;
float filteredIrSignalStrength = 0.0F;

uint32_t nextSampleUs = 0;

namespace {

constexpr float SIGNAL_FILTER_ALPHA = 0.2F;

float normalizeBearingDegrees(float bearingDegrees) {
  while (bearingDegrees > 180.0F) {
    bearingDegrees -= 360.0F;
  }

  while (bearingDegrees <= -180.0F) {
    bearingDegrees += 360.0F;
  }

  return bearingDegrees;
}

}  // namespace

ir::Vector2 calculateIrDirection(
    const float (&strengths)[config::SENSOR_COUNT]) {

  ir::Vector2 direction;
  bool selected[config::SENSOR_COUNT] = {};

  constexpr float SENSOR_ANGLE_DEGREES =
      360.0F / static_cast<float>(config::SENSOR_COUNT);

  for (uint8_t rank = 0; rank < config::SIGNALS_TO_USE; ++rank) {
    uint8_t strongestSensor = config::SENSOR_COUNT;
    float strongestWeight = 0.0F;

    for (uint8_t sensor = 0;
         sensor < config::SENSOR_COUNT;
         ++sensor) {

      if (selected[sensor]) {
        continue;
      }

      const float weight =
          strengths[sensor] *
          static_cast<float>(config::SENSOR_CALIBRATION[sensor]) /
          256.0F;

      if (weight > strongestWeight) {
        strongestWeight = weight;
        strongestSensor = sensor;
      }
    }

    if (strongestSensor == config::SENSOR_COUNT) {
      break;
    }

    selected[strongestSensor] = true;

    float bearing =
        static_cast<float>(strongestSensor + 1) *
        SENSOR_ANGLE_DEGREES;

    if (bearing >= 360.0F) {
      bearing -= 360.0F;
    }

    direction +=
        ir::Vector2::fromBearingDegrees(
            bearing,
            strongestWeight);
  }

  return direction;
}

float calculateSignalStrength(
    const float (&strengths)[config::SENSOR_COUNT]) {

  float strength = 0.0F;

  for (uint8_t sensor = 0;
       sensor < config::SENSOR_COUNT;
       ++sensor) {

    strength +=
        strengths[sensor] *
        static_cast<float>(config::SENSOR_CALIBRATION[sensor]) /
        256.0F;
  }

  strength =
      (strength / static_cast<float>(config::SENSOR_COUNT)) *
      100.0F;

  constexpr float SIGNAL_MIN = 20.0F;
  constexpr float SIGNAL_MAX = 50.0F;

  strength =
      ((strength - SIGNAL_MIN) /
       (SIGNAL_MAX - SIGNAL_MIN)) *
      100.0F;

  return constrain(strength, 0.0F, 100.0F);
}

void calculateSensorStrengths(
    const uint16_t (&counts)[config::SENSOR_COUNT],
    uint16_t samplesTaken) {

  if (samplesTaken == 0) {
    memset(sensorStrengths, 0, sizeof(sensorStrengths));
    return;
  }

  const float samples =
      static_cast<float>(samplesTaken);

  for (uint8_t sensor = 0;
       sensor < config::SENSOR_COUNT;
       ++sensor) {

    sensorStrengths[sensor] =
        static_cast<float>(counts[sensor]) /
        samples;
  }
}

void updateFilteredSignalStrength(float newStrength) {
  filteredIrSignalStrength +=
      SIGNAL_FILTER_ALPHA *
      (newStrength - filteredIrSignalStrength);
}

void printCalibrationHeader() {

  Serial0.print("samples");

  for (uint8_t sensor = 0;
       sensor < config::SENSOR_COUNT;
       ++sensor) {

    Serial0.print(",sensor");
    Serial0.print(sensor + 1);

    Serial0.print(",percent");

    Serial0.print(",calibrated");
  }

  Serial0.println();
}

void printCalibrationValues() {

  Serial0.print(sampleCount);

  for (uint8_t sensor = 0;
       sensor < config::SENSOR_COUNT;
       ++sensor) {

    const float calibratedStrength =
        sensorStrengths[sensor] *
        static_cast<float>(config::SENSOR_CALIBRATION[sensor]) /
        256.0F;

    Serial0.print(',');
    Serial0.print(sensorCounts[sensor]);

    Serial0.print(',');
    Serial0.print(sensorStrengths[sensor] * 100.0F, 2);

    Serial0.print(',');
    Serial0.print(calibratedStrength, 3);
  }

  Serial0.println();
}

void printDebugValues() {

  Serial0.print("bearing=");
  Serial0.print(irBearingDegrees, 2);

  Serial0.print(",strength=");
  Serial0.print(filteredIrSignalStrength, 2);

  Serial0.print(",confidence=");
  Serial0.println(irDirectionConfidence, 3);
}

void sampleSensors() {

  for (uint8_t sensor = 0;
       sensor < config::SENSOR_COUNT;
       ++sensor) {

    const bool pinHigh =
        digitalRead(config::SENSOR_PINS[sensor]);

    const bool active =
        config::SENSOR_ACTIVE_LOW ? !pinHigh : pinHigh;

    if (active && sensorCounts[sensor] < UINT16_MAX) {
      ++sensorCounts[sensor];
    }
  }

  if (sampleCount < UINT16_MAX) {
    ++sampleCount;
  }
}

void processMeasurement() {

  calculateSensorStrengths(
      sensorCounts,
      sampleCount);

  irDirection =
      calculateIrDirection(sensorStrengths);

  irBearingDegrees =
      normalizeBearingDegrees(
          irDirection.bearingDegrees());

  irDirectionConfidence =
      irDirection.magnitude();

  irSignalStrength =
      calculateSignalStrength(sensorStrengths);

  updateFilteredSignalStrength(
      irSignalStrength);

  if (config::CALIBRATION_MODE) {

    printCalibrationValues();

  } else if (config::DEBUG_OUTPUT_ENABLED) {

    printDebugValues();

  } else {

    uart_communication::sendIrMeasurement(
        irBearingDegrees,
        filteredIrSignalStrength);
  }

  memset(
      sensorCounts,
      0,
      sizeof(sensorCounts));

  sampleCount = 0;
}

void readSensorValues() {

  const uint32_t now = micros();

  if (static_cast<int32_t>(
          now - nextSampleUs) >= 0) {

    nextSampleUs +=
        config::SAMPLE_PERIOD_US;

    sampleSensors();

    const uint32_t requiredSamples =
        config::MEASUREMENT_PERIOD_US /
        config::SAMPLE_PERIOD_US;

    if (sampleCount >= requiredSamples) {
      processMeasurement();
    }
  }
}

void setup() {

  const uint32_t uartBaud =
      config::CALIBRATION_MODE || config::DEBUG_OUTPUT_ENABLED
          ? config::TEST_SERIAL_BAUD
          : config::UART_BAUD;

  Serial0.begin(
      uartBaud,
      SERIAL_8N1);

  for (uint8_t sensor = 0;
       sensor < config::SENSOR_COUNT;
       ++sensor) {

    pinMode(
        config::SENSOR_PINS[sensor],
        config::SENSOR_INPUT_MODE);
  }

  if (config::CALIBRATION_MODE) {
    printCalibrationHeader();
  }

  nextSampleUs = micros();
}

void loop() {
  readSensorValues();
}