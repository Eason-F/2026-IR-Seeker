#include <Arduino.h>
#include <esp_system.h>
#include <math.h>

#include "config.h"
#include "protocol.h"

namespace {

HardwareSerial teensySerial(1);

uint16_t activeSamples[config::SENSOR_COUNT] = {};
uint16_t sensorValues[config::SENSOR_COUNT] = {};
uint16_t samplesInWindow = 0;
uint16_t measurementRateHz = config::DEFAULT_MEASUREMENT_RATE_HZ;
uint16_t rawRateHz = config::DEFAULT_RAW_RATE_HZ;

uint16_t txSequence = 0;
uint32_t nextSampleUs = 0;
uint32_t nextRawUs = 0;
uint32_t nextStatusUs = 0;
uint32_t nextTestPrintUs = 0;
uint32_t captureOverruns = 0;
bool captureOverrunInWindow = false;
uint32_t uartErrors = 0;
uint32_t measurementCount = 0;
uint32_t measurementRateWindowStartMs = 0;
uint16_t measuredRateHz = 0;
uint32_t faultySensorMask = 0;
uint16_t lastMeasurementFlags = protocol::CALIBRATION_VALID;

uint8_t rxEncoded[protocol::MAX_ENCODED_SIZE] = {};
size_t rxEncodedLength = 0;

struct Measurement {
  int16_t bearingCentidegrees = 0;
  uint16_t strength = 0;
  uint8_t confidence = 0;
  uint8_t peakSensor = 0xFF;
  uint32_t activeMask = 0;
  uint16_t flags = protocol::CALIBRATION_VALID;
};

Measurement lastMeasurement;

bool timeReached(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

void captureSample() {
  for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
    const bool electricalHigh = digitalRead(config::SENSOR_PINS[sensor]) == HIGH;
    const bool active = config::SENSOR_ACTIVE_LOW ? !electricalHigh
                                                  : electricalHigh;
    if (active && activeSamples[sensor] != UINT16_MAX) {
      ++activeSamples[sensor];
    }
  }
  ++samplesInWindow;
}

uint8_t countActiveClusters(uint32_t mask) {
  mask &= (1UL << config::SENSOR_COUNT) - 1UL;
  if (mask == 0) return 0;
  if (mask == ((1UL << config::SENSOR_COUNT) - 1UL)) return 1;

  uint8_t clusters = 0;
  for (uint8_t i = 0; i < config::SENSOR_COUNT; ++i) {
    const uint8_t previous = (i + config::SENSOR_COUNT - 1) % config::SENSOR_COUNT;
    const bool currentOn = (mask & (1UL << i)) != 0;
    const bool previousOn = (mask & (1UL << previous)) != 0;
    if (currentOn && !previousOn) ++clusters;
  }
  return clusters;
}

Measurement calculateMeasurement() {
  Measurement result;
  if (samplesInWindow == 0) return result;

  float vectorX = 0.0F;
  float vectorY = 0.0F;
  float totalWeight = 0.0F;
  uint16_t peakValue = 0;
  uint8_t activeCount = 0;
  bool continuouslyActive = false;

  const uint16_t activeThreshold = max<uint16_t>(2, samplesInWindow / 20);

  for (uint8_t i = 0; i < config::SENSOR_COUNT; ++i) {
    uint32_t calibrated =
        (static_cast<uint32_t>(activeSamples[i]) * config::SENSOR_GAIN_Q8[i]) >> 8;
    calibrated = min<uint32_t>(calibrated, samplesInWindow);
    sensorValues[i] = static_cast<uint16_t>(
        (calibrated * 65535UL) / static_cast<uint32_t>(samplesInWindow));

    if (calibrated >= activeThreshold) {
      result.activeMask |= 1UL << i;
      ++activeCount;
    }
    if (calibrated > peakValue) {
      peakValue = static_cast<uint16_t>(calibrated);
      result.peakSensor = i;
    }
    if (calibrated * 100UL >= static_cast<uint32_t>(samplesInWindow) * 95UL) {
      continuouslyActive = true;
    }

    const float angle = (2.0F * PI * static_cast<float>(i)) /
                        static_cast<float>(config::SENSOR_COUNT);
    vectorX += static_cast<float>(calibrated) * cosf(angle);
    vectorY += static_cast<float>(calibrated) * sinf(angle);
    totalWeight += static_cast<float>(calibrated);
  }

  result.strength = static_cast<uint16_t>(
      (static_cast<uint32_t>(peakValue) * 65535UL) / samplesInWindow);

  if (totalWeight > 0.0F) {
    float bearing = atan2f(vectorY, vectorX) * (18000.0F / PI);
    if (bearing > 18000.0F) bearing -= 36000.0F;
    if (bearing <= -18000.0F) bearing += 36000.0F;
    result.bearingCentidegrees = static_cast<int16_t>(lroundf(bearing));

    const float coherence =
        sqrtf(vectorX * vectorX + vectorY * vectorY) / totalWeight;
    const float signalScore = min(1.0F, static_cast<float>(peakValue) /
                                            (samplesInWindow * 0.35F));
    const float confidence = (0.70F * coherence + 0.30F * signalScore) * 255.0F;
    result.confidence = static_cast<uint8_t>(constrain(lroundf(confidence), 0L, 255L));
  }

  if (peakValue >= activeThreshold && !continuouslyActive) {
    result.flags |= protocol::BALL_VALID;
  }
  if (result.strength < 8000 && peakValue > 0) {
    result.flags |= protocol::WEAK_SIGNAL;
  }
  if (activeCount >= 12) {
    result.flags |= protocol::SATURATED;
  }
  if (continuouslyActive) {
    result.flags |= protocol::INTERFERENCE;
  }
  if (countActiveClusters(result.activeMask) > 1) {
    result.flags |= protocol::MULTIPLE_CLUSTERS;
  }
  if (captureOverrunInWindow) {
    result.flags |= protocol::RAW_OVERRUN;
  }
  if (faultySensorMask != 0) {
    result.flags |= protocol::SENSOR_FAULT;
  }

  return result;
}

void sendPacket(protocol::PacketType type, uint32_t timestampUs,
                const uint8_t *payload, uint16_t payloadLength) {
  if (protocol::HEADER_SIZE + payloadLength + protocol::CRC_SIZE >
      protocol::MAX_DECODED_SIZE) {
    return;
  }

  uint8_t decoded[protocol::MAX_DECODED_SIZE];
  size_t position = 0;
  protocol::appendU8(decoded, position, config::PROTOCOL_VERSION);
  protocol::appendU8(decoded, position, static_cast<uint8_t>(type));
  protocol::appendU16(decoded, position, payloadLength);
  protocol::appendU16(decoded, position, txSequence++);
  protocol::appendU32(decoded, position, timestampUs);
  memcpy(decoded + position, payload, payloadLength);
  position += payloadLength;

  const uint16_t crc = protocol::crc16Ccitt(decoded, position);
  protocol::appendU16(decoded, position, crc);

  uint8_t encoded[protocol::MAX_ENCODED_SIZE];
  const size_t encodedLength =
      protocol::cobsEncode(decoded, position, encoded, sizeof(encoded));
  if (encodedLength == 0) return;

  teensySerial.write(encoded, encodedLength);
  teensySerial.write(static_cast<uint8_t>(0));
}

void sendBallMeasurement(uint32_t timestampUs) {
  uint8_t payload[12];
  size_t position = 0;
  protocol::appendI16(payload, position, lastMeasurement.bearingCentidegrees);
  protocol::appendU16(payload, position, lastMeasurement.strength);
  protocol::appendU8(payload, position, lastMeasurement.confidence);
  protocol::appendU8(payload, position, lastMeasurement.peakSensor);
  protocol::appendU32(payload, position, lastMeasurement.activeMask);
  protocol::appendU16(payload, position, lastMeasurement.flags);
  sendPacket(protocol::PacketType::BALL_MEASUREMENT, timestampUs, payload,
             sizeof(payload));
}

void sendRawSensors(uint32_t timestampUs) {
  uint8_t payload[42];
  size_t position = 0;
  const uint16_t captureDurationUs =
      static_cast<uint16_t>(1000000UL / measurementRateHz);
  protocol::appendU16(payload, position, captureDurationUs);
  protocol::appendU16(payload, position,
                      config::SAMPLE_RATE_HZ / measurementRateHz);
  for (uint8_t i = 0; i < config::SENSOR_COUNT; ++i) {
    protocol::appendU16(payload, position, sensorValues[i]);
  }
  protocol::appendU16(payload, position, lastMeasurement.flags);
  sendPacket(protocol::PacketType::RAW_SENSORS, timestampUs, payload,
             sizeof(payload));
}

void printRawSensors() {
  Serial0.print("RAW");
  Serial0.print(',');
  Serial0.print(lastMeasurement.bearingCentidegrees / 100.0F, 2);
  Serial0.print(',');
  Serial0.print((lastMeasurement.flags & protocol::BALL_VALID) != 0 ? 1 : 0);
  for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
    Serial0.print(',');
    Serial0.print(sensorValues[sensor]);
  }
  Serial0.println();
}

void sendStatus(uint32_t timestampUs) {
  uint8_t payload[20];
  size_t position = 0;
  protocol::appendU32(payload, position, millis() / 1000UL);
  protocol::appendU16(payload, position, 0);  // Supply voltage unavailable.
  protocol::appendU16(payload, position, measuredRateHz);
  protocol::appendU16(payload, position,
                      static_cast<uint16_t>(min<uint32_t>(uartErrors, UINT16_MAX)));
  protocol::appendU16(
      payload, position,
      static_cast<uint16_t>(min<uint32_t>(captureOverruns, UINT16_MAX)));
  protocol::appendU16(payload, position,
                      esp_reset_reason() == ESP_RST_POWERON ? 0 : 1);
  protocol::appendU32(payload, position, faultySensorMask);
  protocol::appendU16(payload, position, lastMeasurementFlags);
  sendPacket(protocol::PacketType::STATUS, timestampUs, payload, sizeof(payload));
}

void sendDeviceInfo(uint32_t timestampUs) {
  uint8_t payload[15];
  size_t position = 0;
  protocol::appendU8(payload, position, config::HARDWARE_VERSION);
  protocol::appendU8(payload, position, config::FIRMWARE_MAJOR);
  protocol::appendU8(payload, position, config::FIRMWARE_MINOR);
  protocol::appendU8(payload, position, config::FIRMWARE_PATCH);
  protocol::appendU8(payload, position, config::SENSOR_COUNT);
  protocol::appendU16(payload, position, 0x0001);  // Raw stream supported.
  protocol::appendU32(payload, position, 0);       // Optional build identifier.
  protocol::appendU32(payload, position,
                      static_cast<uint32_t>(ESP.getEfuseMac()));
  sendPacket(protocol::PacketType::DEVICE_INFO, timestampUs, payload,
             sizeof(payload));
}

void sendCommandResponse(uint8_t command, uint8_t requestId,
                         protocol::CommandResult result,
                         const uint8_t *data = nullptr, uint8_t dataLength = 0) {
  uint8_t payload[32];
  size_t position = 0;
  protocol::appendU8(payload, position, command);
  protocol::appendU8(payload, position, requestId);
  protocol::appendU8(payload, position, static_cast<uint8_t>(result));
  if (data != nullptr && dataLength != 0) {
    memcpy(payload + position, data, dataLength);
    position += dataLength;
  }
  sendPacket(protocol::PacketType::COMMAND_RESPONSE, micros(), payload,
             static_cast<uint16_t>(position));
}

void handleCommand(const uint8_t *payload, uint16_t length) {
  if (length < 2) {
    sendCommandResponse(0, 0, protocol::CommandResult::MALFORMED);
    return;
  }

  const uint8_t commandByte = payload[0];
  const uint8_t requestId = payload[1];
  const protocol::Command command = static_cast<protocol::Command>(commandByte);

  switch (command) {
    case protocol::Command::REQUEST_DEVICE_INFO:
      sendCommandResponse(commandByte, requestId, protocol::CommandResult::SUCCESS);
      sendDeviceInfo(micros());
      break;

    case protocol::Command::REQUEST_STATUS:
      sendCommandResponse(commandByte, requestId, protocol::CommandResult::SUCCESS);
      sendStatus(micros());
      break;

    case protocol::Command::SET_MEASUREMENT_RATE: {
      if (length != 4) {
        sendCommandResponse(commandByte, requestId,
                            protocol::CommandResult::MALFORMED);
        break;
      }
      const uint16_t requestedRate = protocol::readU16(payload + 2);
      if (requestedRate != 100 && requestedRate != 250 && requestedRate != 500) {
        sendCommandResponse(commandByte, requestId,
                            protocol::CommandResult::INVALID_ARGUMENT);
        break;
      }
      measurementRateHz = requestedRate;
      sendCommandResponse(commandByte, requestId, protocol::CommandResult::SUCCESS);
      break;
    }

    case protocol::Command::SET_RAW_RATE: {
      if (length != 4) {
        sendCommandResponse(commandByte, requestId,
                            protocol::CommandResult::MALFORMED);
        break;
      }
      const uint16_t requestedRate = protocol::readU16(payload + 2);
      if (requestedRate > 100) {
        sendCommandResponse(commandByte, requestId,
                            protocol::CommandResult::INVALID_ARGUMENT);
        break;
      }
      rawRateHz = requestedRate;
      nextRawUs = micros();
      sendCommandResponse(commandByte, requestId, protocol::CommandResult::SUCCESS);
      break;
    }

    case protocol::Command::ENTER_DIAGNOSTIC_MODE:
      rawRateHz = 25;
      nextRawUs = micros();
      sendCommandResponse(commandByte, requestId, protocol::CommandResult::SUCCESS);
      break;

    case protocol::Command::EXIT_DIAGNOSTIC_MODE:
      rawRateHz = 0;
      sendCommandResponse(commandByte, requestId, protocol::CommandResult::SUCCESS);
      break;

    case protocol::Command::RELOAD_CALIBRATION:
      // Calibration is currently compiled into config.h.
      sendCommandResponse(commandByte, requestId,
                          protocol::CommandResult::UNAVAILABLE);
      break;

    case protocol::Command::PING:
      sendCommandResponse(commandByte, requestId, protocol::CommandResult::SUCCESS,
                          payload + 2, static_cast<uint8_t>(length - 2));
      break;

    default:
      sendCommandResponse(commandByte, requestId,
                          protocol::CommandResult::UNKNOWN_COMMAND);
      break;
  }
}

void processReceivedFrame() {
  uint8_t decoded[protocol::MAX_DECODED_SIZE];
  const size_t decodedLength = protocol::cobsDecode(
      rxEncoded, rxEncodedLength, decoded, sizeof(decoded));

  if (decodedLength < protocol::HEADER_SIZE + protocol::CRC_SIZE) {
    ++uartErrors;
    return;
  }

  const uint16_t payloadLength = protocol::readU16(decoded + 2);
  const size_t expectedLength =
      protocol::HEADER_SIZE + payloadLength + protocol::CRC_SIZE;
  if (decoded[0] != config::PROTOCOL_VERSION ||
      decodedLength != expectedLength) {
    ++uartErrors;
    return;
  }

  const uint16_t receivedCrc =
      protocol::readU16(decoded + decodedLength - protocol::CRC_SIZE);
  const uint16_t calculatedCrc =
      protocol::crc16Ccitt(decoded, decodedLength - protocol::CRC_SIZE);
  if (receivedCrc != calculatedCrc) {
    ++uartErrors;
    return;
  }

  if (decoded[1] == static_cast<uint8_t>(protocol::PacketType::COMMAND)) {
    handleCommand(decoded + protocol::HEADER_SIZE, payloadLength);
  }
}

void receiveFromTeensy() {
  while (teensySerial.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(teensySerial.read());
    if (byte == 0) {
      if (rxEncodedLength != 0) processReceivedFrame();
      rxEncodedLength = 0;
    } else if (rxEncodedLength < sizeof(rxEncoded)) {
      rxEncoded[rxEncodedLength++] = byte;
    } else {
      rxEncodedLength = 0;
      ++uartErrors;
    }
  }
}

void finishMeasurementWindow(uint32_t timestampUs) {
  lastMeasurement = calculateMeasurement();
  lastMeasurementFlags = lastMeasurement.flags;
  sendBallMeasurement(timestampUs);
  ++measurementCount;

  memset(activeSamples, 0, sizeof(activeSamples));
  samplesInWindow = 0;
  captureOverrunInWindow = false;
}

}  // namespace

void setup() {
  for (uint8_t sensor = 0; sensor < config::SENSOR_COUNT; ++sensor) {
    pinMode(config::SENSOR_PINS[sensor], config::SENSOR_INPUT_MODE);
  }
  teensySerial.begin(config::UART_BAUD, SERIAL_8N1, config::PIN_UART_RX,
                     config::PIN_UART_TX);
  Serial0.begin(config::TEST_SERIAL_BAUD);

  Serial0.println("IR seeker raw sensor test output");
  Serial0.println(
      "RAW,DirectionDeg,Valid,S0,S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12,S13,S14,S15,S16,S17");

  const uint32_t nowUs = micros();
  nextSampleUs = nowUs;
  nextRawUs = nowUs + 1000000UL;
  nextStatusUs = nowUs + 1000000UL;
  nextTestPrintUs = nowUs;
  measurementRateWindowStartMs = millis();

  sendDeviceInfo(nowUs);
}

void loop() {
  receiveFromTeensy();

  const uint32_t nowUs = micros();
  constexpr uint32_t samplePeriodUs = 1000000UL / config::SAMPLE_RATE_HZ;

  if (timeReached(nowUs, nextSampleUs)) {
    if (static_cast<uint32_t>(nowUs - nextSampleUs) >= samplePeriodUs * 3UL) {
      const uint32_t missed = (nowUs - nextSampleUs) / samplePeriodUs;
      captureOverruns += missed;
      captureOverrunInWindow = true;
      nextSampleUs = nowUs;
    }

    captureSample();
    nextSampleUs += samplePeriodUs;

    const uint16_t targetSamples = config::SAMPLE_RATE_HZ / measurementRateHz;
    if (samplesInWindow >= targetSamples) {
      finishMeasurementWindow(nowUs);
    }
  }

  if (rawRateHz != 0 && timeReached(nowUs, nextRawUs)) {
    sendRawSensors(nowUs);
    nextRawUs += 1000000UL / rawRateHz;
  }

  if (timeReached(nowUs, nextStatusUs)) {
    sendStatus(nowUs);
    nextStatusUs += 1000000UL / config::STATUS_RATE_HZ;
  }

  if (timeReached(nowUs, nextTestPrintUs)) {
    printRawSensors();
    nextTestPrintUs += 1000000UL / config::TEST_PRINT_RATE_HZ;
  }

  const uint32_t nowMs = millis();
  if (nowMs - measurementRateWindowStartMs >= 1000UL) {
    const uint32_t elapsedMs = nowMs - measurementRateWindowStartMs;
    measuredRateHz = static_cast<uint16_t>((measurementCount * 1000UL) / elapsedMs);
    measurementCount = 0;
    measurementRateWindowStartMs = nowMs;
  }
}
