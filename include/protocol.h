#pragma once

#include <Arduino.h>

namespace protocol {

enum class PacketType : uint8_t {
  BALL_MEASUREMENT = 0x01,
  RAW_SENSORS = 0x02,
  STATUS = 0x03,
  DEVICE_INFO = 0x04,
  COMMAND = 0x10,
  COMMAND_RESPONSE = 0x11,
};

enum MeasurementFlag : uint16_t {
  BALL_VALID = 1U << 0,
  SATURATED = 1U << 1,
  INTERFERENCE = 1U << 2,
  WEAK_SIGNAL = 1U << 3,
  MULTIPLE_CLUSTERS = 1U << 4,
  CALIBRATION_VALID = 1U << 5,
  RAW_OVERRUN = 1U << 6,
  SENSOR_FAULT = 1U << 7,
  POWER_WARNING = 1U << 8,
};

enum class Command : uint8_t {
  REQUEST_DEVICE_INFO = 0x01,
  REQUEST_STATUS = 0x02,
  SET_MEASUREMENT_RATE = 0x03,
  SET_RAW_RATE = 0x04,
  ENTER_DIAGNOSTIC_MODE = 0x05,
  EXIT_DIAGNOSTIC_MODE = 0x06,
  RELOAD_CALIBRATION = 0x07,
  PING = 0x08,
};

enum class CommandResult : uint8_t {
  SUCCESS = 0,
  UNKNOWN_COMMAND = 1,
  INVALID_ARGUMENT = 2,
  UNAVAILABLE = 3,
  FAILED = 4,
  MALFORMED = 5,
};

constexpr size_t HEADER_SIZE = 10;
constexpr size_t CRC_SIZE = 2;
constexpr size_t MAX_DECODED_SIZE = 96;
constexpr size_t MAX_ENCODED_SIZE = MAX_DECODED_SIZE + 2;

uint16_t crc16Ccitt(const uint8_t *data, size_t length);
size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output,
                  size_t outputCapacity);
size_t cobsDecode(const uint8_t *input, size_t length, uint8_t *output,
                  size_t outputCapacity);

void appendU8(uint8_t *buffer, size_t &position, uint8_t value);
void appendU16(uint8_t *buffer, size_t &position, uint16_t value);
void appendI16(uint8_t *buffer, size_t &position, int16_t value);
void appendU32(uint8_t *buffer, size_t &position, uint32_t value);
uint16_t readU16(const uint8_t *buffer);
uint32_t readU32(const uint8_t *buffer);

}  // namespace protocol
