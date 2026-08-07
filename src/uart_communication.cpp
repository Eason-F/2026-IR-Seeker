#include "uart_communication.hpp"

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>

#include "config.hpp"

namespace uart_communication {
namespace {

constexpr uint8_t MARKER_0 = 0xA5;
constexpr uint8_t MARKER_1 = 0x5A;

constexpr size_t TYPE_OFFSET = 2;
constexpr size_t LENGTH_OFFSET = 3;
constexpr size_t SEQUENCE_OFFSET = 4;
constexpr size_t PAYLOAD_OFFSET = 5;
constexpr size_t CRC_LENGTH = sizeof(uint16_t);
constexpr size_t MAX_FRAME_LENGTH =
    PAYLOAD_OFFSET + MAX_PAYLOAD_LENGTH + CRC_LENGTH;

uint8_t txSequence = 0;

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void writeU16(uint8_t *destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8);
}

int16_t encodeBearing(float bearingDegrees) {
  long value = std::lround(bearingDegrees * 100.0F);
  if (value < -18000L) value = -18000L;
  if (value > 18000L) value = 18000L;
  return static_cast<int16_t>(value);
}

uint16_t encodeStrength(float strength) {
  long value = std::lround(strength);
  if (value < 0L) value = 0L;
  if (value > UINT16_MAX) value = UINT16_MAX;
  return static_cast<uint16_t>(value);
}

}  // namespace

bool sendPacket(PacketType type, const uint8_t *payload,
                uint8_t payloadLength) {
  if (config::DEBUG_OUTPUT_ENABLED ||
      payloadLength > MAX_PAYLOAD_LENGTH ||
      (payloadLength != 0 && payload == nullptr)) {
    return false;
  }

  uint8_t frame[MAX_FRAME_LENGTH];
  frame[0] = MARKER_0;
  frame[1] = MARKER_1;
  frame[TYPE_OFFSET] = static_cast<uint8_t>(type);
  frame[LENGTH_OFFSET] = payloadLength;
  frame[SEQUENCE_OFFSET] = txSequence;

  if (payloadLength != 0) {
    memcpy(frame + PAYLOAD_OFFSET, payload, payloadLength);
  }

  const size_t crcPosition = PAYLOAD_OFFSET + payloadLength;
  const uint16_t crc =
      crc16Ccitt(frame + TYPE_OFFSET, 3 + payloadLength);
  writeU16(frame + crcPosition, crc);

  const size_t frameLength = crcPosition + CRC_LENGTH;
  const bool sent = Serial0.write(frame, frameLength) == frameLength;
  if (sent) ++txSequence;
  return sent;
}

bool sendIrMeasurement(float bearingDegrees, float strength) {
  uint8_t payload[4];
  writeU16(payload, static_cast<uint16_t>(encodeBearing(bearingDegrees)));
  writeU16(payload + sizeof(uint16_t), encodeStrength(strength));
  return sendPacket(PacketType::IR_MEASUREMENT, payload, sizeof(payload));
}

bool sendBluetoothToTeensy(const uint8_t *data, uint8_t length) {
  return sendPacket(PacketType::BLUETOOTH_TO_TEENSY, data, length);
}

}  // namespace uart_communication
