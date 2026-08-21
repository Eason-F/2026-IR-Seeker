#pragma once

#include <Arduino.h>

namespace uart_communication {

constexpr uint8_t MAX_PAYLOAD_LENGTH = 64;

enum class PacketType : uint8_t {
  IR_MEASUREMENT = 0x01,
  BLUETOOTH_TO_TEENSY = 0x02,
};

//   A5 5A | type | length | sequence | payload | CRC16 (little-endian)
bool sendPacket(PacketType type, const uint8_t *payload,
                uint8_t payloadLength);

// Payload: int16 bearing in centidegrees, then uint16 signal strength.
bool sendIrMeasurement(float bearingDegrees, float strength);

// Call this from the Bluetooth handling code to forward a received message.
bool sendBluetoothToTeensy(const uint8_t *data, uint8_t length);

}  // namespace uart_communication
