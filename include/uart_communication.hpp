#pragma once

#include <Arduino.h>

namespace uart_communication {

constexpr uint8_t MAX_PAYLOAD_LENGTH = 64;

enum class PacketType : uint8_t {
  IR_MEASUREMENT = 0x01,
  BLUETOOTH_TO_TEENSY = 0x02,
  TEENSY_TO_BLUETOOTH = 0x03,
};

//   A5 5A | type | length | sequence | payload | CRC16 (little-endian)
bool sendPacket(PacketType type, const uint8_t *payload,
                uint8_t payloadLength);

bool sendIrMeasurement(float bearingDegrees, float strength);

bool sendBluetoothToTeensy(const uint8_t *data, uint8_t length);

using PacketHandler = void (*)(PacketType type, const uint8_t *payload,
                               uint8_t payloadLength);

// Consumes available UART bytes and invokes handler for each valid frame.
void receivePackets(PacketHandler handler);

}  // namespace uart_communication
