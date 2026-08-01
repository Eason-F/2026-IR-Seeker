#include "protocol.h"

namespace protocol {

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output,
                  size_t outputCapacity) {
  if (outputCapacity == 0) return 0;

  size_t readIndex = 0;
  size_t writeIndex = 1;
  size_t codeIndex = 0;
  uint8_t code = 1;

  while (readIndex < length) {
    if (input[readIndex] == 0) {
      if (codeIndex >= outputCapacity) return 0;
      output[codeIndex] = code;
      code = 1;
      codeIndex = writeIndex++;
      if (writeIndex > outputCapacity) return 0;
      ++readIndex;
    } else {
      if (writeIndex >= outputCapacity) return 0;
      output[writeIndex++] = input[readIndex++];
      ++code;
      if (code == 0xFF) {
        if (codeIndex >= outputCapacity) return 0;
        output[codeIndex] = code;
        code = 1;
        codeIndex = writeIndex++;
        if (writeIndex > outputCapacity) return 0;
      }
    }
  }

  if (codeIndex >= outputCapacity) return 0;
  output[codeIndex] = code;
  return writeIndex;
}

size_t cobsDecode(const uint8_t *input, size_t length, uint8_t *output,
                  size_t outputCapacity) {
  size_t readIndex = 0;
  size_t writeIndex = 0;

  while (readIndex < length) {
    const uint8_t code = input[readIndex++];
    if (code == 0) return 0;

    for (uint8_t i = 1; i < code; ++i) {
      if (readIndex >= length || writeIndex >= outputCapacity) return 0;
      output[writeIndex++] = input[readIndex++];
    }

    if (code != 0xFF && readIndex < length) {
      if (writeIndex >= outputCapacity) return 0;
      output[writeIndex++] = 0;
    }
  }
  return writeIndex;
}

void appendU8(uint8_t *buffer, size_t &position, uint8_t value) {
  buffer[position++] = value;
}

void appendU16(uint8_t *buffer, size_t &position, uint16_t value) {
  buffer[position++] = static_cast<uint8_t>(value);
  buffer[position++] = static_cast<uint8_t>(value >> 8);
}

void appendI16(uint8_t *buffer, size_t &position, int16_t value) {
  appendU16(buffer, position, static_cast<uint16_t>(value));
}

void appendU32(uint8_t *buffer, size_t &position, uint32_t value) {
  buffer[position++] = static_cast<uint8_t>(value);
  buffer[position++] = static_cast<uint8_t>(value >> 8);
  buffer[position++] = static_cast<uint8_t>(value >> 16);
  buffer[position++] = static_cast<uint8_t>(value >> 24);
}

uint16_t readU16(const uint8_t *buffer) {
  return static_cast<uint16_t>(buffer[0]) |
         (static_cast<uint16_t>(buffer[1]) << 8);
}

uint32_t readU32(const uint8_t *buffer) {
  return static_cast<uint32_t>(buffer[0]) |
         (static_cast<uint32_t>(buffer[1]) << 8) |
         (static_cast<uint32_t>(buffer[2]) << 16) |
         (static_cast<uint32_t>(buffer[3]) << 24);
}

}  // namespace protocol
