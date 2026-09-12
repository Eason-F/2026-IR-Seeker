#pragma once

#include <cstdint>

#pragma pack(push, 1)

struct RobotPacket {
  int16_t x;
  int16_t y;
  int16_t heading;
  int16_t ballBearing;
  uint8_t ballStrength;
  uint8_t attackScore;
  uint8_t state;
  uint8_t role;
  uint8_t flags;
  uint8_t sequence;
};

#pragma pack(pop)

static_assert(sizeof(RobotPacket) == 14,
              "RobotPacket wire format must be exactly 14 bytes");
