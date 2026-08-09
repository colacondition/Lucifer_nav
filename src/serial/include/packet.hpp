#pragma once

#include <cstdint>
#include <cstring>

namespace serial_driver
{

// NUC -> MCU chassis velocity command.
struct ChassisCommandPacket
{
  uint8_t head[2] = {'H', 'L'};
  float vel_x = 0.0f;
  float vel_y = 0.0f;
  float vel_w = 0.0f;
  uint16_t crc16 = 0;
} __attribute__((packed));

// MCU -> NUC simulated referee data.
struct DecisionPacket
{
  uint8_t head[2] = {'H', 'L'};
  uint16_t current_hp = 0;
  uint8_t game_progress = 0;
  uint16_t stage_remain_time = 0;
  uint16_t crc16 = 0;
} __attribute__((packed));

template <typename T>
inline T bufferToStruct(const uint8_t * buffer)
{
  T result{};
  std::memcpy(&result, buffer, sizeof(T));
  return result;
}

template <typename T>
inline void structToBuffer(const T & inputStruct, uint8_t * outputArray)
{
  std::memcpy(outputArray, reinterpret_cast<const uint8_t *>(&inputStruct), sizeof(T));
}

}  // namespace serial_driver
