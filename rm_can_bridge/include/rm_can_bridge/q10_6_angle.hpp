#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rm_can_bridge
{

inline int16_t clampRoundedToInt16(double value)
{
  const int64_t raw = static_cast<int64_t>(std::llround(value));
  if (raw > 32767) {
    return 32767;
  }
  if (raw < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(raw);
}

inline int16_t degToQ10_6Raw(double degrees)
{
  return clampRoundedToInt16(degrees * 64.0);
}

inline int16_t radToQ10_6Raw(double radians)
{
  return clampRoundedToInt16(radians * 64.0);
}

inline double q10_6RawToDeg(int16_t raw)
{
  return static_cast<double>(raw) / 64.0;
}

template<size_t N>
inline void writeInt16BE(std::array<uint8_t, N> & data, size_t offset, int16_t value)
{
  data[offset] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF);
  data[offset + 1] = static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFF);
}

template<size_t N>
inline int16_t readInt16BE(const std::array<uint8_t, N> & data, size_t offset)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(data[offset]) << 8) |
    static_cast<uint16_t>(data[offset + 1]));
}

inline int16_t readInt16BE(const uint8_t * data, size_t offset)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(data[offset]) << 8) |
    static_cast<uint16_t>(data[offset + 1]));
}

}  // namespace rm_can_bridge
