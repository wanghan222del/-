#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "rm_can_bridge/q10_6_angle.hpp"

TEST(Q10_6Angle, Converts15DegTo960)
{
  EXPECT_EQ(rm_can_bridge::degToQ10_6Raw(15.0), 960);
}

TEST(Q10_6Angle, ConvertsNegative15DegToMinus960)
{
  EXPECT_EQ(rm_can_bridge::degToQ10_6Raw(-15.0), -960);
}

TEST(Q10_6Angle, Converts960To15Deg)
{
  EXPECT_NEAR(rm_can_bridge::q10_6RawToDeg(960), 15.0, 1e-6);
}

TEST(Q10_6Angle, ConvertsMinus960ToMinus15Deg)
{
  EXPECT_NEAR(rm_can_bridge::q10_6RawToDeg(static_cast<int16_t>(-960)), -15.0, 1e-6);
}

TEST(Q10_6Angle, EncodesBigEndianBytes)
{
  std::array<uint8_t, 8> data{};
  rm_can_bridge::writeInt16BE(data, 0, 960);
  EXPECT_EQ(data[0], 0x03);
  EXPECT_EQ(data[1], 0xC0);
}

TEST(Q10_6Angle, DecodesBigEndianBytes)
{
  const std::array<uint8_t, 8> data{0xFC, 0x40, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(rm_can_bridge::readInt16BE(data, 0), static_cast<int16_t>(-960));
  EXPECT_EQ(rm_can_bridge::readInt16BE(data, 2), static_cast<int16_t>(960));
}
