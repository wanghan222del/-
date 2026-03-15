#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "rm_can_bridge/usb_can_frame_parser.hpp"

TEST(UsbCanFrameParser, ParsesLegacyThirtyByteFrame)
{
  const std::vector<uint8_t> buffer{
    0x55, 0xAA, 0x1E, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00,
    0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00,
    0x00, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x88,
  };

  const auto result = rm_can_bridge::tryParseUsbCanFrame(buffer);

  ASSERT_EQ(result.action, rm_can_bridge::UsbCanParseAction::Parsed);
  EXPECT_EQ(result.frame.can_id, 0x402U);
  EXPECT_EQ(result.frame.consumed_bytes, 30U);
  EXPECT_EQ(
    result.frame.payload,
    (std::array<uint8_t, 8>{0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89}));
}

TEST(UsbCanFrameParser, ParsesSixteenByteFrameFromNewAdapter)
{
  const std::vector<uint8_t> buffer{
    0xAA, 0x11, 0x08, 0x02, 0x04, 0x00, 0x00,
    0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89,
    0x55,
  };

  const auto result = rm_can_bridge::tryParseUsbCanFrame(buffer);

  ASSERT_EQ(result.action, rm_can_bridge::UsbCanParseAction::Parsed);
  EXPECT_EQ(result.frame.can_id, 0x402U);
  EXPECT_EQ(result.frame.consumed_bytes, 16U);
  EXPECT_EQ(
    result.frame.payload,
    (std::array<uint8_t, 8>{0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89}));
}

TEST(UsbCanFrameParser, RequestsSkipOnUnknownLeadingByte)
{
  const std::vector<uint8_t> buffer{0x00, 0xAA, 0x11, 0x08};

  const auto result = rm_can_bridge::tryParseUsbCanFrame(buffer);

  EXPECT_EQ(result.action, rm_can_bridge::UsbCanParseAction::SkipByte);
}

TEST(UsbCanFrameParser, RequestsMoreDataForPartialNewFrame)
{
  const std::vector<uint8_t> buffer{0xAA, 0x11, 0x08, 0x02, 0x04};

  const auto result = rm_can_bridge::tryParseUsbCanFrame(buffer);

  EXPECT_EQ(result.action, rm_can_bridge::UsbCanParseAction::NeedMoreData);
}
