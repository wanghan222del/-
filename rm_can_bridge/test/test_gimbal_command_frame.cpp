#include <gtest/gtest.h>

#include <array>

#include "rm_can_bridge/gimbal_command_frame.hpp"

TEST(GimbalCommandFrame, SendsFreshAimWhenTrackingIsFresh)
{
  const auto payload = rm_can_bridge::buildGimbalCommandPayload(
    rm_can_bridge::GimbalCommandInput{
    false, true, false, false, false, true, 960, -640});

  EXPECT_EQ(payload, (std::array<uint8_t, 8>{0x03, 0xC0, 0xFD, 0x80, 0x00, 0x01, 0x00, 0x00}));
}

TEST(GimbalCommandFrame, HoldsLastAimWhenTargetIsLostAndConfiguredToHold)
{
  const auto payload = rm_can_bridge::buildGimbalCommandPayload(
    rm_can_bridge::GimbalCommandInput{
    false, false, false, false, true, true, 960, -640});

  EXPECT_EQ(payload, (std::array<uint8_t, 8>{0x03, 0xC0, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x00}));
}

TEST(GimbalCommandFrame, ZeroesAimWhenTargetIsLostAndConfiguredToZero)
{
  const auto payload = rm_can_bridge::buildGimbalCommandPayload(
    rm_can_bridge::GimbalCommandInput{
    false, false, false, false, false, true, 960, -640});

  EXPECT_EQ(payload, (std::array<uint8_t, 8>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
}

TEST(GimbalCommandFrame, KeepsFireDisabledWhenAimIsStale)
{
  const auto payload = rm_can_bridge::buildGimbalCommandPayload(
    rm_can_bridge::GimbalCommandInput{
    false, false, false, true, true, true, 960, -640});

  EXPECT_EQ(payload[4], 0x00);
  EXPECT_EQ(payload[5], 0x00);
}
