#include <gtest/gtest.h>

#include "rm_can_bridge/aim_axis_sign.hpp"

TEST(AimAxisSign, KeepsAnglesWhenNotInverted)
{
  const auto output = rm_can_bridge::applyAimAxisSign(
    rm_can_bridge::AimAxisSignInputDeg{12.5, -3.0},
    rm_can_bridge::AimAxisSignConfig{false, false});
  EXPECT_NEAR(output.yaw_deg, 12.5, 1e-6);
  EXPECT_NEAR(output.pitch_deg, -3.0, 1e-6);
}

TEST(AimAxisSign, InvertsConfiguredAxes)
{
  const auto output = rm_can_bridge::applyAimAxisSign(
    rm_can_bridge::AimAxisSignInputDeg{12.5, -3.0},
    rm_can_bridge::AimAxisSignConfig{true, true});
  EXPECT_NEAR(output.yaw_deg, -12.5, 1e-6);
  EXPECT_NEAR(output.pitch_deg, 3.0, 1e-6);
}

TEST(AimAxisSign, CanInvertYawOnly)
{
  const auto output = rm_can_bridge::applyAimAxisSign(
    rm_can_bridge::AimAxisSignInputDeg{-8.0, 6.0},
    rm_can_bridge::AimAxisSignConfig{true, false});
  EXPECT_NEAR(output.yaw_deg, 8.0, 1e-6);
  EXPECT_NEAR(output.pitch_deg, 6.0, 1e-6);
}
