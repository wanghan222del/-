#include <gtest/gtest.h>

#include "rm_can_bridge/target_aim_math.hpp"
#include "rm_can_bridge/q10_6_angle.hpp"

TEST(TargetAimMath, ComputesZeroAnglesWhenTargetIsStraightAhead)
{
  const auto angles = rm_can_bridge::computeRelativeAim(
    rm_can_bridge::Vec3{1.0, 0.0, 0.0}, rm_can_bridge::Vec3{0.0, 0.0, 0.0});
  EXPECT_NEAR(angles.yaw_rad, 0.0, 1e-6);
  EXPECT_NEAR(angles.pitch_rad, 0.0, 1e-6);
}

TEST(TargetAimMath, ComputesPositiveYawAndPitchInGimbalFrame)
{
  const auto angles = rm_can_bridge::computeRelativeAim(
    rm_can_bridge::Vec3{1.0, 0.26794919, 0.27740142}, rm_can_bridge::Vec3{0.0, 0.0, 0.0});
  EXPECT_NEAR(angles.yaw_rad, 0.261799, 1e-4);
  EXPECT_NEAR(angles.pitch_rad, 0.261799, 1e-4);
}

TEST(TargetAimMath, AppliesMuzzleOffsetBeforeAngleCalculation)
{
  const auto angles = rm_can_bridge::computeRelativeAim(
    rm_can_bridge::Vec3{1.0, 0.0, 0.10}, rm_can_bridge::Vec3{0.0, 0.0, 0.10});
  EXPECT_NEAR(angles.yaw_rad, 0.0, 1e-6);
  EXPECT_NEAR(angles.pitch_rad, 0.0, 1e-6);
}

TEST(TargetAimMath, Encodes15DegToQ10_6)
{
  const auto angles = rm_can_bridge::computeRelativeAim(
    rm_can_bridge::Vec3{1.0, 0.26794919, 0.27740142}, rm_can_bridge::Vec3{0.0, 0.0, 0.0});
  EXPECT_EQ(rm_can_bridge::degToQ10_6Raw(angles.yaw_rad * 180.0 / M_PI), 960);
  EXPECT_EQ(rm_can_bridge::degToQ10_6Raw(angles.pitch_rad * 180.0 / M_PI), 960);
}
