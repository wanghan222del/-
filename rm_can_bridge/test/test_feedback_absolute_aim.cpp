#include <gtest/gtest.h>

#include "rm_can_bridge/feedback_absolute_aim.hpp"

TEST(FeedbackAbsoluteAim, AddsFeedbackAndRelativeOffsets)
{
  const auto command = rm_can_bridge::composeAbsoluteAimFromFeedback(
    rm_can_bridge::FeedbackAimDeg{10.0, -3.0},
    rm_can_bridge::RelativeAimDeg{5.0, 2.0});

  EXPECT_NEAR(command.yaw_deg, 15.0, 1e-6);
  EXPECT_NEAR(command.pitch_deg, -1.0, 1e-6);
}

TEST(FeedbackAbsoluteAim, WrapsYawAcrossPositiveBoundary)
{
  const auto command = rm_can_bridge::composeAbsoluteAimFromFeedback(
    rm_can_bridge::FeedbackAimDeg{178.0, 0.0},
    rm_can_bridge::RelativeAimDeg{5.0, 0.0});

  EXPECT_NEAR(command.yaw_deg, -177.0, 1e-6);
  EXPECT_NEAR(command.pitch_deg, 0.0, 1e-6);
}

TEST(FeedbackAbsoluteAim, WrapsYawAcrossNegativeBoundary)
{
  const auto command = rm_can_bridge::composeAbsoluteAimFromFeedback(
    rm_can_bridge::FeedbackAimDeg{-179.0, 1.0},
    rm_can_bridge::RelativeAimDeg{-4.0, -2.5});

  EXPECT_NEAR(command.yaw_deg, 177.0, 1e-6);
  EXPECT_NEAR(command.pitch_deg, -1.5, 1e-6);
}
