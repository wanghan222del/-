#include <gtest/gtest.h>

#include <rclcpp/time.hpp>

#include "rm_can_bridge/pending_aim_latency.hpp"

namespace
{
TEST(PendingAimLatencyTest, ReportsLatencyOnlyForFirstSend)
{
  rm_can_bridge::PendingAimLatency pending;
  pending.markNewCommand(rclcpp::Time(1, 0, RCL_ROS_TIME));

  double latency_ms = 0.0;
  EXPECT_TRUE(pending.takePendingLatency(rclcpp::Time(1, 50'000'000, RCL_ROS_TIME), latency_ms));
  EXPECT_DOUBLE_EQ(latency_ms, 50.0);
  EXPECT_FALSE(pending.takePendingLatency(rclcpp::Time(1, 70'000'000, RCL_ROS_TIME), latency_ms));
}

TEST(PendingAimLatencyTest, RearmsAfterNewCommand)
{
  rm_can_bridge::PendingAimLatency pending;
  pending.markNewCommand(rclcpp::Time(2, 0, RCL_ROS_TIME));

  double latency_ms = 0.0;
  ASSERT_TRUE(pending.takePendingLatency(rclcpp::Time(2, 20'000'000, RCL_ROS_TIME), latency_ms));
  EXPECT_DOUBLE_EQ(latency_ms, 20.0);

  pending.markNewCommand(rclcpp::Time(3, 0, RCL_ROS_TIME));
  ASSERT_TRUE(pending.takePendingLatency(rclcpp::Time(3, 30'000'000, RCL_ROS_TIME), latency_ms));
  EXPECT_DOUBLE_EQ(latency_ms, 30.0);
}
}  // namespace
