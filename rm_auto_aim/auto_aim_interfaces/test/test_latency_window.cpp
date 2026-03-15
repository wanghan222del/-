#include <gtest/gtest.h>

#include <chrono>

#include "auto_aim_interfaces/latency_window.hpp"

namespace
{
using auto_aim_interfaces::LatencyWindow;
using Clock = std::chrono::steady_clock;

TEST(LatencyWindowTest, AggregatesSamplesWhenPeriodElapsed)
{
  LatencyWindow window(std::chrono::milliseconds(1000));
  const auto start = Clock::now();

  window.observe(5.0);
  window.observe(15.0);
  window.observe(10.0);

  LatencyWindow::Snapshot snapshot{};
  EXPECT_FALSE(window.takeSnapshotIfDue(start + std::chrono::milliseconds(900), snapshot));
  EXPECT_TRUE(window.takeSnapshotIfDue(start + std::chrono::milliseconds(1000), snapshot));
  EXPECT_EQ(snapshot.sample_count, 3u);
  EXPECT_DOUBLE_EQ(snapshot.avg_ms, 10.0);
  EXPECT_DOUBLE_EQ(snapshot.max_ms, 15.0);
  EXPECT_DOUBLE_EQ(snapshot.last_ms, 10.0);
}

TEST(LatencyWindowTest, ResetsAfterSnapshot)
{
  LatencyWindow window(std::chrono::milliseconds(1000));
  const auto start = Clock::now();

  window.observe(8.0);
  LatencyWindow::Snapshot snapshot{};
  ASSERT_TRUE(window.takeSnapshotIfDue(start + std::chrono::milliseconds(1000), snapshot));
  EXPECT_EQ(snapshot.sample_count, 1u);
  EXPECT_DOUBLE_EQ(snapshot.avg_ms, 8.0);

  window.observe(20.0);
  EXPECT_FALSE(window.takeSnapshotIfDue(start + std::chrono::milliseconds(1500), snapshot));
  ASSERT_TRUE(window.takeSnapshotIfDue(start + std::chrono::milliseconds(2000), snapshot));
  EXPECT_EQ(snapshot.sample_count, 1u);
  EXPECT_DOUBLE_EQ(snapshot.avg_ms, 20.0);
  EXPECT_DOUBLE_EQ(snapshot.max_ms, 20.0);
  EXPECT_DOUBLE_EQ(snapshot.last_ms, 20.0);
}
}  // namespace
