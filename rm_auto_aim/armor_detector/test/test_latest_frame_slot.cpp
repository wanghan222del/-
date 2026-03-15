#include <gtest/gtest.h>

#include "armor_detector/latest_frame_slot.hpp"

TEST(LatestFrameSlotTest, NewFrameOverwritesOlderPendingFrame)
{
  rm_auto_aim::LatestFrameSlot<int> slot;
  slot.push(1);
  slot.push(2);

  int value = 0;
  ASSERT_TRUE(slot.try_take(value));
  EXPECT_EQ(value, 2);
  EXPECT_EQ(slot.overwritten_count(), 1u);
}

TEST(LatestFrameSlotTest, StopUnblocksWaitingConsumer)
{
  rm_auto_aim::LatestFrameSlot<int> slot;
  slot.stop();

  int value = 0;
  EXPECT_FALSE(slot.wait_and_take(value));
}
