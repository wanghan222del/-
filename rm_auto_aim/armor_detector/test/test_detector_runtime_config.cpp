#include <gtest/gtest.h>

#include <rclcpp/parameter.hpp>
#include <vector>

#include "armor_detector/detector_runtime_config.hpp"

TEST(DetectorRuntimeConfigTest, AppliesParameterUpdates)
{
  rm_auto_aim::DetectorRuntimeConfig config;
  config.binary_thres = 80;
  config.detect_color = 0;
  config.classifier_threshold = 0.8;

  config.apply(std::vector<rclcpp::Parameter>{
    rclcpp::Parameter("binary_thres", 96), rclcpp::Parameter("detect_color", 1),
    rclcpp::Parameter("classifier_threshold", 0.65)});

  EXPECT_EQ(config.binary_thres, 96);
  EXPECT_EQ(config.detect_color, 1);
  EXPECT_DOUBLE_EQ(config.classifier_threshold, 0.65);
}
