// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/detector_runtime_config.hpp"
#include "armor_detector/latest_frame_slot.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/latency_window.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

namespace rm_auto_aim
{

class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void workerLoop();
  void processLatestFrame(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
    const DetectorRuntimeConfig & runtime_config);
  bool shouldPublishDebugImages(double debug_image_fps);

  DetectorRuntimeConfig getRuntimeConfig();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void logStatsIfNeeded(double latency_ms, const DetectorRuntimeConfig & runtime_config);

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();

  // Armor Detector
  std::unique_ptr<Detector> detector_;
  DetectorRuntimeConfig runtime_config_;
  std::mutex config_mutex_;

  // Latest frame worker
  LatestFrameSlot<sensor_msgs::msg::Image::ConstSharedPtr> latest_frame_slot_;
  std::thread worker_thread_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::mutex camera_info_mutex_;
  cv::Point2f cam_center_{0.0F, 0.0F};
  std::shared_ptr<PnPSolver> pnp_solver_;

  // Image subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Runtime parameter callback
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  // Debug information
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;
  std::chrono::steady_clock::time_point last_debug_image_pub_time_;
  bool has_debug_image_pub_time_{false};

  std::chrono::milliseconds latency_log_period_{1000};
  auto_aim_interfaces::LatencyWindow latency_window_{std::chrono::milliseconds(1000)};

  // Stats
  std::atomic<std::uint64_t> input_frame_count_{0};
  std::atomic<std::uint64_t> processed_frame_count_{0};
  std::chrono::steady_clock::time_point last_stats_log_time_;
  bool has_stats_log_time_{false};
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
