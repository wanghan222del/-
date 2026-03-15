// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

// STD
#include <algorithm>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  detector_ = initDetector();

  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  runtime_config_.debug = this->declare_parameter("debug", false);
  runtime_config_.debug_image_fps = this->declare_parameter("debug_image_fps", 20.0);
  runtime_config_.debug_publish_result_img =
    this->declare_parameter("debug_publish_result_img", true);
  runtime_config_.debug_publish_binary_img =
    this->declare_parameter("debug_publish_binary_img", false);
  runtime_config_.debug_publish_number_img =
    this->declare_parameter("debug_publish_number_img", false);
  runtime_config_.stats_log_period_ms = this->declare_parameter("stats_log_period_ms", 1000);
  this->declare_parameter("latest_frame_policy", std::string("keep_latest"));

  runtime_config_.binary_thres = this->get_parameter("binary_thres").as_int();
  runtime_config_.detect_color = this->get_parameter("detect_color").as_int();
  runtime_config_.classifier_threshold = this->get_parameter("classifier_threshold").as_double();

  createDebugPublishers();

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArmorDetectorNode::parametersCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      pnp_solver_ = std::make_shared<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  worker_thread_ = std::thread(&ArmorDetectorNode::workerLoop, this);
}

ArmorDetectorNode::~ArmorDetectorNode()
{
  latest_frame_slot_.stop();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  destroyDebugPublishers();
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  input_frame_count_.fetch_add(1, std::memory_order_relaxed);
  latest_frame_slot_.push(img_msg);
}

void ArmorDetectorNode::workerLoop()
{
  sensor_msgs::msg::Image::ConstSharedPtr img_msg;
  while (latest_frame_slot_.wait_and_take(img_msg)) {
    if (!img_msg) {
      continue;
    }

    try {
      processLatestFrame(img_msg);
      processed_frame_count_.fetch_add(1, std::memory_order_relaxed);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Failed to convert image in detector worker: %s", ex.what());
    } catch (const cv::Exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "OpenCV exception in detector worker: %s",
        ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Unhandled detector worker exception: %s",
        ex.what());
    }
  }
}

void ArmorDetectorNode::processLatestFrame(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  const auto runtime_config = getRuntimeConfig();
  auto armors = detectArmors(img_msg, runtime_config);

  std::shared_ptr<PnPSolver> pnp_solver;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    pnp_solver = pnp_solver_;
  }

  if (pnp_solver != nullptr) {
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      cv::Mat rvec;
      cv::Mat tvec;
      const bool success = pnp_solver->solvePnP(armor, rvec, tvec);
      if (!success) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "PnP failed in detector worker");
        continue;
      }

      armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
      armor_msg.number = armor.number;
      armor_msg.pose.position.x = tvec.at<double>(0);
      armor_msg.pose.position.y = tvec.at<double>(1);
      armor_msg.pose.position.z = tvec.at<double>(2);

      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);
      tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));
      tf2::Quaternion tf2_q;
      tf2_rotation_matrix.getRotation(tf2_q);
      armor_msg.pose.orientation.x = tf2_q.x();
      armor_msg.pose.orientation.y = tf2_q.y();
      armor_msg.pose.orientation.z = tf2_q.z();
      armor_msg.pose.orientation.w = tf2_q.w();
      armor_msg.distance_to_image_center = pnp_solver->calculateDistanceToCenter(armor.center);

      armor_marker_.id++;
      armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
      armor_marker_.pose = armor_msg.pose;
      text_marker_.id++;
      text_marker_.pose.position = armor_msg.pose.position;
      text_marker_.pose.position.y -= 0.1;
      text_marker_.text = armor.classfication_result;
      armors_msg_.armors.emplace_back(armor_msg);
      marker_array_.markers.emplace_back(armor_marker_);
      marker_array_.markers.emplace_back(text_marker_);
    }

    armors_pub_->publish(armors_msg_);
    publishMarkers();
  }

  const double latency_ms = (this->now() - img_msg->header.stamp).seconds() * 1000.0;
  logStatsIfNeeded(latency_ms, runtime_config);
}

rcl_interfaces::msg::SetParametersResult ArmorDetectorNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto next_config = getRuntimeConfig();

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "latest_frame_policy") {
      if (parameter.as_string() != "keep_latest") {
        result.successful = false;
        result.reason = "latest_frame_policy only supports keep_latest";
        return result;
      }
      continue;
    }

    if (name == "debug_image_fps" && parameter.as_double() < 0.0) {
      result.successful = false;
      result.reason = "debug_image_fps must be >= 0";
      return result;
    }

    if (name == "stats_log_period_ms" && parameter.as_int() < 0) {
      result.successful = false;
      result.reason = "stats_log_period_ms must be >= 0";
      return result;
    }
  }

  next_config.apply(parameters);

  std::lock_guard<std::mutex> lock(config_mutex_);
  runtime_config_ = next_config;
  return result;
}

DetectorRuntimeConfig ArmorDetectorNode::getRuntimeConfig()
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return runtime_config_;
}

void ArmorDetectorNode::logStatsIfNeeded(
  double latency_ms, const DetectorRuntimeConfig & runtime_config)
{
  if (runtime_config.stats_log_period_ms <= 0) {
    return;
  }

  const auto period = std::chrono::milliseconds(runtime_config.stats_log_period_ms);
  if (period != latency_log_period_) {
    latency_log_period_ = period;
    latency_window_ = auto_aim_interfaces::LatencyWindow(latency_log_period_);
  }

  const auto steady_now = std::chrono::steady_clock::now();
  latency_window_.observe(latency_ms);

  auto latency_snapshot = auto_aim_interfaces::LatencyWindow::Snapshot{};
  if (latency_window_.takeSnapshotIfDue(steady_now, latency_snapshot)) {
    RCLCPP_INFO(
      this->get_logger(), "detector latency: samples=%zu avg=%.2fms max=%.2fms last=%.2fms",
      latency_snapshot.sample_count, latency_snapshot.avg_ms, latency_snapshot.max_ms,
      latency_snapshot.last_ms);
  }

  if (!has_stats_log_time_) {
    last_stats_log_time_ = steady_now;
    has_stats_log_time_ = true;
    return;
  }

  if (steady_now - last_stats_log_time_ < period) {
    return;
  }

  last_stats_log_time_ = steady_now;
  RCLCPP_INFO(
    this->get_logger(), "detector stats: input=%zu processed=%zu overwritten=%zu latency=%.2fms",
    static_cast<std::size_t>(input_frame_count_.load(std::memory_order_relaxed)),
    static_cast<std::size_t>(processed_frame_count_.load(std::memory_order_relaxed)),
    latest_frame_slot_.overwritten_count(), latency_ms);
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
  const DetectorRuntimeConfig & runtime_config)
{
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  detector_->binary_thres = runtime_config.binary_thres;
  detector_->detect_color = runtime_config.detect_color;
  detector_->classifier->threshold = runtime_config.classifier_threshold;

  auto armors = detector_->detect(img);

  const auto final_time = this->now();
  const auto latency = (final_time - img_msg->header.stamp).seconds() * 1000.0;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  if (runtime_config.debug) {
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & light_1, const auto & light_2) {
        return light_1.center_x < light_2.center_x;
      });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & armor_1, const auto & armor_2) {
        return armor_1.center_x < armor_2.center_x;
      });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (shouldPublishDebugImages(runtime_config.debug_image_fps)) {
      if (runtime_config.debug_publish_binary_img) {
        binary_img_pub_.publish(
          cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());
      }

      if (runtime_config.debug_publish_number_img && !armors.empty()) {
        auto all_num_img = detector_->getAllNumbersImage();
        number_img_pub_.publish(
          *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
      }

      if (runtime_config.debug_publish_result_img) {
        detector_->drawResults(img);
        cv::Point2f cam_center;
        {
          std::lock_guard<std::mutex> lock(camera_info_mutex_);
          cam_center = cam_center_;
        }
        cv::circle(img, cam_center, 5, cv::Scalar(255, 0, 0), 2);

        std::stringstream latency_stream;
        latency_stream << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
        cv::putText(
          img, latency_stream.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
          cv::Scalar(0, 255, 0), 2);
        result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
      }
    }
  }

  return armors;
}

bool ArmorDetectorNode::shouldPublishDebugImages(double debug_image_fps)
{
  if (debug_image_fps <= 0.0) {
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!has_debug_image_pub_time_) {
    last_debug_image_pub_time_ = now;
    has_debug_image_pub_time_ = true;
    return true;
  }

  const auto min_interval = std::chrono::duration<double>(1.0 / debug_image_fps);
  if (now - last_debug_image_pub_time_ < min_interval) {
    return false;
  }

  last_debug_image_pub_time_ = now;
  return true;
}

void ArmorDetectorNode::createDebugPublishers()
{
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
