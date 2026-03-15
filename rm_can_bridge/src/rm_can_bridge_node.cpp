#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "auto_aim_interfaces/latency_window.hpp"
#include "rm_can_bridge/aim_axis_sign.hpp"
#include "rm_can_bridge/feedback_absolute_aim.hpp"
#include "rm_can_bridge/gimbal_command_frame.hpp"
#include "rm_can_bridge/pending_aim_latency.hpp"
#include "rm_can_bridge/q10_6_angle.hpp"
#include "rm_can_bridge/target_aim_math.hpp"
#include "rm_can_bridge/usb_can_frame_parser.hpp"

namespace
{
constexpr uint32_t CAN_ID_NUC_ESTOP = 0x00A;
constexpr uint32_t CAN_ID_NUC_GIMBAL_CMD = 0x301;
constexpr uint32_t CAN_ID_NUC_CHASSIS_CMD = 0x311;
constexpr size_t USB_CAN_TX_FRAME_SIZE = 30;
constexpr size_t DEFAULT_DEBUG_HEX_BYTES = 32;
}  // namespace

class RmCanBridgeNode : public rclcpp::Node
{
public:
  RmCanBridgeNode()
  : Node("rm_can_bridge")
  {
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    serial_baud_ = this->declare_parameter<int>("serial_baud", 921600);
    reconnect_interval_ms_ = this->declare_parameter<int>("reconnect_interval_ms", 1000);
    debug_serial_rx_ = this->declare_parameter<bool>("debug_serial_rx", false);
    debug_usb_can_frames_ = this->declare_parameter<bool>("debug_usb_can_frames", false);
    debug_hex_bytes_ =
      this->declare_parameter<int>("debug_hex_bytes", static_cast<int>(DEFAULT_DEBUG_HEX_BYTES));
    if (debug_hex_bytes_ < 1) {
      RCLCPP_WARN(get_logger(), "debug_hex_bytes=%d is too small, clamp to 1", debug_hex_bytes_);
      debug_hex_bytes_ = 1;
    }
    if (reconnect_interval_ms_ < 100) {
      RCLCPP_WARN(
        get_logger(), "reconnect_interval_ms=%d is too small, clamp to 100ms",
        reconnect_interval_ms_);
      reconnect_interval_ms_ = 100;
    }

    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    armors_topic_ = this->declare_parameter<std::string>("armors_topic", "/detector/armors");
    target_topic_ = this->declare_parameter<std::string>("target_topic", "/tracker/target");
    e_stop_topic_ = this->declare_parameter<std::string>("e_stop_topic", "/can/e_stop");
    fire_enable_topic_ =
      this->declare_parameter<std::string>("fire_enable_topic", "/can/fire_enable");
    avoid_active_topic_ =
      this->declare_parameter<std::string>("avoid_active_topic", "/can/avoid_active");

    input_mode_ = this->declare_parameter<std::string>("input_mode", "detector_armors");
    aim_reference_mode_ = this->declare_parameter<std::string>("aim_reference_mode", "relative");
    auto_fire_ = this->declare_parameter<bool>("auto_fire", false);
    zero_when_lost_ = this->declare_parameter<bool>("zero_when_lost", true);
    publish_feedback_ = this->declare_parameter<bool>("publish_feedback", true);
    angle_unit_ = this->declare_parameter<std::string>("angle_unit", "deg");
    invert_yaw_axis_ = this->declare_parameter<bool>("invert_yaw_axis", false);
    invert_pitch_axis_ = this->declare_parameter<bool>("invert_pitch_axis", false);
    expected_target_frame_ =
      this->declare_parameter<std::string>("expected_target_frame", "gimbal_link");
    feedback_frame_id_ =
      this->declare_parameter<std::string>("feedback_frame_id", "gimbal_link");
    linear_scale_ = this->declare_parameter<double>("linear_scale", 1000.0);
    angular_scale_ = this->declare_parameter<double>("angular_scale", 1000.0);
    nav_timeout_ms_ = this->declare_parameter<int>("nav_timeout_ms", 300);
    aim_timeout_ms_ = this->declare_parameter<int>("aim_timeout_ms", 300);
    target_timeout_ms_ = this->declare_parameter<int>("target_timeout_ms", 300);
    feedback_timeout_ms_ = this->declare_parameter<int>("feedback_timeout_ms", 200);
    feedback_can_id_ =
      static_cast<uint32_t>(this->declare_parameter<int>("feedback_can_id", 0x402));

    const auto muzzle_offset = this->declare_parameter<std::vector<double>>(
      "muzzle_offset_xyz_in_gimbal", std::vector<double>{0.0, 0.0, 0.0});
    if (muzzle_offset.size() == 3) {
      muzzle_offset_in_gimbal_ =
        rm_can_bridge::Vec3{muzzle_offset[0], muzzle_offset[1], muzzle_offset[2]};
    } else {
      RCLCPP_WARN(
        get_logger(),
        "muzzle_offset_xyz_in_gimbal expects 3 elements, got %zu. Fallback to zeros.",
        muzzle_offset.size());
      muzzle_offset_in_gimbal_ = rm_can_bridge::Vec3{0.0, 0.0, 0.0};
    }

    if (input_mode_ != "detector_armors" && input_mode_ != "tracker_target") {
      RCLCPP_WARN(
        get_logger(), "Unsupported input_mode='%s', fallback to detector_armors",
        input_mode_.c_str());
      input_mode_ = "detector_armors";
    }
    if (angle_unit_ != "deg" && angle_unit_ != "rad") {
      RCLCPP_WARN(
        get_logger(), "Unsupported angle_unit='%s', fallback to deg",
        angle_unit_.c_str());
      angle_unit_ = "deg";
    }
    if (aim_reference_mode_ != "relative" && aim_reference_mode_ != "absolute_from_feedback") {
      RCLCPP_WARN(
        get_logger(), "Unsupported aim_reference_mode='%s', fallback to relative",
        aim_reference_mode_.c_str());
      aim_reference_mode_ = "relative";
    }
    if (feedback_timeout_ms_ < 10) {
      RCLCPP_WARN(
        get_logger(), "feedback_timeout_ms=%d is too small, clamp to 10ms",
        feedback_timeout_ms_);
      feedback_timeout_ms_ = 10;
    }

    clearAim();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&RmCanBridgeNode::cmdVelCallback, this, std::placeholders::_1));

    if (input_mode_ == "tracker_target") {
      target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        target_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RmCanBridgeNode::targetCallback, this, std::placeholders::_1));
    } else {
      armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        armors_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RmCanBridgeNode::armorsCallback, this, std::placeholders::_1));
    }

    e_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      e_stop_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&RmCanBridgeNode::eStopCallback, this, std::placeholders::_1));

    fire_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      fire_enable_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&RmCanBridgeNode::fireEnableCallback, this, std::placeholders::_1));

    avoid_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      avoid_active_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&RmCanBridgeNode::avoidActiveCallback, this, std::placeholders::_1));

    if (publish_feedback_) {
      feedback_rpy_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/gimbal/feedback/rpy_deg", rclcpp::SystemDefaultsQoS());
      feedback_yaw_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/gimbal/feedback/yaw_motor_pos_deg", rclcpp::SystemDefaultsQoS());
    }

    estop_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RmCanBridgeNode::sendEstopFrame, this));

    gimbal_cmd_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&RmCanBridgeNode::sendGimbalFrame, this));

    chassis_cmd_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RmCanBridgeNode::sendChassisFrame, this));

    serial_rx_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&RmCanBridgeNode::pollSerial, this));

    if (!openSerialPort()) {
      scheduleReconnect();
    }
    RCLCPP_INFO(
      get_logger(),
      "rm_can_bridge started. mode=%s aim_reference_mode=%s angle_unit=%s "
      "invert_yaw_axis=%s invert_pitch_axis=%s IDs: 0x%03X (e-stop), 0x%03X "
      "(gimbal cmd), 0x%03X (chassis cmd), 0x%03X (feedback)",
      input_mode_.c_str(), aim_reference_mode_.c_str(), angle_unit_.c_str(),
      invert_yaw_axis_ ? "true" : "false", invert_pitch_axis_ ? "true" : "false",
      CAN_ID_NUC_ESTOP, CAN_ID_NUC_GIMBAL_CMD, CAN_ID_NUC_CHASSIS_CMD, feedback_can_id_);
  }

  ~RmCanBridgeNode() override
  {
    closeSerialPort();
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_vel_ = *msg;
    latest_cmd_vel_stamp_ = this->now();
  }

  void eStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    e_stop_ = msg->data;
  }

  void fireEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    fire_enable_ = msg->data;
  }

  void avoidActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    avoid_active_ = msg->data;
  }

  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
  {
    if (msg->armors.empty()) {
      if (zero_when_lost_) {
        clearAim();
      }
      return;
    }

    const auto * best = &msg->armors.front();
    for (const auto & armor : msg->armors) {
      if (armor.distance_to_image_center < best->distance_to_image_center) {
        best = &armor;
      }
    }

    const double x = best->pose.position.x;
    const double y = best->pose.position.y;
    const double z = best->pose.position.z;

    if (std::abs(z) < 1e-6) {
      if (zero_when_lost_) {
        clearAim();
      }
      return;
    }

    const double yaw_rad = std::atan2(x, z);
    const double pitch_rad = std::atan2(-y, std::hypot(x, z));
    if (setAimFromRelativeRadians(yaw_rad, pitch_rad)) {
      pending_aim_latency_.markNewCommand(msg->header.stamp);
    }
  }

  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
  {
    if (!msg->tracking) {
      if (zero_when_lost_) {
        clearAim();
      }
      return;
    }

    if (
      !expected_target_frame_.empty() && !msg->header.frame_id.empty() &&
      msg->header.frame_id != expected_target_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "tracker target frame is '%s', expected '%s'. Still using its position directly.",
        msg->header.frame_id.c_str(), expected_target_frame_.c_str());
    }

    try {
      const auto angles = rm_can_bridge::computeRelativeAim(
        rm_can_bridge::Vec3{msg->position.x, msg->position.y, msg->position.z},
        muzzle_offset_in_gimbal_);
      if (setAimFromRelativeRadians(angles.yaw_rad, angles.pitch_rad)) {
        pending_aim_latency_.markNewCommand(msg->header.stamp);
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to compute tracker target aim: %s", e.what());
      if (zero_when_lost_) {
        clearAim();
      }
    }
  }

  bool setAimFromRelativeRadians(double yaw_rad, double pitch_rad)
  {
    auto signed_relative = rm_can_bridge::applyAimAxisSign(
      rm_can_bridge::AimAxisSignInputDeg{yaw_rad * 180.0 / M_PI, pitch_rad * 180.0 / M_PI},
      rm_can_bridge::AimAxisSignConfig{invert_yaw_axis_, invert_pitch_axis_});
    double yaw_deg = signed_relative.yaw_deg;
    double pitch_deg = signed_relative.pitch_deg;

    if (aim_reference_mode_ == "absolute_from_feedback") {
      const bool feedback_fresh = feedback_valid_ && isFresh(
        latest_feedback_stamp_,
        feedback_timeout_ms_);
      if (!feedback_fresh) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Aim reference mode requires fresh 0x%03X feedback before sending gimbal targets",
          feedback_can_id_);
        if (zero_when_lost_) {
          clearAim();
        }
        return false;
      }

      const auto absolute_command = rm_can_bridge::composeAbsoluteAimFromFeedback(
        rm_can_bridge::FeedbackAimDeg{latest_feedback_yaw_deg_, latest_feedback_pitch_deg_},
        rm_can_bridge::RelativeAimDeg{yaw_deg, pitch_deg});
      yaw_deg = absolute_command.yaw_deg;
      pitch_deg = absolute_command.pitch_deg;
    }

    latest_yaw_q10_6_ = encodeAngleDegreesToQ10_6(yaw_deg);
    latest_pitch_q10_6_ = encodeAngleDegreesToQ10_6(pitch_deg);
    has_aim_command_ = true;
    latest_aim_stamp_ = this->now();
    return true;
  }

  void clearAim()
  {
    latest_yaw_q10_6_ = 0;
    latest_pitch_q10_6_ = 0;
    has_aim_command_ = false;
    latest_aim_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    pending_aim_latency_.clear();
  }

  int currentAimTimeoutMs() const
  {
    if (input_mode_ == "tracker_target") {
      return target_timeout_ms_;
    }
    return aim_timeout_ms_;
  }

  int16_t encodeAngleDegreesToQ10_6(double angle_deg) const
  {
    if (angle_unit_ == "deg") {
      return rm_can_bridge::degToQ10_6Raw(angle_deg);
    }
    return rm_can_bridge::radToQ10_6Raw(angle_deg * M_PI / 180.0);
  }

  void sendEstopFrame()
  {
    std::array<uint8_t, 8> data{};
    if (e_stop_) {
      data.fill(0xFF);
    }
    sendCanFrame(CAN_ID_NUC_ESTOP, data);
  }

  void sendGimbalFrame()
  {
    const bool aim_fresh = isFresh(latest_aim_stamp_, currentAimTimeoutMs());
    const auto data = rm_can_bridge::buildGimbalCommandPayload(
      rm_can_bridge::GimbalCommandInput{
      e_stop_,
      aim_fresh,
      fire_enable_,
      auto_fire_,
      !zero_when_lost_,
      has_aim_command_,
      latest_yaw_q10_6_,
      latest_pitch_q10_6_,
    });

    const auto send_time = this->now();
    sendCanFrame(CAN_ID_NUC_GIMBAL_CMD, data);

    if (aim_fresh && has_aim_command_) {
      double latency_ms = 0.0;
      if (pending_aim_latency_.takePendingLatency(send_time, latency_ms)) {
        can_send_latency_window_.observe(latency_ms);
        auto latency_snapshot = auto_aim_interfaces::LatencyWindow::Snapshot{};
        if (can_send_latency_window_.takeSnapshotIfDue(
            std::chrono::steady_clock::now(), latency_snapshot))
        {
          RCLCPP_INFO(
            get_logger(),
            "can_send latency: samples=%zu avg=%.2fms max=%.2fms last=%.2fms",
            latency_snapshot.sample_count, latency_snapshot.avg_ms, latency_snapshot.max_ms,
            latency_snapshot.last_ms);
        }
      }
    }
  }

  void sendChassisFrame()
  {
    std::array<uint8_t, 8> data{};
    const bool nav_fresh = isFresh(latest_cmd_vel_stamp_, nav_timeout_ms_);
    const bool aim_fresh = isFresh(latest_aim_stamp_, currentAimTimeoutMs());
    const bool fire = !e_stop_ && (fire_enable_ || (auto_fire_ && aim_fresh));

    if (!e_stop_ && nav_fresh) {
      const int16_t vx = toScaledInt16(latest_cmd_vel_.linear.x, linear_scale_);
      const int16_t vy = toScaledInt16(latest_cmd_vel_.linear.y, linear_scale_);
      const int16_t wz = toScaledInt16(latest_cmd_vel_.angular.z, angular_scale_);
      rm_can_bridge::writeInt16BE(data, 0, vx);
      rm_can_bridge::writeInt16BE(data, 2, vy);
      rm_can_bridge::writeInt16BE(data, 4, wz);
    }

    uint8_t status = 0;
    if (nav_fresh) {
      status |= (1U << 0);
    }
    if (avoid_active_) {
      status |= (1U << 1);
    }
    if (aim_fresh) {
      status |= (1U << 2);
    }
    if (fire) {
      status |= (1U << 3);
    }
    data[6] = status;

    sendCanFrame(CAN_ID_NUC_CHASSIS_CMD, data);
  }

  static std::string formatHex(const uint8_t * data, size_t size, size_t max_bytes)
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::uppercase;
    const size_t limit = std::min(size, max_bytes);
    for (size_t i = 0; i < limit; ++i) {
      if (i != 0) {
        stream << ' ';
      }
      stream << std::setw(2) << static_cast<int>(data[i]);
    }
    if (size > limit) {
      stream << " ...";
    }
    return stream.str();
  }

  void pollSerial()
  {
    if (!ensureSerialReady()) {
      return;
    }

    std::array<uint8_t, 256> buffer{};
    while (true) {
      const ssize_t received = read(serial_fd_, buffer.data(), buffer.size());
      if (received > 0) {
        if (debug_serial_rx_) {
          RCLCPP_INFO(
            get_logger(), "USB-CAN raw read: %zd bytes [%s]", received,
            formatHex(
              buffer.data(), static_cast<size_t>(received),
              static_cast<size_t>(debug_hex_bytes_)).c_str());
        }
        serial_rx_buffer_.insert(
          serial_rx_buffer_.end(), buffer.begin(), buffer.begin() + received);
        continue;
      }
      if (received == 0) {
        break;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      const int saved_errno = errno;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "USB-CAN read failed (errno=%d), reconnecting",
        saved_errno);
      closeSerialPort();
      scheduleReconnect();
      return;
    }

    consumeSerialFrames();
  }

  void consumeSerialFrames()
  {
    while (!serial_rx_buffer_.empty()) {
      const auto result = rm_can_bridge::tryParseUsbCanFrame(serial_rx_buffer_);
      if (result.action == rm_can_bridge::UsbCanParseAction::NeedMoreData) {
        break;
      }

      if (result.action == rm_can_bridge::UsbCanParseAction::SkipByte) {
        if (debug_usb_can_frames_) {
          RCLCPP_WARN(
            get_logger(),
            "USB-CAN frame sync mismatch, drop 1 byte from buffer(size=%zu) [%s]",
            serial_rx_buffer_.size(),
            formatHex(
              serial_rx_buffer_.data(), serial_rx_buffer_.size(),
              static_cast<size_t>(debug_hex_bytes_)).c_str());
        }
        serial_rx_buffer_.erase(serial_rx_buffer_.begin());
        continue;
      }

      serial_rx_buffer_.erase(
        serial_rx_buffer_.begin(),
        serial_rx_buffer_.begin() + static_cast<std::ptrdiff_t>(result.frame.consumed_bytes));
      handleUsbCanFrame(result.frame);
    }
  }

  void handleUsbCanFrame(const rm_can_bridge::ParsedUsbCanFrame & frame)
  {
    if (frame.can_id != feedback_can_id_) {
      return;
    }

    if (debug_usb_can_frames_) {
      RCLCPP_INFO(
        get_logger(), "USB-CAN frame id=0x%03X payload=[%s]", frame.can_id,
        formatHex(frame.payload.data(), frame.payload.size(), frame.payload.size()).c_str());
    }
    publishFeedback(frame.payload);
  }

  void publishFeedback(const std::array<uint8_t, 8> & payload)
  {
    const double yaw_deg = rm_can_bridge::q10_6RawToDeg(rm_can_bridge::readInt16BE(payload, 0));
    const double pitch_deg = rm_can_bridge::q10_6RawToDeg(rm_can_bridge::readInt16BE(payload, 2));
    const double roll_deg = rm_can_bridge::q10_6RawToDeg(rm_can_bridge::readInt16BE(payload, 4));
    const double yaw_motor_pos_deg =
      rm_can_bridge::q10_6RawToDeg(rm_can_bridge::readInt16BE(payload, 6));

    if (debug_usb_can_frames_) {
      RCLCPP_INFO(
        get_logger(),
        "0x%03X decoded yaw=%.2f pitch=%.2f roll=%.2f yaw_motor=%.2f", feedback_can_id_, yaw_deg,
        pitch_deg, roll_deg, yaw_motor_pos_deg);
    }

    latest_feedback_yaw_deg_ = yaw_deg;
    latest_feedback_pitch_deg_ = pitch_deg;
    latest_feedback_stamp_ = this->now();
    feedback_valid_ = true;

    if (!publish_feedback_) {
      return;
    }

    geometry_msgs::msg::Vector3Stamped rpy_msg;
    rpy_msg.header.stamp = this->now();
    rpy_msg.header.frame_id = feedback_frame_id_;
    rpy_msg.vector.x = roll_deg;
    rpy_msg.vector.y = pitch_deg;
    rpy_msg.vector.z = yaw_deg;
    feedback_rpy_pub_->publish(rpy_msg);

    std_msgs::msg::Float64 yaw_motor_msg;
    yaw_motor_msg.data = yaw_motor_pos_deg;
    feedback_yaw_motor_pub_->publish(yaw_motor_msg);
  }

  bool isFresh(const rclcpp::Time & stamp, int timeout_ms) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).nanoseconds() <=
           static_cast<int64_t>(timeout_ms) * static_cast<int64_t>(1000000);
  }

  static int16_t toScaledInt16(double value, double scale)
  {
    const int64_t raw = static_cast<int64_t>(std::llround(value * scale));
    if (raw > 32767) {
      return 32767;
    }
    if (raw < -32768) {
      return -32768;
    }
    return static_cast<int16_t>(raw);
  }

  bool openSerialPort()
  {
    closeSerialPort();

    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Failed to open %s: %s",
        serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "tcgetattr failed: %s", std::strerror(errno));
      closeSerialPort();
      return false;
    }

#ifdef B921600
    speed_t baud = B921600;
#else
    speed_t baud = B115200;
    RCLCPP_WARN(get_logger(), "B921600 is unavailable on this platform, fallback to B115200");
#endif

    if (serial_baud_ != 921600) {
      RCLCPP_WARN(
        get_logger(), "serial_baud=%d, but current implementation applies 921600 preset only",
        serial_baud_);
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "tcsetattr failed: %s", std::strerror(errno));
      closeSerialPort();
      return false;
    }

    const int flags = fcntl(serial_fd_, F_GETFL, 0);
    if (flags >= 0) {
      (void)fcntl(serial_fd_, F_SETFL, flags | O_NONBLOCK);
    }
    tcflush(serial_fd_, TCIOFLUSH);
    serial_rx_buffer_.clear();

    RCLCPP_INFO(get_logger(), "USB-CAN serial opened: %s @ 921600", serial_port_.c_str());
    return true;
  }

  void scheduleReconnect()
  {
    next_reconnect_try_ =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(reconnect_interval_ms_);
  }

  bool ensureSerialReady()
  {
    if (serial_fd_ >= 0) {
      return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_try_) {
      return false;
    }

    if (!openSerialPort()) {
      scheduleReconnect();
      return false;
    }

    return true;
  }

  void closeSerialPort()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    serial_rx_buffer_.clear();
  }

  bool sendCanFrame(uint32_t can_id, const std::array<uint8_t, 8> & payload)
  {
    if (!ensureSerialReady()) {
      return false;
    }

    std::array<uint8_t, USB_CAN_TX_FRAME_SIZE> frame = {
      0x55, 0xAA, 0x1E, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88};

    frame[13] = static_cast<uint8_t>(can_id & 0xFF);
    frame[14] = static_cast<uint8_t>((can_id >> 8) & 0xFF);
    frame[15] = static_cast<uint8_t>((can_id >> 16) & 0xFF);
    frame[16] = static_cast<uint8_t>((can_id >> 24) & 0xFF);
    for (size_t i = 0; i < payload.size(); ++i) {
      frame[21 + i] = payload[i];
    }

    const ssize_t sent = write(serial_fd_, frame.data(), frame.size());
    if (sent != static_cast<ssize_t>(frame.size())) {
      const int saved_errno = errno;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "USB-CAN write failed (id=0x%03X, sent=%zd, errno=%d), reconnecting",
        can_id, sent, saved_errno);
      closeSerialPort();
      scheduleReconnect();
      return false;
    }
    return true;
  }

private:
  std::string serial_port_;
  int serial_baud_{921600};
  int reconnect_interval_ms_{1000};
  bool debug_serial_rx_{false};
  bool debug_usb_can_frames_{false};
  int debug_hex_bytes_{static_cast<int>(DEFAULT_DEBUG_HEX_BYTES)};
  int serial_fd_{-1};
  std::chrono::steady_clock::time_point next_reconnect_try_{
    std::chrono::steady_clock::time_point::min()};

  std::string cmd_vel_topic_;
  std::string armors_topic_;
  std::string target_topic_;
  std::string e_stop_topic_;
  std::string fire_enable_topic_;
  std::string avoid_active_topic_;
  std::string input_mode_{"detector_armors"};
  std::string aim_reference_mode_{"relative"};
  std::string angle_unit_{"deg"};
  std::string expected_target_frame_{"gimbal_link"};
  std::string feedback_frame_id_{"gimbal_link"};

  bool auto_fire_{false};
  bool zero_when_lost_{true};
  bool publish_feedback_{true};
  bool invert_yaw_axis_{false};
  bool invert_pitch_axis_{false};
  double linear_scale_{1000.0};
  double angular_scale_{1000.0};
  int nav_timeout_ms_{300};
  int aim_timeout_ms_{300};
  int target_timeout_ms_{300};
  int feedback_timeout_ms_{200};
  uint32_t feedback_can_id_{0x402};
  rm_can_bridge::Vec3 muzzle_offset_in_gimbal_{0.0, 0.0, 0.0};

  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time latest_cmd_vel_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_aim_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_feedback_stamp_{0, 0, RCL_ROS_TIME};
  int16_t latest_yaw_q10_6_{0};
  int16_t latest_pitch_q10_6_{0};
  double latest_feedback_yaw_deg_{0.0};
  double latest_feedback_pitch_deg_{0.0};
  std::vector<uint8_t> serial_rx_buffer_;
  bool feedback_valid_{false};
  rm_can_bridge::PendingAimLatency pending_aim_latency_{};
  auto_aim_interfaces::LatencyWindow can_send_latency_window_{std::chrono::milliseconds(1000)};

  bool e_stop_{false};
  bool fire_enable_{false};
  bool has_aim_command_{false};
  bool avoid_active_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr avoid_active_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr feedback_rpy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_yaw_motor_pub_;

  rclcpp::TimerBase::SharedPtr estop_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_cmd_timer_;
  rclcpp::TimerBase::SharedPtr chassis_cmd_timer_;
  rclcpp::TimerBase::SharedPtr serial_rx_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RmCanBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
