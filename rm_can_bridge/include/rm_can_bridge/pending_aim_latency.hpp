#ifndef RM_CAN_BRIDGE__PENDING_AIM_LATENCY_HPP_
#define RM_CAN_BRIDGE__PENDING_AIM_LATENCY_HPP_

#include <rclcpp/time.hpp>

namespace rm_can_bridge
{

class PendingAimLatency
{
public:
  void markNewCommand(const rclcpp::Time & source_stamp)
  {
    source_stamp_ = source_stamp;
    pending_ = source_stamp_.nanoseconds() > 0;
  }

  void clear()
  {
    source_stamp_ = rclcpp::Time(0, 0, source_stamp_.get_clock_type());
    pending_ = false;
  }

  bool takePendingLatency(const rclcpp::Time & now, double & latency_ms)
  {
    if (!pending_ || source_stamp_.nanoseconds() <= 0) {
      return false;
    }

    latency_ms = (now - source_stamp_).seconds() * 1000.0;
    pending_ = false;
    return true;
  }

private:
  rclcpp::Time source_stamp_{0, 0, RCL_ROS_TIME};
  bool pending_{false};
};

}  // namespace rm_can_bridge

#endif  // RM_CAN_BRIDGE__PENDING_AIM_LATENCY_HPP_
