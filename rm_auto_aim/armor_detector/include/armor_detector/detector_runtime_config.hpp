#ifndef ARMOR_DETECTOR__DETECTOR_RUNTIME_CONFIG_HPP_
#define ARMOR_DETECTOR__DETECTOR_RUNTIME_CONFIG_HPP_

#include <rclcpp/parameter.hpp>
#include <vector>

namespace rm_auto_aim
{

struct DetectorRuntimeConfig
{
  int binary_thres{80};
  int detect_color{0};
  double classifier_threshold{0.8};
  bool debug{false};
  double debug_image_fps{10.0};
  bool debug_publish_result_img{false};
  bool debug_publish_binary_img{false};
  bool debug_publish_number_img{false};
  int stats_log_period_ms{1000};

  void apply(const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "binary_thres") {
        binary_thres = parameter.as_int();
      } else if (name == "detect_color") {
        detect_color = parameter.as_int();
      } else if (name == "classifier_threshold") {
        classifier_threshold = parameter.as_double();
      } else if (name == "debug") {
        debug = parameter.as_bool();
      } else if (name == "debug_image_fps") {
        debug_image_fps = parameter.as_double();
      } else if (name == "debug_publish_result_img") {
        debug_publish_result_img = parameter.as_bool();
      } else if (name == "debug_publish_binary_img") {
        debug_publish_binary_img = parameter.as_bool();
      } else if (name == "debug_publish_number_img") {
        debug_publish_number_img = parameter.as_bool();
      } else if (name == "stats_log_period_ms") {
        stats_log_period_ms = parameter.as_int();
      }
    }
  }
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_RUNTIME_CONFIG_HPP_
