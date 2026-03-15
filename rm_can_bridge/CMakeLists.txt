cmake_minimum_required(VERSION 3.8)
project(rm_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)

add_executable(rm_can_bridge_node src/rm_can_bridge_node.cpp)
ament_target_dependencies(rm_can_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
  auto_aim_interfaces
)
target_include_directories(rm_can_bridge_node PUBLIC include)

target_compile_features(rm_can_bridge_node PUBLIC cxx_std_17)

install(TARGETS rm_can_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_target_aim_math test/test_target_aim_math.cpp)
  target_include_directories(test_target_aim_math PRIVATE include)

  ament_add_gtest(test_q10_6_angle test/test_q10_6_angle.cpp)
  target_include_directories(test_q10_6_angle PRIVATE include)

  ament_add_gtest(test_feedback_absolute_aim test/test_feedback_absolute_aim.cpp)
  target_include_directories(test_feedback_absolute_aim PRIVATE include)

  ament_add_gtest(test_usb_can_frame_parser test/test_usb_can_frame_parser.cpp)
  target_include_directories(test_usb_can_frame_parser PRIVATE include)

  ament_add_gtest(test_gimbal_command_frame test/test_gimbal_command_frame.cpp)
  target_include_directories(test_gimbal_command_frame PRIVATE include)

  ament_add_gtest(test_aim_axis_sign test/test_aim_axis_sign.cpp)
  target_include_directories(test_aim_axis_sign PRIVATE include)

  ament_add_gtest(test_pending_aim_latency test/test_pending_aim_latency.cpp)
  target_include_directories(test_pending_aim_latency PRIVATE include)
  ament_target_dependencies(test_pending_aim_latency rclcpp)
endif()

ament_package()
