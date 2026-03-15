cmake_minimum_required(VERSION 3.8)
project(auto_aim_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"

  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/TrackerInfo.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
)


install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_latency_window test/test_latency_window.cpp)
  target_include_directories(test_latency_window PRIVATE include)
endif()

ament_package()
