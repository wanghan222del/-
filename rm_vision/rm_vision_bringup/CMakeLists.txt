cmake_minimum_required(VERSION 3.8)
project(rm_vision_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_vision_bringup_can_launch
    test/test_vision_bringup_can_launch.py
    PYTHON_EXECUTABLE /usr/bin/python3
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
