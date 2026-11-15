cmake_minimum_required(VERSION 3.16)
project(autonomy_bringup NONE)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Scenario: fixed /cmd_vel pattern; asserts EKF error vs ground truth and
  # that the mapper produced occupied cells.
  add_launch_test(test/test_localization_mapping.py TIMEOUT 180)
endif()

ament_package()
