cmake_minimum_required(VERSION 3.16)
project(local_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_core REQUIRED)

add_executable(local_controller_node src/local_controller_node.cpp)
target_compile_options(local_controller_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(local_controller_node control_core::control_core rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})
install(TARGETS local_controller_node DESTINATION lib/${PROJECT_NAME})
ament_package()
