cmake_minimum_required(VERSION 3.16)
project(sim2d LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_core REQUIRED)

add_executable(sim2d_node src/sim2d_node.cpp)
target_compile_options(sim2d_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(sim2d_node grid_core::grid_core rclcpp::rclcpp tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})
install(TARGETS sim2d_node DESTINATION lib/${PROJECT_NAME})
ament_package()
