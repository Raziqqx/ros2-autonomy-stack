cmake_minimum_required(VERSION 3.16)
project(grid_mapping LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_core REQUIRED)

add_executable(grid_mapping_node src/grid_mapping_node.cpp)
target_compile_options(grid_mapping_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(grid_mapping_node grid_core::grid_core rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})
install(TARGETS grid_mapping_node DESTINATION lib/${PROJECT_NAME})
ament_package()
