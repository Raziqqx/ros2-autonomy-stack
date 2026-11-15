cmake_minimum_required(VERSION 3.16)
project(global_planner LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grid_core REQUIRED)
find_package(planning_core REQUIRED)

add_executable(global_planner_node src/global_planner_node.cpp)
target_compile_options(global_planner_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(global_planner_node grid_core::grid_core planning_core::planning_core
  rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS})
install(TARGETS global_planner_node DESTINATION lib/${PROJECT_NAME})
ament_package()
