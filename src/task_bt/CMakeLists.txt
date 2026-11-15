cmake_minimum_required(VERSION 3.16)
project(task_bt LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(task_bt_node src/task_bt_node.cpp)
target_include_directories(task_bt_node PRIVATE include)
target_compile_options(task_bt_node PRIVATE -Wall -Wextra)
target_link_libraries(task_bt_node ${behaviortree_cpp_TARGETS} rclcpp::rclcpp ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_msgs_TARGETS})
install(TARGETS task_bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The mission trees with mock leaves: retry/recovery and patrol order.
  ament_add_gtest(task_bt_tree_test test/test_trees.cpp)
  target_include_directories(task_bt_tree_test PRIVATE include)
  target_link_libraries(task_bt_tree_test ${behaviortree_cpp_TARGETS})
  target_compile_definitions(task_bt_tree_test PRIVATE
    TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/mission.xml")
endif()
ament_package()
