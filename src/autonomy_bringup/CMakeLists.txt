cmake_minimum_required(VERSION 3.16)
project(autonomy_bringup NONE)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
# Evaluation tools (run recorder, metrics, experiment runner) as executables.
install(PROGRAMS scripts/nav_recorder.py scripts/run_eval.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/nav_metrics.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Scenario: fixed /cmd_vel pattern; asserts EKF error vs ground truth and
  # that the mapper produced occupied cells.
  add_launch_test(test/test_localization_mapping.py TIMEOUT 180)
  # Full stack: reach the goal within the time and ATE budgets.
  add_launch_test(test/test_navigation.py TIMEOUT 180)
  # A wall dropped in front of the robot: the BT must recover and still arrive.
  add_launch_test(test/test_recovery.py TIMEOUT 240)
  # Metrics maths (ATE, RPE, curvature) without ROS.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(nav_metrics test/test_nav_metrics.py)
endif()

ament_package()
