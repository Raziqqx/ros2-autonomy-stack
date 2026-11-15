cmake_minimum_required(VERSION 3.16)
project(control_core LANGUAGES CXX)

# Shared shape of every *_core package: builds as an ament package under colcon,
# or as a plain CMake project (native tests with GoogleTest from FetchContent,
# usable under sanitizers without ROS installed).
set(CORE_SOURCES src/pure_pursuit.cpp src/dwa.cpp src/path_tracking.cpp)
set(CORE_TESTS test/test_pure_pursuit.cpp test/test_dwa.cpp test/test_path_tracking.cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
find_package(ament_cmake QUIET)

add_library(${PROJECT_NAME} ${CORE_SOURCES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test ${CORE_TESTS})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  endif()
  ament_package()
else()
  include(CTest)
  if(BUILD_TESTING)
    if(NOT TARGET GTest::gtest_main)
      include(FetchContent)
      FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
      set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable(googletest)
    endif()
    add_executable(${PROJECT_NAME}_test ${CORE_TESTS})
    target_link_libraries(${PROJECT_NAME}_test PRIVATE ${PROJECT_NAME} GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(${PROJECT_NAME}_test)
  endif()
endif()
