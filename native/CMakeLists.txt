# Native (ROS-free) superbuild of the pure-maths packages, used for fast local
# testing under AddressSanitizer/UndefinedBehaviorSanitizer and in CI without
# ROS. Each package also builds on its own and as an ament package via colcon.
cmake_minimum_required(VERSION 3.16)
project(autonomy_core_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(SANITIZE "Build with ASan + UBSan" ON)
if(SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

include(CTest)
include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(CORE_PACKAGES ekf_core icp_core grid_core planning_core control_core
  CACHE STRING "Packages to build (semicolon list)")
foreach(pkg ${CORE_PACKAGES})
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../src/${pkg} ${pkg})
endforeach()
