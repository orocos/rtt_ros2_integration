cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_idl_tests)

if(NOT BUILD_TESTING)
  install(CODE "") # no-op
  return()
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_idl REQUIRED)
find_package(rtt_ros2_test_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# unit tests
find_package(ament_cmake_gtest REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/rtt_ros2_idl)
ament_add_gtest(test_typekit test/test_typekit.cpp)
orocos_configure_executable(test_typekit)
target_link_libraries(test_typekit
  rtt_ros2::rtt_ros2
  rtt_ros2_test_msgs::rtt_test_msgs_typekit
)
