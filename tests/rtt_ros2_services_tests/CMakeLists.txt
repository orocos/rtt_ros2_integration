cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_services_tests)

# Skip building this package if BUILD_TESTING is OFF
option(BUILD_TESTING "Build the testing tree." ON)
if(NOT BUILD_TESTING)
  install(CODE
    "message(STATUS \"Skipping installation of package ${PROJECT_NAME} because BUILD_TESTING is OFF.\")"
  )
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
find_package(rtt_ros2_services REQUIRED)
find_package(rtt_ros2_test_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# unit tests
find_package(ament_cmake_gtest REQUIRED)

# The test_ros_service fails with
#
#     double free or corruption (fasttop)
#
# during destruction in ROS dashing. This seems to be a known problem:
# https://gitlab.com/ApexAI/performance_test/-/issues/87
# ==> workaround: disable the test for dashing
#
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(ROS_DISTRO STREQUAL "dashing")
  return()
endif()

ament_add_gtest(test_ros_service test/test_ros_service.cpp)
orocos_configure_executable(test_ros_service)
ament_target_dependencies(test_ros_service test_msgs)
target_link_libraries(test_ros_service
  rtt_ros2::rtt_ros2
  rtt_ros2_node::rtt_ros2_node
  rtt_ros2_services::rtt_ros2_services
)
target_compile_definitions(test_ros_service
  PRIVATE
    test_msgs_VERSION_MAJOR=${test_msgs_VERSION_MAJOR}
    test_msgs_VERSION_MINOR=${test_msgs_VERSION_MINOR}
    test_msgs_VERSION_PATCH=${test_msgs_VERSION_PATCH}
)
