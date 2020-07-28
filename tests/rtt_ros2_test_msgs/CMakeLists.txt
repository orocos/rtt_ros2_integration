cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_test_msgs)

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
find_package(rtt_ros2_interfaces REQUIRED)

include_directories(include/orocos)
rtt_ros2_generate_interfaces_plugins(test_msgs
  EXTRA_INCLUDES
    ${PROJECT_NAME}/srv/empty.hpp
)

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# must be called *after* the targets to check exported libraries etc.
ament_package()

# orocos_generate_package() is deprecated for ROS 2.
# Prefer cmake target export and import instead, in combination with
# ament_export_interfaces() or ament_export_targets() when building with
# ament_cmake.
orocos_generate_package(
  DEPENDS_TARGETS rtt_ros2_interfaces
)
