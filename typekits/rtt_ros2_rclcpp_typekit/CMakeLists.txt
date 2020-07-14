cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_rclcpp_typekit)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtt_ros2 REQUIRED)

# build ros2-primitives typekit
orocos_typekit(rtt_ros2_rclcpp_typekit
  src/ros2_duration_type.cpp
  src/ros2_node_options_type.cpp
  src/ros2_parameter_type.cpp
  src/ros2_parameter_value_type.cpp
  src/ros2_publisher_options_type.cpp
  src/ros2_qos_type.cpp
  src/ros2_time_type.cpp
  src/rtt_ros2_rclcpp_typekit.cpp
  EXPORT ${PROJECT_NAME}
  INCLUDES DESTINATION include/orocos
)
target_include_directories(rtt_ros2_rclcpp_typekit
  PRIVATE include/orocos
)
ament_target_dependencies(rtt_ros2_rclcpp_typekit
  rclcpp
  rtt_ros2
)

# install
install(
  DIRECTORY include/
  DESTINATION include
)

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# export information to downstream packages
# ament_export_dependencies(rtt_ros2)
ament_export_include_directories(include/orocos)
if(COMMAND ament_export_targets)
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
else()
  ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()
rtt_ros2_export_plugin_depend(rtt_ros2_primitives_typekit)
rtt_ros2_export_plugin_depend(rtt_ros2_builtin_interfaces)

# must be called *after* the targets to check exported libraries etc.
ament_package()

# orocos_generate_package() is deprecated for ROS 2.
# Prefer cmake target export and import instead, in combination with
# ament_export_interfaces() or ament_export_targets() when building with
# ament_cmake.
orocos_generate_package(
  # DEPENDS_TARGETS rtt_ros2
)
