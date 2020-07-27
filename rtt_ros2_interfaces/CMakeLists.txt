cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rtt_ros2_idl REQUIRED)
find_package(rtt_ros2_topics REQUIRED)

# install typekit package template and create_typekit_package script
install(
  DIRECTORY template
  DESTINATION share/${PROJECT_NAME}/
)
install(PROGRAMS scripts/create_typekit_package
  DESTINATION lib/${PROJECT_NAME}/
)

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# export information to downstream packages
ament_export_dependencies(rtt_ros2_idl)
ament_export_dependencies(rtt_ros2_topics)

# must be called *after* the targets to check exported libraries etc.
ament_package(
  CONFIG_EXTRAS
    cmake/${PROJECT_NAME}-generate_interfaces_plugins.cmake
)

# orocos_generate_package() is deprecated for ROS 2.
# Prefer cmake target export and import instead, in combination with
# ament_export_interfaces() or ament_export_targets() when building with
# ament_cmake.
orocos_generate_package(
  DEPENDS_TARGETS
    rtt_ros2_idl
    rtt_ros2_topics
)
