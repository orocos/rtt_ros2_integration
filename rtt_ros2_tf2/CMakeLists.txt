cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_tf2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_node REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# setup targets
include_directories(include/orocos)

# Target: library
# message(AUTHOR_WARNING "The tf2 includes are in: ${tf2_INCLUDE_DIRS}")
# message(AUTHOR_WARNING "The rclcpp includes are in: ${rclcpp_INCLUDE_DIRS}")
# message(AUTHOR_WARNING "The tf2_msgs includes are in: ${tf2_msgs_INCLUDE_DIRS}")
orocos_library(rtt_ros2_tf2
  src/rtt_ros2_tf2.cpp
  EXPORT ${PROJECT_NAME}
  INCLUDES DESTINATION include/orocos/${PROJECT_NAME}
)
# target_link_libraries(rtt_ros2_tf2 tf2)
# target_include_directories(rtt_ros2_tf2 tf2)
ament_target_dependencies(rtt_ros2_tf2
  geometry_msgs
  rclcpp
  rtt_ros2
  rtt_ros2_node
  tf2
  tf2_msgs
  tf2_ros
)
target_link_libraries(rtt_ros2_tf2
  rtt_ros2_node::rtt_ros2_node
)

# Target: orocos plugin
orocos_plugin(rtt_ros2_tf2_service
  src/rtt_ros2_tf2_service.cpp
)
target_link_libraries(rtt_ros2_tf2_service rtt_ros2_tf2)

# Target: orocos component
orocos_component(rtt_ros2_tf2_component
  src/rtt_ros2_tf2_component.cpp
)
target_link_libraries(rtt_ros2_tf2_component rtt_ros2_tf2)

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
ament_export_dependencies(
  geometry_msgs
  rclcpp
  tf2_msgs
  tf2_ros
  tf2
)
ament_export_include_directories(include/orocos)
if(COMMAND ament_export_targets)
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
else()
  ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()
rtt_ros2_export_plugin_depend(rtt_ros2_node)

# orocos_generate_package(
#   INCLUDE_DIRS include
#   DEPENDS tf2
#   DEPENDS_TARGETS rtt_geometry_msgs rtt_tf2_msgs
# )

# Legacy from rtt_tf
# Tests
# if(CATKIN_ENABLE_TESTING)
#   orocos_use_package(ocl-taskbrowser REQUIRED)
#   orocos_use_package(ocl-deployment REQUIRED)

#   add_definitions(-DRTT_COMPONENT)
#   orocos_executable(broadcaster_test tests/broadcaster_test.cpp)
#   target_link_libraries(broadcaster_test ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
#   orocos_executable(lookup_test tests/lookup_test.cpp)
#   target_link_libraries(lookup_test ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
# endif()

# must be called *after* the targets to check exported libraries etc.
ament_package()

# orocos_generate_package() is deprecated for ROS 2.
# Prefer cmake target export and import instead, in combination with
# ament_export_interfaces() or ament_export_targets() when building with
# ament_cmake.
orocos_generate_package()
