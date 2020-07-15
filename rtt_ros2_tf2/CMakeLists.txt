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

# Include RTT, this should include orocos_*() macros
find_package(OROCOS-RTT REQUIRED)
if(NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(rtt_ros2_geometry_msgs REQUIRED)
# find_package(rtt_tf2_msgs REQUIRED)

# Include rtt_ros2 for the macro: rtt_ros2_export_plugin_depend()
find_package(rtt_ros2 REQUIRED)

# setup targets
include_directories(include)

# Target: library
orocos_library(rtt_ros2_tf2
  src/rtt_ros2_tf2.cpp
  EXPORT ${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
# target_link_libraries(rtt_ros2_tf2 tf2)
ament_target_dependencies(rtt_ros2_tf2
  rclcpp tf2 rtt_ros2_geometry_msgs
)

# Target: orocos plugin
orocos_plugin(rtt_ros2_tf2_service
  src/rtt_ros2_tf2_service.cpp
)
target_link_libraries(rtt_ros2_tf2_service rtt_ros2_tf2)

# Target: orocos component
orocos_component(rtt_ros2_tf2_component
  src/rtt_ros2_tf2_component.cpp)
ament_target_dependencies(rtt_ros2_tf2_component
  OROCOS-RTT
)
# target_link_libraries(rtt_ros2_tf2_component ${catkin_LIBRARIES})

# Target: typekit (from tf2)
# ros_generate_rtt_typekit(tf2)

# install
install(
  DIRECTORY include/
  DESTINATION include
)

# orocos_install_headers(include/rtt_ros2_tf2/rtt_ros2_tf2.hpp)
# orocos_install_headers(include/rtt_ros2_tf2/tf2_interface.h)
# orocos_install_headers(include/rtt_ros2_tf2/rtt_ros2_tf2_component.hpp)

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# export information to downstream packages
# ament_export_dependencies(rtt_ros2)
ament_export_dependencies(rclcpp tf2)
ament_export_include_directories(include/${PROJECT_NAME})
if(COMMAND ament_export_targets)
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
else()
  ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()
rtt_ros2_export_plugin_depend(rclcpp)
rtt_ros2_export_plugin_depend(tf2)
rtt_ros2_export_plugin_depend(rtt_ros2_tf2)
# rtt_ros2_export_plugin_depend(rtt_ros2_tf2_msgs) # If typekit is generated

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
