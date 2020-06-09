cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_idl)

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
find_package(ament_cmake_python REQUIRED)
find_package(rtt_ros2 REQUIRED)

# setup Python module
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# install
install(PROGRAMS
  bin/generate_typekit
  DESTINATION lib/${PROJECT_NAME}
)
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
ament_export_dependencies(rtt_ros2)
ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_cpp)
ament_export_dependencies(rosidl_typesupport_introspection_cpp)
ament_export_include_directories(include/orocos)

# must be called *after* the targets to check exported libraries etc.
ament_package(
  CONFIG_EXTRAS
    cmake/${PROJECT_NAME}-extras.cmake.in
    cmake/${PROJECT_NAME}_generate_typekit.cmake
)

# orocos_generate_package() is deprecated for ROS 2.
# Prefer cmake target export and import instead, in combination with
# ament_export_interfaces() or ament_export_targets() when building with
# ament_cmake.
orocos_generate_package(
  DEPENDS_TARGETS rtt_ros2
)
