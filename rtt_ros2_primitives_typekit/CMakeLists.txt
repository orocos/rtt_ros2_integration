cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_primitives_typekit)

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
find_package(rtt_ros2 REQUIRED)

# build ros2-primitives typekit
orocos_typekit(rtt_ros2_primitives
  src/typekit/ros2_bool_type.cpp
  src/typekit/ros2_float32_type.cpp
  src/typekit/ros2_float64_type.cpp
  src/typekit/ros2_int16_type.cpp
  src/typekit/ros2_int32_type.cpp
  src/typekit/ros2_int64_type.cpp
  src/typekit/ros2_int8_type.cpp
  src/typekit/ros2_primitives_typekit.cpp
  src/typekit/ros2_string_type.cpp
  src/typekit/ros2_uint16_type.cpp
  src/typekit/ros2_uint32_type.cpp
  src/typekit/ros2_uint64_type.cpp
  src/typekit/ros2_uint8_type.cpp
  src/typekit/ros2_wstring_type.cpp
  EXPORT export_${PROJECT_NAME}
)
target_include_directories(rtt_ros2_primitives
  PRIVATE include/orocos
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
ament_export_interfaces(export_${PROJECT_NAME})

# must be called *after* the targets to check exported libraries etc.
ament_package()

# orocos_generate_package() is deprecated for ROS 2.
# Prefer cmake target export and import instead, in combination with
# ament_export_interfaces() when building with ament_cmake.
orocos_generate_package(
  # DEPENDS_TARGETS rtt_ros2
)
