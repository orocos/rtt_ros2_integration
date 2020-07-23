cmake_minimum_required(VERSION 3.5)
project(rtt_ros2_tests)

# Skip building this package if BUILD_TESTING is OFF
option(BUILD_TESTING "Build the testing tree." ON)
if(NOT BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  ament_package()
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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rtt_ros2 REQUIRED)

# declare RTT plugin dependencies
rtt_ros2_export_plugin_depend()  # empty (to skip parsing package.xml on import even if there would be no other plugin dependencies declared in CMakeLists.txt)
rtt_ros2_export_plugin_depend(rtt_ros2)

# build test service plugin
orocos_service(test_service src/test_service.cpp)

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# must be called *after* the targets to check exported libraries etc.
ament_package()

# unit tests
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_find test/test_find.cpp)
orocos_configure_executable(test_find)
ament_target_dependencies(test_find rtt_ros2)
target_link_libraries(test_find rtt_ros2::rtt_ros2)
target_compile_definitions(test_find PRIVATE "-DCMAKE_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\"")

ament_add_gtest(test_import test/test_import.cpp)
orocos_configure_executable(test_import)
ament_target_dependencies(test_import rtt_ros2)
target_link_libraries(test_import rtt_ros2::rtt_ros2)
