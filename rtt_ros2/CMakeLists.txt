cmake_minimum_required(VERSION 3.5)
project(rtt_ros2)

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
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(LibXml2 REQUIRED)
find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-marshalling rtt-scripting)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# setup targets
include_directories(include/orocos)
orocos_library(rtt_ros2
  src/rtt_ros2.cpp
  EXPORT ${PROJECT_NAME}
  INCLUDES DESTINATION include/orocos
)
# Note: Dependency ament_index_cpp is actually not PUBLIC, but
# ament_target_dependencies() does not support PRIVATE...
# (https://github.com/ament/ament_cmake/issues/158)
#ament_target_dependencies(rtt_ros2 PUBLIC ament_index_cpp)
ament_target_dependencies(rtt_ros2 ament_index_cpp)
target_link_libraries(rtt_ros2 Boost::filesystem)
if(TARGET LibXml2::LibXml2)
  target_link_libraries(rtt_ros2 LibXml2::LibXml2)
else()
  target_compile_definitions(rtt_ros2 PRIVATE ${LIBXML2_DEFINITIONS})
  target_include_directories(rtt_ros2 PRIVATE ${LIBXML2_INCLUDE_DIR})
  target_link_libraries(rtt_ros2 ${LIBXML2_LIBRARIES})
endif()

orocos_plugin(rtt_ros2_service
  src/rtt_ros2_service.cpp
)
target_link_libraries(rtt_ros2_service rtt_ros2)

# install
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

# export information to downstream packages
ament_export_dependencies(ament_index_cpp)
#ament_export_dependencies(Boost)  # exported in rtt_ros2-extras.cmake
#ament_export_dependencies(LibXml2)  # exported in rtt_ros2-extras.cmake
ament_export_include_directories(include/orocos)
if(COMMAND ament_export_targets)
  ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
else()
  ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()

# linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# export no plugin dependencies for this package, such that package.xml is not searched
# for recursive dependencies
ament_index_register_resource("rtt_ros2_plugin_depends" CONTENT "")

# register environment hook to prepend to RTT_COMPONENT_PATH
set(
  AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_rtt_component_path
  "prepend-non-duplicate;RTT_COMPONENT_PATH;lib/orocos"
)
ament_environment_hooks(env_hook/rtt_component_path.sh.in)

# must be called *after* the targets to check exported libraries etc.
ament_package(
  CONFIG_EXTRAS
    cmake/${PROJECT_NAME}-extras.cmake
    cmake/${PROJECT_NAME}_export_plugin_depend_hook-extras.cmake
    cmake/${PROJECT_NAME}_export_plugin_depend.cmake
    cmake/${PROJECT_NAME}_use_orocos.cmake
)

# orocos_generate_package() is deprecated for ROS 2.
# Prefer cmake target export and import instead, in combination with
# ament_export_interfaces() or ament_export_targets() when building with
# ament_cmake.
orocos_generate_package()
