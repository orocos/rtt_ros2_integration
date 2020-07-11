# Copyright 2020 Intermodalics BVBA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if(NOT COMMAND string_camel_case_to_lower_case_underscore)
  find_package(rosidl_cmake REQUIRED)
  include("${rosidl_cmake_DIR}/string_camel_case_to_lower_case_underscore.cmake")
endif()

#
# Generate an RTT transport plugin for ROS message types.
#
function(_rtt_ros2_generate_ros_transport _package)
  cmake_parse_arguments(ARG "" "BUILD_TYPE;TARGET;EXPORT" "MESSAGES;EXCLUDE_MESSAGES" ${ARGN})

  # Find the requested package if it was not found before,
  if(NOT ${_package}_FOUND)
    find_package(${_package} REQUIRED)
  endif()

  # The target name of the ros_transport library
  set(_target "${ARG_TARGET}")

  # The export set
  if(ARG_EXPORT)
    set(_export ${ARG_EXPORT})
  else()
    set(_export ${_target})
  endif()

  # Set CMAKE_BUILD_TYPE
  if(ARG_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${ARG_BUILD_TYPE})
  elseif(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE MinSizeRel)
    message(STATUS "[rtt_ros2_topics] Setting CMAKE_BUILD_TYPE to ${CMAKE_BUILD_TYPE} for target ${_target}")
  endif()

  # Add plugin dependencies
  rtt_ros2_export_plugin_depend(rtt_ros2_node)
  rtt_ros2_export_plugin_depend(rtt_ros2_topics)
  set(${PROJECT_NAME}_EXEC_DEPENDS "${${PROJECT_NAME}_EXEC_DEPENDS}" PARENT_SCOPE)
  set(${PROJECT_NAME}_RTT_ROS2_PLUGIN_DEPENDS "${${PROJECT_NAME}_RTT_ROS2_PLUGIN_DEPENDS}" PARENT_SCOPE)

  # Find all (selected) types in the given package.
  set(_include_dir "${${_package}_DIR}/../../../include/${_package}")
  normalize_path(_include_dir "${_include_dir}")
  set(_output_dir "${CMAKE_CURRENT_BINARY_DIR}/rtt_ros2_topics/${_package}/ros_transport")
  set(_messages "")
  set(_cpp_headers "")
  set(_generated_header_files "")
  set(_generated_source_files "")
  foreach(_file ${${_package}_INTERFACE_FILES})
    get_filename_component(_path "${_file}" DIRECTORY)
    get_filename_component(_extension "${_file}" EXT)
    get_filename_component(_type "${_file}" NAME_WE)
    string_camel_case_to_lower_case_underscore("${_type}" _header_name)
    if(_path STREQUAL "msg" AND _extension STREQUAL ".msg")
      if((NOT DEFINED ARG_MESSAGES OR ${_type} IN_LIST ARG_MESSAGES)
          AND NOT ${_type} IN_LIST ARG_EXCLUDE_MESSAGES)
        list(APPEND _messages ${_type})
      endif()
    endif()
  endforeach()
  list(APPEND _generated_source_files "${_output_dir}/ros_transport_plugin.cpp")

  # Generate source and header files
  add_custom_command(
    OUTPUT ${_generated_header_files} ${_generated_source_files}
    COMMAND ${PYTHON_EXECUTABLE} -m rtt_ros2_topics
      --package "${_package}"
      --messages ${_messages}
      --output-dir "${_output_dir}"
    DEPENDS
      ${rtt_ros2_topics_GENERATOR_FILES}
      "${rtt_ros2_topics_TEMPLATE_DIR}/ros_transport_plugin.cpp.em"
      "${_cpp_headers}"
    COMMENT "Generating RTT ROS transport plugin for ${_package}"
    VERBATIM
  )

  # Build the transport library target
  orocos_typekit(${_target}
    ${_generated_header_files}
    ${_generated_source_files}
    EXPORT ${_export}
    INCLUDES DESTINATION include/orocos
  )
  target_include_directories(${_target}
    PRIVATE
      ${_output_dir}
  )
  ament_target_dependencies(${_target}
    ${_package}
    rclcpp
    rtt_ros2_node
    rtt_ros2_topics
  )
  # Note: Not sure why this is required. It should not be required anymore in ROS foxy and upwards.
  target_link_libraries(${_target}
    ${rtt_ros2_node_INTERFACES}
  )

  # Install headers
  install(
    DIRECTORY ${_output_dir}/
    DESTINATION include/orocos/${_package}/typekit
    FILES_MATCHING PATTERN "*.hpp"
  )
  ament_export_include_directories(include/orocos)

  # Set variables in PARENT_SCOPE necessary for .pc file generation (orocos_generate_package())
  set(OROCOS_DEFINED_TYPES ${OROCOS_DEFINED_TYPES} PARENT_SCOPE)
  set(${PROJECT_NAME}_EXPORTED_LIBRARIES ${${PROJECT_NAME}_EXPORTED_LIBRARIES} PARENT_SCOPE)
  set(${PROJECT_NAME}_EXPORTED_LIBRARY_DIRS ${${PROJECT_NAME}_EXPORTED_LIBRARY_DIRS} PARENT_SCOPE)
endfunction()

macro(rtt_ros2_generate_ros_transport _package)
  cmake_parse_arguments(ARG "" "TARGET" "" ${ARGN})

  # Target name defaults to ${_package}_ros_transport.
  if(NOT ARG_TARGET)
    set(_target "rtt_${_package}_ros_transport")
  else()
    set(_target ${ARG_TARGET})
  endif()

  _rtt_ros2_generate_ros_transport(${_package}
    TARGET ${_target}
    ${ARG_UNPARSED_ARGUMENTS}
  )

  # Export dependencies, include directories and interface
  ament_export_dependencies(${_package})
  ament_export_include_directories(include/orocos)
  if(COMMAND ament_export_targets)
    ament_export_targets(${_target} HAS_LIBRARY_TARGET)
  else()
    ament_export_interfaces(${_target} HAS_LIBRARY_TARGET)
  endif()
endmacro()
