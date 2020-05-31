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

macro(rtt_ros2_export_plugin_depend)
  if(${ARGC})
    message(STATUS
      "[rtt_ros2] Adding RTT plugin dependency '${ARGN}' for package "
      "'${PROJECT_NAME}'")
  else()
    message(WARNING
      "[rtt_ros2] rtt_ros2_export_plugin_depend() was called without "
      "arguments. This is okay if you want to skip parsing package.xml for RTT "
      "plugin dependencies.")
  endif()
  list(APPEND ${PROJECT_NAME}_RTT_ROS2_PLUGIN_DEPENDS "${ARGN}")
endmacro()
