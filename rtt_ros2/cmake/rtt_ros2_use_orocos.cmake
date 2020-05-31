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

#
# Find Orocos RTT and its CMake macros (UseOROCOS-RTT.cmake)
#
# By default this macro also finds the RTT core plugins rtt-marshalling and
# rtt-scripting. To include more plugins, you can add them as additional
# arguments.
#
# The rtt_ros2_use_orocos() macro can be called more than once.
#
macro(rtt_ros2_use_orocos)
  find_package(OROCOS-RTT REQUIRED
    COMPONENTS rtt-marshalling rtt-scripting ${ARGN})
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
endmacro()

rtt_ros2_use_orocos()
