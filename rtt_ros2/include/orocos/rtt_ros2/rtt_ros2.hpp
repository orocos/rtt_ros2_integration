// Copyright 2020 Intermodalics BVBA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OROCOS__RTT_ROS2__RTT_ROS2_HPP_
#define OROCOS__RTT_ROS2__RTT_ROS2_HPP_

#include <string>

namespace rtt_ros2
{

/// Find a ROS package's share directory
/**
 * @param[in] package The package name
 * @returns The full path to the package's share directory if it has been found.
 *          Otherwise an empty string is returned.
 */
std::string find(const std::string & package);

/// Import a ROS package and all of its rtt_ros2/plugin_depend dependencies
/**
 * @param[in] package The package name
 * @returns true on success. The packages might already have been imported before,
 *          which is not considered a failure.
 */
bool import(const std::string & package);

}  // namespace rtt_ros2

#endif  // OROCOS__RTT_ROS2__RTT_ROS2_HPP_
