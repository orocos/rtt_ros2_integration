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

#include <string>

#include "rtt/internal/GlobalService.hpp"

#include "rtt_ros2/rtt_ros2.hpp"

namespace rtt_ros2
{

static void loadROSService()
{
  RTT::Service::shared_ptr ros =
    RTT::internal::GlobalService::Instance()->provides("ros");
  ros->doc("ROS operations and services");

  ros->addOperation("find", &rtt_ros2::find)
  .doc(
    "Finds the given ROS package and returns the full path to its share "
    "directory.")
  .arg("package", "The ROS package name");

  ros->addOperation("import", &rtt_ros2::import)
  .doc(
    "Imports the Orocos plugins from a given ROS package (if found) "
    "along with the plugins of all of the package's <plugin_depend> "
    "dependencies as listed in the package.xml.")
  .arg("package", "The ROS package name");
}

extern "C" {
bool loadRTTPlugin(RTT::TaskContext * tc)
{
  if (tc != nullptr) {return false;}
  loadROSService();
  return true;
}
std::string getRTTPluginName() {return "ros2";}
std::string getRTTTargetName() {return OROCOS_TARGET_NAME;}
}

}  // namespace rtt_ros2
