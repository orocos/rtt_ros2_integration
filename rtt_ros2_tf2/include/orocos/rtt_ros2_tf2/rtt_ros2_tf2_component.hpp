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

#ifndef OROCOS__RTT_ROS2_TF2__RTT_ROS2_TF2_COMPONENT_HPP_
#define OROCOS__RTT_ROS2_TF2__RTT_ROS2_TF2_COMPONENT_HPP_

#include <string>

#include "rtt/RTT.hpp"
// #include "tf2_msgs/TFMessage.h"
// #include "tf2/buffer_core.h"

namespace rtt_ros2_tf2
{
// Inherit from TaskContext
class TF2_Component : public RTT::TaskContext
{
public:
  explicit TF2_Component(std::string const & name);

  bool configureHook();

  void updateHook();

  void cleanupHook();
};

}  // namespace rtt_ros2_tf2

#endif  // OROCOS__RTT_ROS2_TF2__RTT_ROS2_TF2_COMPONENT_HPP_
