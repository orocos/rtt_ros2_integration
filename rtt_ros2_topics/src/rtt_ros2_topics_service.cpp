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
#include "rtt/plugin/ServicePlugin.hpp"

#include "rtt_ros2_topics/topic.hpp"

namespace rtt_ros2_topics
{

static void loadGlobalROSService()
{
  RTT::Service::shared_ptr ros =
    RTT::internal::GlobalService::Instance()->provides("ros");
  ros->doc("ROS operations and services");

  ros->addOperation("topic", &topic)
  .doc("Returns a ConnPolicy object for streaming to or from the given ROS topic.")
  .arg("topic", "The topic")
  .arg(
    "latch",
    "If true, new subscribers will see old messages published before, i.e. "
    "set durability to transient local.");

  ros->addOperation("topicLatched", &topicLatched)
  .doc("Returns a ConnPolicy object for streaming to or from the given ROS topic (latched).")
  .arg("topic", "The topic");

  ros->addOperation("topicBuffered", &topicBuffered)
  .doc("Returns a ConnPolicy object for streaming to or from the given ROS topic.")
  .arg("topic", "The topic")
  .arg("size", "The size of buffers created by RTT and queues in the ROS middleware (depth)")
  .arg(
    "latch",
    "If true, new subscribers will see old messages published before, i.e. "
    "set durability to transient local.");

  ros->addOperation("topicDirect", &topicDirect)
  .doc("Returns a ConnPolicy object for streaming to or from the given ROS topic.")
  .arg("topic", "The topic")
  .arg("size", "The size of queues in the ROS middleware (depth)")
  .arg(
    "latch",
    "If true, new subscribers will see old messages published before, i.e. "
    "set durability to transient local.");
}

extern "C" {
bool loadRTTPlugin(RTT::TaskContext * tc)
{
  // This service cannot be loaded into individual components.
  if (tc != nullptr) {return false;}
  loadGlobalROSService();
  return true;
}
std::string getRTTPluginName() {return "rostopic";}
std::string getRTTTargetName() {return OROCOS_TARGET_NAME;}
}

}  // namespace rtt_ros2_topics
