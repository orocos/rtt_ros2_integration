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

#ifndef OROCOS__RTT_ROS2_TOPICS__TOPIC_HPP_
#define OROCOS__RTT_ROS2_TOPICS__TOPIC_HPP_

#include <string>
#include <utility>

#include "rtt/ConnPolicy.hpp"

#include "rtt_ros2_topics/protocol_id.h"

namespace rtt_ros2_topics
{

/**
 * Returns a ConnPolicy object for streaming to or from the given ROS topic.
 * Only a single data object is buffered within RTT and ROS, which replaces all earlier samples
 * that have not been actually published or read yet.
 * If the topic is empty, use the port name converted to lower case with underscores.
 */
static RTT::ConnPolicy topic(std::string topic, bool latch = false)
{
  RTT::ConnPolicy policy = RTT::ConnPolicy::data();
  policy.init = latch;
  policy.name_id = std::move(topic);
  policy.transport = ORO_ROS2_PROTOCOL_ID;
  return policy;
}

/**
 * Returns a ConnPolicy object for buffered streaming to or from the given ROS topic.
 * At least size elements are kept in RTT and ROS buffers/queues.
 * If the topic is empty, use the port name converted to lower case with underscores.
 */
static RTT::ConnPolicy topicBuffered(std::string topic, int size, bool latch = false)
{
  RTT::ConnPolicy policy = RTT::ConnPolicy::buffer(size);
  policy.init = latch;
  policy.name_id = std::move(topic);
  policy.transport = ORO_ROS2_PROTOCOL_ID;
  return policy;
}

/**
 * Returns a ConnPolicy object for direct (unbuffered) streaming to a given ROS topic (publishing).
 * The returned policy is invalid for input ports/subscribers.
 * No data is buffered by RTT. The publish() method is called in the thread of the writing
 * TaskContext (not real-time safe if rclcpp publishers are not real-time safe).
 * If the topic is empty, use the port name converted to lower case with underscores.
 */
static RTT::ConnPolicy topicDirect(
  std::string topic, int size, bool latch = false)
{
  RTT::ConnPolicy policy = RTT::ConnPolicy::data();
  policy.init = latch;
  policy.name_id = std::move(topic);
  policy.transport = ORO_ROS2_PROTOCOL_ID;
  policy.type = RTT::ConnPolicy::UNBUFFERED;
  return policy;
}

}  // namespace rtt_ros2_topics

#endif  // OROCOS__RTT_ROS2_TOPICS__TOPIC_HPP_
