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

#ifndef OROCOS__RTT_ROS2_TOPICS__ROS_TYPE_TRANSPORTER_HPP_
#define OROCOS__RTT_ROS2_TOPICS__ROS_TYPE_TRANSPORTER_HPP_

#include <utility>

#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include "rtt/DataFlowInterface.hpp"
#include "rtt/Logger.hpp"
#include "rtt/TaskContext.hpp"
#include "rtt/internal/ConnFactory.hpp"
#include "rtt/types/TypeTransporter.hpp"
#include "rtt_ros2_node/rtt_ros2_node.hpp"

#include "rtt_ros2_topics/protocol_id.h"
#include "rtt_ros2_topics/ros_publisher.hpp"
#include "rtt_ros2_topics/ros_subscription.hpp"

namespace rtt_ros2_topics
{

template<typename T>
class RosTypeTransporter : public RTT::types::TypeTransporter
{
public:
  virtual RTT::base::ChannelElementBase::shared_ptr createStream(
    RTT::base::PortInterface * port,
    const RTT::ConnPolicy & policy,
    bool is_sender) const
  {
    // Pull connections are not supported by the ROS message transport.
    if (policy.pull) {
      RTT::log(RTT::Error) <<
        "Pull connections are not supported by the ROS message transport." <<
        RTT::endlog();
      return RTT::base::ChannelElementBase::shared_ptr();
    }

    // Retrieve rclcpp::Node::SharedPtr to be used for this port
    const RTT::DataFlowInterface * interface = port->getInterface();
    RTT::TaskContext * owner = (interface != nullptr ? interface->getOwner() : nullptr);
    rclcpp::Node::SharedPtr node = rtt_ros2_node::getNode(owner);
    if (node == nullptr) {
      RTT::log(RTT::Error) <<
        "Cannot create a ROS topic stream for port " << port->getName();
      if (owner != nullptr) {
        RTT::log() <<
          " owned by component " << owner->getName();
      }
      RTT::log() <<
        ". Did you load the Node service" <<
        " into either this component or initialized a global ROS node for this process?" <<
        RTT::endlog();
      return RTT::base::ChannelElementBase::shared_ptr();
    }

    RTT::log(RTT::Debug) <<
      "Using node '" << node->get_fully_qualified_name() << "'" <<
      " to create a new ROS topic stream for port " << port->getName() <<
      " owned by component " << owner->getName() << RTT::endlog();

    RTT::base::ChannelElementBase::shared_ptr channel;
    if (is_sender) {
      channel = new RosPublisherChannelElement<T>(port, policy, std::move(node));

      if (policy.type == RTT::ConnPolicy::UNBUFFERED) {
        RTT::log(RTT::Debug) << "Creating unbuffered publisher connection for port " <<
          port->getName() << ". This may not be real-time safe!" << RTT::endlog();
        return channel;
      }

      RTT::base::ChannelElementBase::shared_ptr buf =
        RTT::internal::ConnFactory::buildDataStorage<T>(policy);
      if (!buf) {return RTT::base::ChannelElementBase::shared_ptr();}
      buf->connectTo(channel);
      return buf;

    } else {
      channel = new RosSubscriptionChannelElement<T>(port, policy, std::move(node));
      return channel;
    }

    // unreachable
    return RTT::base::ChannelElementBase::shared_ptr();
  }
};

}  // namespace rtt_ros2_topics

#endif  // OROCOS__RTT_ROS2_TOPICS__ROS_TYPE_TRANSPORTER_HPP_
