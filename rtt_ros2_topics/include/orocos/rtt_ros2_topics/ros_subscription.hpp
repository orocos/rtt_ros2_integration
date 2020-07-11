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

#ifndef OROCOS__RTT_ROS2_TOPICS__ROS_SUBSCRIPTION_HPP_
#define OROCOS__RTT_ROS2_TOPICS__ROS_SUBSCRIPTION_HPP_

#include <algorithm>
#include <string>
#include <utility>

#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"

#include "rtt/ConnPolicy.hpp"
#include "rtt/TaskContext.hpp"
#include "rtt/base/ChannelElement.hpp"
#include "rtt/base/PortInterface.hpp"

#include "rtt_ros2_topics/utilities.hpp"

namespace rtt_ros2_topics
{

template<typename T>
class RosSubscriptionChannelElement
  : public RTT::base::ChannelElement<T>
{
public:
  RosSubscriptionChannelElement(
    RTT::base::PortInterface * port,
    const RTT::ConnPolicy & policy,
    rclcpp::Node::SharedPtr node)
  : node_(std::move(node))
  {
    const std::string topic = utilities::find_topic(port, policy);
    if (port->getInterface() && port->getInterface()->getOwner()) {
      RTT::log(RTT::Debug) <<
        "Creating ROS subscription for port " <<
        port->getInterface()->getOwner()->getName() << "." << port->getName() << " on topic " <<
        topic << RTT::endlog();
    } else {
      RTT::log(RTT::Debug) <<
        "Creating ROS subscription for port " << port->getName() <<
        " on topic " << topic << RTT::endlog();
    }

    rclcpp::QoS qos(static_cast<std::size_t>(std::max(1, policy.size)));
    if (policy.init) {qos.transient_local();}
    subscription_ = node_->create_subscription<T>(
      topic, qos, std::bind(&RosSubscriptionChannelElement::callback, this, std::placeholders::_1));
  }

  virtual ~RosSubscriptionChannelElement() = default;

  void callback(const typename T::SharedPtr msg)
  {
    (void) this->write(*msg);
  }

  bool inputReady() override
  {
    return true;
  }

  bool isRemoteElement() const override
  {
    return true;
  }

  std::string getElementName() const override
  {
    return "RosSubscriptionChannelElement";
  }

  std::string getRemoteURI() const override
  {
    return subscription_->get_topic_name();
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
};

}  // namespace rtt_ros2_topics

#endif  // OROCOS__RTT_ROS2_TOPICS__ROS_SUBSCRIPTION_HPP_
