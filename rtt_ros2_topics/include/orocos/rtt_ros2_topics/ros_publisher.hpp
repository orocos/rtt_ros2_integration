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

#ifndef OROCOS__RTT_ROS2_TOPICS__ROS_PUBLISHER_HPP_
#define OROCOS__RTT_ROS2_TOPICS__ROS_PUBLISHER_HPP_

#include <algorithm>
#include <string>
#include <utility>

#include "rcl/guard_condition.h"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/waitable.hpp"

#include "rtt/ConnPolicy.hpp"
#include "rtt/Logger.hpp"
#include "rtt/TaskContext.hpp"
#include "rtt/base/ChannelElement.hpp"
#include "rtt/base/PortInterface.hpp"

#include "rtt_ros2/detail/rclcpp_version.h"
#include "rtt_ros2_topics/utilities.hpp"
#include "rtt_ros2_topics/waitable.hpp"

namespace rtt_ros2_topics
{

template<typename T>
class RosPublisherChannelElement
  : public RTT::base::ChannelElement<T>
{
public:
  RosPublisherChannelElement(
    RTT::base::PortInterface * port,
    const RTT::ConnPolicy & policy,
    rclcpp::Node::SharedPtr node)
  : node_(std::move(node)),
    waitable_(Waitable::make_shared(
        std::bind(&RosPublisherChannelElement::publish, this),
        node_->get_node_base_interface()->get_context()))
  {
    const std::string topic = utilities::find_topic(port, policy);
    if (port->getInterface() && port->getInterface()->getOwner()) {
      RTT::log(RTT::Debug) <<
        "Creating ROS publisher for port " <<
        port->getInterface()->getOwner()->getName() << "." << port->getName() <<
        " on topic " << topic <<
        " with policy " << policy << RTT::endlog();
    } else {
      RTT::log(RTT::Debug) <<
        "Creating ROS publisher for port " << port->getName() <<
        " on topic " << topic <<
        " with policy " << policy << RTT::endlog();
    }

    rclcpp::QoS qos(static_cast<std::size_t>(std::max(1, policy.size)));
    if (policy.init) {qos.transient_local();}
    publisher_ = node_->create_publisher<T>(topic, qos);

    node_->get_node_waitables_interface()->add_waitable(waitable_, nullptr);
  }

  virtual ~RosPublisherChannelElement() noexcept
  {
    node_->get_node_waitables_interface()->remove_waitable(waitable_, nullptr);
    waitable_->cancel();
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
    return "RosPublisherChannelElement";
  }

  std::string getRemoteURI() const override
  {
    return publisher_->get_topic_name();
  }

  RTT::WriteStatus write(typename RTT::base::ChannelElement<T>::param_t sample) override
  {
    publisher_->publish(sample);
    return RTT::WriteSuccess;
  }

  bool signal() override
  {
    waitable_->trigger();
    return true;
  }

  bool publish()
  {
#if rclcpp_VERSION_GTE(0, 8, 1)
    if (publisher_->can_loan_messages()) {
      return publish_impl_loaned_message();
    } else {
      return publish_impl();
    }
#else
    return publish_impl();
#endif
  }

protected:
  bool publish_impl()
  {
    while (true) {
      // MessageInitialization is an alias for either
      // rosidl_generator_cpp::MessageInitialization or rosidl_runtime_cpp::MessageInitialization
      // depending on the ROS version.
#ifdef ROSIDL_RUNTIME_CPP__MESSAGE_INITIALIZATION_HPP_
      using rosidl_runtime_cpp::MessageInitialization;
#else
      using rosidl_generator_cpp::MessageInitialization;
#endif
      T message{MessageInitialization::SKIP};
      static constexpr bool kCopyOldData = false;
      if (this->read(message, kCopyOldData) != RTT::NewData) {
        // No more work for now.
        return true;
      }
      publisher_->publish(std::move(message));
    }
  }

#if rclcpp_VERSION_GTE(0, 8, 1)
  bool publish_impl_loaned_message()
  {
    while (true) {
      auto message = publisher_->borrow_loaned_message();
      if (!message.is_valid()) {
        // Eventually there is more work...
        return false;
      }
      static constexpr bool kCopyOldData = false;
      if (this->read(message.get(), kCopyOldData) != RTT::NewData) {
        // No more work for now.
        return true;
      }
      publisher_->publish(std::move(message));
    }
  }
#endif

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Publisher<T>::SharedPtr publisher_;
  Waitable::SharedPtr waitable_;
};

}  // namespace rtt_ros2_topics

#endif  // OROCOS__RTT_ROS2_TOPICS__ROS_PUBLISHER_HPP_
