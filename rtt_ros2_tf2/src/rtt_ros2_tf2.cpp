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

#include "rtt_ros2_tf2/rtt_ros2_tf2.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rtt/TaskContext.hpp"
#include "rtt_ros2_node/rtt_ros2_node.hpp"

#include "rtt/internal/GlobalService.hpp"

// The main interface happens through tf2::BufferCore
// http://docs.ros2.org/foxy/api/tf2/
// http://docs.ros2.org/foxy/api/tf2/classtf2_1_1BufferCore.html
namespace rtt_ros2_tf2
{

Tf2Service::Tf2Service(RTT::TaskContext * owner)
: RTT::Service("tf2", owner),
  ip_stamped_transform_("ip_stamped_transform"),
  ip_stamped_transform_static_("ip_stamped_transform_static"),
  // ip_tf_port_("pi_tf_port"),
  rosnode(nullptr),
// clock_(boost::make_shared<rclcpp::Clock>(rcl_clock_type_t::RCL_SYSTEM_TIME)),
  buffer_core_(boost::make_shared<tf2::BufferCore>()),
  // buffer_client_(nullptr),
  transform_broadcaster_(nullptr),
  transform_listener_(nullptr),
  static_transform_broadcaster_(nullptr)
{
  RTT::Logger::In in(getName());
  clock_ = boost::make_shared<rclcpp::Clock>(rcl_clock_type_t::RCL_SYSTEM_TIME);
  // Ports are only added if a TaskContext is provided (not in GlobalService)
  if (owner != nullptr) {
    // Add the interface to the own service
    addTf2Interface(this->provides());
    RTT::log(RTT::Info) <<
      "TF2 service instantiated in: " << owner->getName() <<
      RTT::endlog();
    // Add operations
    this->addOperation(
      "broadcastTransform",
      &Tf2Service::broadcastTransform, this, RTT::OwnThread)
    .doc("Broadcasts a stamped transform as TF2");
    this->addOperation(
      "broadcastTransforms",
      &Tf2Service::broadcastTransforms, this, RTT::OwnThread)
    .doc("Broadcasts a vector of stamped transforms as TF2");
    this->addOperation(
      "broadcastStaticTransform",
      &Tf2Service::broadcastStaticTransform, this, RTT::OwnThread)
    .doc("Broadcasts a stamped transform as TF2 static");
    this->addOperation(
      "broadcastStaticTransforms",
      &Tf2Service::broadcastStaticTransforms, this, RTT::OwnThread)
    .doc("Broadcasts a vector of stamped transforms as TF2 static");
    this->addOperation(
      "lookupTransform",
      &Tf2Service::lookupTransform, this, RTT::OwnThread)
    .doc("Looks up for a TF2 transform");
    this->addOperation(
      "clear",
      &Tf2Service::clear, this, RTT::OwnThread)
    .doc("Clears TF2 transforms");
  } else {
    // addTf2Interface(
    //     RTT::internal::GlobalService::Instance()->provides("tf2"));
    // Add operations
    this->addOperation(
      "broadcastTransform",
      &Tf2Service::broadcastTransform, this, RTT::ClientThread)
    .doc("Broadcasts a stamped transform as TF2");
    this->addOperation(
      "broadcastTransforms",
      &Tf2Service::broadcastTransforms, this, RTT::ClientThread)
    .doc("Broadcasts a vector of stamped transforms as TF2");
    this->addOperation(
      "broadcastStaticTransform",
      &Tf2Service::broadcastStaticTransform, this, RTT::ClientThread)
    .doc("Broadcasts a stamped transform as TF2 static");
    this->addOperation(
      "broadcastStaticTransforms",
      &Tf2Service::broadcastStaticTransforms, this, RTT::ClientThread)
    .doc("Broadcasts a vector of stamped transforms as TF2 static");
    this->addOperation(
      "lookupTransform",
      &Tf2Service::lookupTransform, this, RTT::ClientThread)
    .doc("Looks up for a TF2 transform");
    this->addOperation(
      "clear",
      &Tf2Service::clear, this, RTT::ClientThread)
    .doc("Clears TF2 transforms");
    RTT::log(RTT::Info) <<
      "TF2 service instantiated standalone without interface" <<
      RTT::endlog();
  }
}

Tf2Service::~Tf2Service() = default;

// void Tf2Service::internalUpdate(
//     tf2_msgs::TFMessage & msg,
//     RTT::InputPort<tf2_msgs::TFMessage> & port,
//     bool is_static) {

// }

// rclcpp::Time Tf2Service::getLatestCommonTime(
//     const std::string & target,
//     const std::string & source) const {
//   return clock_->now();
// }

void Tf2Service::clear()
{
  RTT::Logger::In in(getName());
  if (buffer_core_) {
    buffer_core_->clear();
  }
}

bool Tf2Service::canTransform(
  const std::string & target,
  const std::string & source) const
{
  std::string ret_error;
  // const tf2::TimePoint time_point_now = tf2::TimePoint(clock_->now());
  try {
    if (!buffer_core_->canTransform(
        target,
        source, tf2::TimePointZero,
        &ret_error))
    {
      RTT::log(RTT::Warning) << "Cannot transform from " <<
        source << " to " << target << RTT::endlog();
      return false;
    }
  } catch (std::exception e) {
    RTT::log(RTT::Error) << "canTransform() produced an exception: " <<
      e.what() << RTT::endlog();
    return false;
  }
  return true;
}

geometry_msgs::msg::TransformStamped Tf2Service::lookupTransform(
  const std::string & target,
  const std::string & source) const
{
  try {
    return buffer_core_->lookupTransform(target, source, tf2::TimePointZero);
  } catch (std::exception e) {
    RTT::log(RTT::Error) << "lookupTransform() produced an exception: " <<
      e.what() << RTT::endlog();
    return geometry_msgs::msg::TransformStamped();
  }
}

// geometry_msgs::msg::TransformStamped Tf2Service::lookupTransformAtTime(
//     const std::string & target,
//     const std::string & source,
//     const rclcpp::Time & common_time) const {

// }

void Tf2Service::broadcastTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  RTT::Logger::In in(getName());
  if (!rosReady()) {
    RTT::log(RTT::Error) << "Ros Node is not ready" << RTT::endlog();
    return;
  }
  // tf2_msgs::msg::TFMessage converted_tf2_msg;
  try {
    std::string authority_name;
    if (nullptr != getOwner()) {
      authority_name = getOwner()->getName();
    } else {
      authority_name = getName();
    }
    // RTT::log(RTT::Debug) << "Calling broadcast a transform and setting "
    //   "authority to: " << authority_name << " in the next line "
    //   "with pos.x = " << transform.transform.translation.x << RTT::endlog();
    buffer_core_->setTransform(transform, authority_name, false);
    transform_broadcaster_->sendTransform(transform);
  } catch (tf2::LookupException e) {
    RTT::log(RTT::Error) << "Error when setting transform: " <<
      e.what() << RTT::endlog();
  }
}

void Tf2Service::broadcastTransforms(
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  if (!rosReady()) {
    return;
  }
  // for(const auto & transform : transforms) {
  //   broadcastTransform(transform);
  // }
  try {
    std::string authority_name;
    if (nullptr != getOwner()) {
      authority_name = getOwner()->getName();
    } else {
      authority_name = getName();
    }
    for (const auto & transform : transforms) {
      buffer_core_->setTransform(transform, authority_name, false);
    }
    // buffer_core_->setTransform(transform, "unknown_authority", false);
    transform_broadcaster_->sendTransform(transforms);
  } catch (std::exception e) {
    RTT::log(RTT::Error) << "Error when setting transform: " <<
      e.what() << RTT::endlog();
  }
}

void Tf2Service::broadcastStaticTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  if (!rosReady()) {
    return;
  }
  try {
    std::string authority_name;
    if (nullptr != getOwner()) {
      authority_name = getOwner()->getName();
    } else {
      authority_name = getName();
    }
    buffer_core_->setTransform(transform, authority_name, true);
    static_transform_broadcaster_->sendTransform(transform);
  } catch (tf2::LookupException e) {
    RTT::log(RTT::Error) << "Error when setting static transform: " <<
      e.what() << RTT::endlog();
  }
}

void Tf2Service::broadcastStaticTransforms(
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  if (!rosReady()) {
    return;
  }
  // for(const auto & transform : transforms) {
  //   broadcastStaticTransform(transform);
  // }
  try {
    std::string authority_name;
    if (nullptr != getOwner()) {
      authority_name = getOwner()->getName();
    } else {
      authority_name = getName();
    }
    for (const auto & transform : transforms) {
      buffer_core_->setTransform(transform, authority_name, true);
    }
    static_transform_broadcaster_->sendTransform(transforms);
  } catch (std::exception e) {
    RTT::log(RTT::Error) << "Error when setting transform: " <<
      e.what() << RTT::endlog();
  }
}

void Tf2Service::addTf2Interface(RTT::Service::shared_ptr service)
{
  // Ports cannot be added to the GlobalService
  // Add ports
  service->addEventPort(
    ip_stamped_transform_,
    boost::bind(&Tf2Service::stamped_message_callback, this, _1))
  .doc("Reception of stamped transform to broadcast through TF2");
  service->addEventPort(
    ip_stamped_transform_static_,
    boost::bind(&Tf2Service::stamped_message_static_callback, this, _1))
  .doc("Reception of stamped transform to broadcast through TF2 static");
}

void Tf2Service::stamped_message_callback(RTT::base::PortInterface * /*in_port*/)
{
  RTT::Logger::In in(getName());
  RTT::log(RTT::Warning) << "Callback called" << RTT::endlog();
  geometry_msgs::msg::TransformStamped in_msg;
  if (ip_stamped_transform_.read(in_msg) != RTT::NewData) {
    RTT::log(RTT::Warning) << "Received an event without new data" <<
      RTT::endlog();
  } else {
    broadcastTransform(in_msg);
  }
}

void Tf2Service::stamped_message_static_callback(
  RTT::base::PortInterface * /*in_port*/)
{
  RTT::Logger::In in(getName());
  RTT::log(RTT::Warning) << "Callback called" << RTT::endlog();
  geometry_msgs::msg::TransformStamped in_msg;
  if (ip_stamped_transform_static_.read(in_msg) != RTT::NewData) {
    RTT::log(RTT::Warning) << "Received an event without new data" <<
      RTT::endlog();
  } else {
    broadcastStaticTransform(in_msg);
  }
}

bool Tf2Service::rosReady()
{
  if (!rosnode) {
    rosnode = rtt_ros2_node::getNode(getOwner());
  }
  if (!rosnode) {
    RTT::log(RTT::Error) <<
      "No ROS node service from package rtt_ros2_node loaded into this "
      "component or as a global service." <<
      RTT::endlog();
    return false;
  }
  RTT::log(RTT::Info) << "We have node: " << rosnode->get_name() << RTT::endlog();
  if ((nullptr != transform_broadcaster_) &&
    (nullptr != transform_listener_) &&
    (nullptr != static_transform_broadcaster_))
  {
    return true;
  } else {
    // Initialize the interfaces to ros
    // buffer_core_ = boost::make_shared<tf2::BufferCore>();
    transform_broadcaster_ = boost::make_shared<tf2_ros::TransformBroadcaster>(
      rosnode);
    if (nullptr == transform_broadcaster_) {
      RTT::log(RTT::Error) << "Could not initialize broadcaster" <<
        RTT::endlog();
    }
    // ToDo: add parameter to decide spin_thread = spin_thread_ in nex call
    if (nullptr == buffer_core_) {
      RTT::log(RTT::Error) << "Buffer core is NULLPTR" <<
        RTT::endlog();
    }
    transform_listener_ = boost::make_shared<tf2_ros::TransformListener>(
      *buffer_core_, true);
    if (nullptr == transform_listener_) {
      RTT::log(RTT::Error) << "Could not initialize listener" <<
        RTT::endlog();
    }
    static_transform_broadcaster_ = boost::make_shared<
      tf2_ros::StaticTransformBroadcaster>(
      rosnode);
    if (nullptr == static_transform_broadcaster_) {
      RTT::log(RTT::Error) << "Could not initialize static broadcaster" <<
        RTT::endlog();
    }
    if ((nullptr == transform_broadcaster_) ||
      (nullptr == transform_listener_) ||
      (nullptr == static_transform_broadcaster_))
    {
      transform_broadcaster_.reset();
      transform_listener_.reset();
      static_transform_broadcaster_.reset();
      RTT::log(RTT::Error) << "Some transform object could not get "
        "initialized" << RTT::endlog();
      return false;
    }
  }
  return true;
}

}  // namespace rtt_ros2_tf2
