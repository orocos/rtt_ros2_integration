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

#include "rclcpp/rclcpp.hpp"
#include <rtt/TaskContext.hpp>

#include "rtt/internal/GlobalService.hpp"

// The main interface happens through tf2::BufferCore
// http://docs.ros2.org/foxy/api/tf2/
// http://docs.ros2.org/foxy/api/tf2/classtf2_1_1BufferCore.html
namespace rtt_ros2_tf2
{

RTT_TF2::RTT_TF2(RTT::TaskContext * owner)
: RTT::Service("TF2", owner),
  buffer_core_(boost::make_shared<tf2::BufferCore>()),
  ip_stamped_transform_("ip_stamped_transform"),
  ip_stamped_transform_static_("ip_stamped_transform_static"),
  ip_tf_port_("pi_tf_port"),
  clock_(boost::make_shared<rclcpp::Clock>(rcl_clock_type_t::RCL_SYSTEM_TIME))
{
  RTT::Logger::In in(getName());

  if (owner != nullptr) {
    // Add the interface to the own service
    addTf2Interface(this->provides("TF2"));
    RTT::log(RTT::Info) <<
      "TF2 service instantiated in: " << owner->getName() <<
      RTT::endlog();
  } else {
    RTT::log(RTT::Info) <<
      "TF2 service instantiated standalone without interface" <<
      RTT::endlog();
  }
}

RTT_TF2::~RTT_TF2() = default;

// void RTT_TF2::internalUpdate(
//     tf2_msgs::TFMessage & msg,
//     RTT::InputPort<tf2_msgs::TFMessage> & port,
//     bool is_static) {

// }

// rclcpp::Time RTT_TF2::getLatestCommonTime(
//     const std::string & target,
//     const std::string & source) const {
//   return clock_->now();
// }

void RTT_TF2::clear() {
  buffer_core_->clear();
}

bool RTT_TF2::canTransform(
    const std::string & target,
    const std::string & source) const {
  std::string ret_error;
  // const tf2::TimePoint time_point_now = tf2::TimePoint(clock_->now());
  try {
    if (!buffer_core_->canTransform(target,
        source, tf2::TimePoint(),
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

geometry_msgs::msg::TransformStamped RTT_TF2::lookupTransform(
    const std::string & target,
    const std::string & source) const {
  try {
    return buffer_core_->lookupTransform(target, source, tf2::TimePoint());
  } catch (std::exception e) {
    RTT::log(RTT::Error) << "lookupTransform() produced an exception: " <<
        e.what() << RTT::endlog();
    return geometry_msgs::msg::TransformStamped();
  }
}

// geometry_msgs::msg::TransformStamped RTT_TF2::lookupTransformAtTime(
//     const std::string & target,
//     const std::string & source,
//     const rclcpp::Time & common_time) const {

// }

void RTT_TF2::broadcastTransform(
    const geometry_msgs::msg::TransformStamped & transform) {
  RTT::Logger::In in(getName());
  // tf2_msgs::msg::TFMessage converted_tf2_msg;
  try {
    buffer_core_->setTransform(transform, "unknown_authority", false);
  } catch (tf2::LookupException e) {
    RTT::log(RTT::Error) << "Error when setting transform: " <<
      e.what() << RTT::endlog();
  }
}

void RTT_TF2::broadcastTransforms(
    const std::vector<geometry_msgs::msg::TransformStamped> & transforms) {
  for(const auto & transform : transforms) {
    broadcastTransform(transform);
  }
}

void RTT_TF2::broadcastStaticTransform(
    const geometry_msgs::msg::TransformStamped & transform) {
  try {
    buffer_core_->setTransform(transform, "unknown_authority", true);
  } catch (tf2::LookupException e) {
    RTT::log(RTT::Error) << "Error when setting static transform: " <<
      e.what() << RTT::endlog();
  }
}

void RTT_TF2::broadcastStaticTransforms(
    const std::vector<geometry_msgs::msg::TransformStamped> & transforms) {
  for(const auto & transform : transforms) {
    broadcastStaticTransform(transform);
  }
}

void RTT_TF2::addTf2Interface(RTT::Service::shared_ptr service) {
  // Add ports
  service->addEventPort(ip_stamped_transform_,
    boost::bind(&RTT_TF2::stamped_message_callback, this, _1))
    .doc("Reception of stamped transform to broadcast through TF2");
  service->addEventPort(ip_stamped_transform_static_,
    boost::bind(&RTT_TF2::stamped_message_static_callback, this, _1))
    .doc("Reception of stamped transform to broadcast through TF2 static");
  // Add operations
  service->addOperation("broadcastTransform",
    &RTT_TF2::broadcastTransform, this, RTT::OwnThread)
    .doc("Broadcasts a stamped transform as TF2");
  service->addOperation("broadcastTransforms",
    &RTT_TF2::broadcastTransforms, this, RTT::OwnThread)
    .doc("Broadcasts a vector of stamped transforms as TF2");
  service->addOperation("broadcastStaticTransform",
    &RTT_TF2::broadcastStaticTransform, this, RTT::OwnThread)
    .doc("Broadcasts a stamped transform as TF2 static");
  service->addOperation("broadcastStaticTransforms",
    &RTT_TF2::broadcastStaticTransforms, this, RTT::OwnThread)
    .doc("Broadcasts a vector of stamped transforms as TF2 static");
}

void RTT_TF2::stamped_message_callback(RTT::base::PortInterface * /*in_port*/) {
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

void RTT_TF2::stamped_message_static_callback(RTT::base::PortInterface * /*in_port*/) {
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

} // namespace rtt_ros2_tf2
