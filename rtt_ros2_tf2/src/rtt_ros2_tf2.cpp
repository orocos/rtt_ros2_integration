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

#include "rtt/internal/GlobalService.hpp"

namespace rtt_ros2_tf2
{

RTT_TF2::RTT_TF2(RTT::TaskContext * owner)
: RTT::Service("TF2", owner),
  clock_(boost::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
{
  RTT::Logger::In in(getName());

  RTT::log(RTT::Info) <<
    "TF2 service instantiated! in " << getName() <<
    RTT::endlog();
}

RTT_TF2::~RTT_TF2() = default;

// void RTT_TF2::internalUpdate(
//     tf2_msgs::TFMessage & msg,
//     RTT::InputPort<tf2_msgs::TFMessage> & port,
//     bool is_static) {

// }

rclcpp::Time RTT_TF2::getLatestCommonTime(
    const std::string & target,
    const std::string & source) const {

  return clock_->now();
}

bool RTT_TF2::canTransform(
    const std::string & target,
    const std::string & source) const {

  return false;
}

bool RTT_TF2::canTransformAtTime(
    const std::string& target,
    const std::string& source,
    const rclcpp::Time& common_time) const {

  return false;
}

// geometry_msgs::TransformStamped RTT_TF2::lookupTransform(
//     const std::string & target,
//     const std::string & source) const {

// }

// geometry_msgs::TransformStamped RTT_TF2::lookupTransformAtTime(
//     const std::string & target,
//     const std::string & source,
//     const rclcpp::Time & common_time) const {

// }

void RTT_TF2::broadcastTransform(
    const geometry_msgs::msg::TransformStamped & tform) {

}

void RTT_TF2::broadcastTransforms(
    const std::vector<geometry_msgs::msg::TransformStamped> & tforms) {

}

void RTT_TF2::broadcastStaticTransform(
    const geometry_msgs::msg::TransformStamped & tform) {

}

void RTT_TF2::broadcastStaticTransforms(
    const std::vector<geometry_msgs::msg::TransformStamped> & tforms) {

}

void RTT_TF2::addTFOperations(RTT::Service::shared_ptr service) {

}

} // namespace rtt_ros2_tf2
