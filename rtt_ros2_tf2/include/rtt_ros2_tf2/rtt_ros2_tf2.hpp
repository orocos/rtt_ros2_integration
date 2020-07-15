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

#ifndef OROCOS__RTT_ROS2_TF2__RTT_ROS2_TF2_HPP_
#define OROCOS__RTT_ROS2_TF2__RTT_ROS2_TF2_HPP_

#include <map>
#include <string>

#include "rtt/Service.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2/buffer_core.h"
#include "geometry_msgs/typekit/msg/transform_stamped_Types.hpp"

namespace rtt_ros2_tf2
{

class RTT_TF2 : public RTT::Service, protected tf2::BufferCore
{


public:
  typedef boost::shared_ptr<RTT_TF2> shared_ptr;

  explicit RTT_TF2(RTT::TaskContext * owner);
  virtual ~RTT_TF2();

protected:
  ///! Operations

  rclcpp::Time getLatestCommonTime(
      const std::string & target,
      const std::string & source) const;

  bool canTransform(
      const std::string & target,
      const std::string & source) const;

  bool canTransformAtTime(
      const std::string & target,
      const std::string & source,
      const rclcpp::Time & common_time) const;

  // geometry_msgs::TransformStamped lookupTransform(
  //     const std::string & target,
  //     const std::string & source) const;

  // geometry_msgs::TransformStamped lookupTransformAtTime(
  //     const std::string & target,
  //     const std::string & source,
  //     const rclcpp::Time& common_time) const;

  void broadcastTransform(
      const geometry_msgs::msg::TransformStamped & tform);

  void broadcastTransforms(
      const std::vector<geometry_msgs::msg::TransformStamped> & tforms);

  void broadcastStaticTransform(
      const geometry_msgs::msg::TransformStamped & tform);

  void broadcastStaticTransforms(
      const std::vector<geometry_msgs::msg::TransformStamped> & tforms);

  void addTFOperations(RTT::Service::shared_ptr service);

  ///! Communication ports
  // RTT::InputPort<tf2_msgs::TFMessage> port_tf_in;
  // RTT::InputPort<tf2_msgs::TFMessage> port_tf_static_in;
  // RTT::OutputPort<tf2_msgs::TFMessage> port_tf_out;
  // RTT::OutputPort<tf2_msgs::TFMessage> port_tf_static_out;

private:
  // Constant
  static constexpr int kDefaultBufferSize = 100;

  // Clock
  boost::shared_ptr<rclcpp::Clock> clock_;

  // Example members

  int private_int_ = 42;
  // tf2::BufferCore buffer_core_;

  // Deprecated?
  std::string prop_tf_prefix_;
  double prop_cache_time_;
  double prop_buffer_size_;

private:
  // void internalUpdate(
  //   tf2_msgs::TFMessage & msg,
  //   RTT::InputPort<tf2_msgs::TFMessage> & port,
  //   bool is_static);

};  // class RTT_TF2

} // namespace rtt_ros2_tf2

#endif // OROCOS__RTT_ROS2_TF2__RTT_ROS2_TF2_HPP_
