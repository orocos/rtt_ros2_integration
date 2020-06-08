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

#ifndef OROCOS__RTT_ROS2_NODE__RTT_ROS2_NODE_HPP_
#define OROCOS__RTT_ROS2_NODE__RTT_ROS2_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "rtt/Service.hpp"

namespace rtt_ros2_node
{

/// Wrap a rclcpp::Node instance in an RTT::Service.
struct Node : public RTT::Service
{
public:
  explicit Node(RTT::TaskContext * owner = nullptr);
  Node(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    RTT::TaskContext * owner = nullptr);
  Node(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    RTT::TaskContext * owner = nullptr);
  Node(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    RTT::TaskContext * owner = nullptr);
  virtual ~Node();

  rclcpp::Node::SharedPtr node() {return node_;}
  rclcpp::executor::Executor::SharedPtr executor() {return executor_;}

  void spin(unsigned int number_of_threads = 1);
  void cancel();

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executor::Executor::SharedPtr executor_;
  std::thread thread_;
};

}  // namespace rtt_ros2_node

#endif  // OROCOS__RTT_ROS2_NODE__RTT_ROS2_NODE_HPP_
