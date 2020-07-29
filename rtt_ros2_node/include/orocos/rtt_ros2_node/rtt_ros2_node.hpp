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

/// Return the default rclcpp::NodeOptions to be used within RTT and rtt_ros2_integration.
rclcpp::NodeOptions getDefaultNodeOptions();

/// Wrap a rclcpp::Node instance in an RTT::Service.
struct Node : public RTT::Service
{
public:
  using shared_ptr = boost::shared_ptr<Node>;

  explicit Node(RTT::TaskContext * owner = nullptr);
  Node(
    const rclcpp::NodeOptions & options = getDefaultNodeOptions(),
    RTT::TaskContext * owner = nullptr);
  Node(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = getDefaultNodeOptions(),
    RTT::TaskContext * owner = nullptr);
  Node(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = getDefaultNodeOptions(),
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

/// Retrieve a pointer to the rtt_ros2_node::Node service to be used for the given TaskContext.
/**
 * @param tc The TaskContext instance for which to retrieve a Node pointer. If nullptr, consider
 *           only the global (process-wide) node.
 * @returns the Node instance loaded as an RTT service in the given TaskContext,
 * or falls back to the global (process-wide) Node loaded into the GlobalService.
 * If none of both is loaded, the function returns nullptr.
 */
Node::shared_ptr getNodeService(RTT::TaskContext * tc = nullptr);

/// Retrieve a rclcpp::Node::SharedPtr to be used for the given TaskContext.
/**
 * @sa \ref getNodeService()
 * @returns getNodeService(tc) ? getNodeService(tc)->node() : nullptr
 */
rclcpp::Node::SharedPtr getNode(RTT::TaskContext * tc = nullptr);

}  // namespace rtt_ros2_node

#endif  // OROCOS__RTT_ROS2_NODE__RTT_ROS2_NODE_HPP_
