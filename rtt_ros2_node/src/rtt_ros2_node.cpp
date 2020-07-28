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

#include "rtt_ros2_node/rtt_ros2_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/executors.hpp"

#include "rtt/TaskContext.hpp"
#include "rtt/internal/GlobalService.hpp"

namespace rtt_ros2_node
{

namespace
{

std::string default_node_name_from_owner(RTT::TaskContext * owner)
{
  if (owner != nullptr) {
    return owner->getName();
  } else {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return "rtt_" + std::to_string(std::chrono::nanoseconds(now).count());
  }
}

}  // namespace

Node::Node(
  RTT::TaskContext * owner)
: Node(default_node_name_from_owner(owner), {},
    rclcpp::NodeOptions().allow_undeclared_parameters(true), owner)
{
}

Node::Node(
  const rclcpp::NodeOptions & options,
  RTT::TaskContext * owner)
: Node(default_node_name_from_owner(owner), {}, options, owner)
{
}

Node::Node(
  const std::string & node_name,
  const rclcpp::NodeOptions & options,
  RTT::TaskContext * owner)
: Node(node_name, {}, options, owner)
{
}

Node::Node(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options,
  RTT::TaskContext * owner)
: RTT::Service("rosnode", owner)
{
  node_ = std::make_shared<rclcpp::Node>(
    !node_name.empty() ? node_name : default_node_name_from_owner(owner),
    namespace_, options);

  this->addOperation("spin", &Node::spin, this)
  .doc("Start a single or multi-threaded spinner for this node")
  .arg("number_of_threads", "The number of spinner threads (0 = hardware concurrency)");
  this->addOperation("cancel", &Node::cancel, this)
  .doc("Cancel all operations and stop the spinner threads for this node");

  // eventually start a spinner
  const auto number_of_threads =
    node_->declare_parameter<int>("spinner_threads", 1);  // 0 = hardware concurrency
  if (number_of_threads >= 0) {
    spin(static_cast<unsigned int>(number_of_threads));
  }
}

Node::~Node()
{
  cancel();
}

void Node::spin(unsigned int number_of_threads)
{
  cancel();
  const auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::executor::ExecutorArgs(),
    number_of_threads
  );
  executor->add_node(node_);
  thread_ = std::thread([ = ]() {executor->spin();});

  const auto actual_number_of_threads = executor->get_number_of_threads();
  RTT::log(RTT::Info) <<
    "ROS node spinner started for node " << node_->get_fully_qualified_name() <<
    " (" << actual_number_of_threads << " " <<
  (actual_number_of_threads > 1 ? "threads" : "thread") << ")." <<
    RTT::endlog();

  executor_ = std::move(executor);
}

void Node::cancel()
{
  const auto executor = std::move(executor_);
  if (executor != nullptr) {
    RTT::log(RTT::Info) <<
      "Shutting down ROS node spinner for node " << node_->get_fully_qualified_name() <<
      "..." << RTT::endlog();
    executor->cancel();
  }
  if (thread_.joinable()) {
    thread_.join();
    thread_ = std::thread();
  }
}

Node::shared_ptr getNodeService(RTT::TaskContext * tc)
{
  Node::shared_ptr node;

  if (tc != nullptr) {
    // Try to find the service by name
    node = boost::dynamic_pointer_cast<Node>(tc->provides()->getService("Node"));
    if (node != nullptr) {
      return node;
    }

    // Try to find the service by type
    for (const auto & name : tc->provides()->getProviderNames()) {
      node = boost::dynamic_pointer_cast<Node>(tc->provides()->getService(name));
      if (node != nullptr) {
        return node;
      }
    }
  }

  // Try global service ros.Node
  RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->getService("ros");
  if (ros != nullptr) {
    node = boost::dynamic_pointer_cast<Node>(ros->getService("rosnode"));
    if (node != nullptr) {
      return node;
    }
  }

  return nullptr;
}

rclcpp::Node::SharedPtr getNode(RTT::TaskContext * tc)
{
  Node::shared_ptr node = getNodeService(tc);
  if (node == nullptr) {
    return nullptr;
  }
  return node->node();
}

}  // namespace rtt_ros2_node
