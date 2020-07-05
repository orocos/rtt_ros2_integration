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

#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rtt/internal/GlobalService.hpp"
#include "rtt/plugin/ServicePlugin.hpp"
#include "rtt/os/startstop.h"

#include "rtt_ros2_node/rtt_ros2_node.hpp"

#include "boost/make_shared.hpp"

namespace rtt_ros2_node
{

static bool create_named_node_with_options(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options)
{
  RTT::Service::shared_ptr ros =
    RTT::internal::GlobalService::Instance()->provides("ros");

  if (ros->hasService("Node")) {
    RTT::log(RTT::Error) <<
      "Another process-wide ROS node was already instantiated." <<
      RTT::endlog();
    return false;
  }

  const auto node = boost::make_shared<Node>(node_name, namespace_, options);
  ros->addService(std::move(node));
  return true;
}

static bool create_named_node_with_namespace(
  const std::string & node_name,
  const std::string & namespace_)
{
  return create_named_node_with_options(node_name, namespace_, rclcpp::NodeOptions());
}

static bool create_named_node(
  const std::string & node_name)
{
  return create_named_node_with_options(node_name, {}, rclcpp::NodeOptions());
}

static bool create_node()
{
  return create_named_node_with_options({}, {}, rclcpp::NodeOptions());
}

static void loadGlobalROSService()
{
  RTT::Service::shared_ptr ros =
    RTT::internal::GlobalService::Instance()->provides("ros");
  ros->doc("ROS operations and services");

  // Call rclcpp::init()
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_sigint = false;
  RTT::log(RTT::Info) <<
    "Initializing ROS context with " << __os_main_argc() << " command-line arguments." <<
    RTT::endlog();
  rclcpp::init(__os_main_argc(), __os_main_argv(), init_options);

  ros->addOperation("create_node", &create_node)
  .doc(
    "Creates a new process-wide ROS node with an anonymous name.");

  ros->addOperation("create_named_node", &create_named_node)
  .doc(
    "Creates a new process-wide ROS node.")
  .arg("node_name", "Name of the node");

  ros->addOperation("create_named_node_with_namespace", &create_named_node_with_namespace)
  .doc(
    "Creates a new process-wide ROS node (with explicit namespace).")
  .arg("node_name", "Name of the node")
  .arg("namespace", "Namespace of the node");

  ros->addOperation("create_named_node_with_options", &create_named_node_with_options)
  .doc(
    "Creates a new process-wide ROS node (with NodeOptions).")
  .arg("node_name", "Name of the node")
  .arg("namespace", "Namespace of the node")
  .arg("options", "Additional options to control creation of the node.");
}

static bool loadROSServiceIntoTaskContext(RTT::TaskContext * tc)
{
  if (tc->provides()->hasService("Node")) {
    RTT::log(RTT::Error) <<
      "Another ROS node was already instantiated for component " <<
      tc->getName() << "." <<
      RTT::endlog();

    return false;
  }

  const auto node = boost::make_shared<Node>(tc);
  tc->provides()->addService(std::move(node));
  return true;
}

extern "C" {
bool loadRTTPlugin(RTT::TaskContext * tc)
{
  if (tc == nullptr) {
    loadGlobalROSService();
    return true;
  } else {
    return loadROSServiceIntoTaskContext(tc);
  }
}
std::string getRTTPluginName() {return "ros2-node";}
std::string getRTTTargetName() {return OROCOS_TARGET_NAME;}
}

}  // namespace rtt_ros2_node
