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


#include <utility>

#include "rtt_ros2_params/rtt_ros2_params.hpp"

#include "rtt/internal/GlobalService.hpp"
#include "rtt/plugin/ServicePlugin.hpp"

#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

namespace rtt_ros2_params {

bool has_node() {
  return true;
}

static void loadGlobalROSService()
{
  if (!RTT::internal::GlobalService::Instance()->hasService("ros")) {
    RTT::log(RTT::Error) << "ROS2 node needs to be loaded before loading ROS2 params" <<
      RTT::endlog();
      // std::cout << "Name of owner of global service is: " << RTT::internal::GlobalService::Instance()->getOwner()->getName() << std::endl;
      // const auto deployer_component = RTT::internal::GlobalService::Instance()->getOwner()->getPeer("Deployer");
      // deployer_component->import("rtt_ros2_node");
    // return;
  }

  RTT::Service::shared_ptr params =
    RTT::internal::GlobalService::Instance()->provides("params");
  params->doc("ROS2 params operations and services");

  // Call rclcpp::init()
//   rclcpp::InitOptions init_options;
//   init_options.shutdown_on_sigint = false;
  RTT::log(RTT::Info) <<
    "Initializing interface to ROS2 params with" << //: \"" << __os_main_argc() << "\" command-line arguments." <<
    RTT::endlog();
}

static bool loadROSServiceIntoTaskContext(RTT::TaskContext * tc)
{
  if (tc->provides()->hasService("Params")) {
    RTT::log(RTT::Error) <<
      "Another ROS params interface was already instantiated for component " <<
      tc->getName() << "." <<
      RTT::endlog();

    return false;
  }

  const auto params = boost::make_shared<Params>(tc);

  // params->addOperation("check_ros2_node", &Params::check_ros2_node, params, RTT::ClientThread)
  //   .doc("Checks wheter the component has a ROS2 service attached to it");


// ros->addOperation("create_named_node_with_namespace", &create_named_node_with_namespace)
//   .doc(
//     "Creates a new process-wide ROS node (with explicit namespace).")
//   .arg("node_name", "Name of the node")
//   .arg("namespace", "Namespace of the node");


  tc->provides()->addService(std::move(params));
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
std::string getRTTPluginName() {return "ros2_params";}
std::string getRTTTargetName() {return OROCOS_TARGET_NAME;}

} // extern "C"

} // namespace rtt_ros2_params
