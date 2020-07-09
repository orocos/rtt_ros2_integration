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

#include "rtt/internal/GlobalService.hpp"
#include "rtt/plugin/ServicePlugin.hpp"

#include "boost/make_shared.hpp"

#include "rtt_ros2_params/rtt_ros2_params.hpp"

namespace rtt_ros2_params
{

bool has_node()
{
  return true;
}

static void loadGlobalROSService()
{
  if (!RTT::internal::GlobalService::Instance()->hasService("ros")) {
    RTT::internal::GlobalService::Instance()->provides("ros");
    if (!RTT::internal::GlobalService::Instance()->hasService("ros")) {
      RTT::log(RTT::Error) <<
        "ROS2 node needs to be loaded before loading ROS2 params" <<
        RTT::endlog();
      return;
    }
  }
  auto ros = RTT::internal::GlobalService::Instance()->provides("ros");
  RTT::Service::shared_ptr params =
    boost::make_shared<rtt_ros2_params::Params>(nullptr);
  params->doc("ROS2 params operations and services");

  if (!ros->addService(params)) {
    // No ROS, but we checked this earlier
    RTT::log(RTT::Error) << "The global ROS service could not load paramter support" <<
      RTT::endlog();
  }

  RTT::log(RTT::Info) <<
    "Initializing interface to ROS2 params" <<
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
std::string getRTTPluginName() {return "ros2-params";}
std::string getRTTTargetName() {return OROCOS_TARGET_NAME;}

}  // extern "C"

}  // namespace rtt_ros2_params
