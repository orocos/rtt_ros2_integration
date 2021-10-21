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

// #include "rtt/deployment/ComponentLoader.hpp"
#include "rtt/internal/GlobalService.hpp"
#include "rtt/plugin/ServicePlugin.hpp"

#include "boost/make_shared.hpp"

#include "rtt_ros2_tf2/rtt_ros2_tf2.hpp"


namespace rtt_ros2_tf2
{

static bool loadROSServiceIntoTaskContext(RTT::TaskContext * tc)
{
  if (tc->provides()->hasService("tf2")) {
    RTT::log(RTT::Error) <<
      "Another rosparam interface was already instantiated for component " <<
      tc->getName() << "." << RTT::endlog();
    return false;
  }

  auto tf_service = boost::make_shared<rtt_ros2_tf2::Tf2Service>(tc);
  // tf_service->addTf2Interface(tc->provides());
  tc->provides()->addService(std::move(tf_service));
  return true;
}

static bool loadGlobalROSService()
{
  RTT::Service::shared_ptr ros =
    RTT::internal::GlobalService::Instance()->provides("ros");
  ros->doc("ROS operations and services");
  rtt_ros2_tf2::Tf2Service::shared_ptr tf_service =
    boost::make_shared<rtt_ros2_tf2::Tf2Service>(nullptr);

  // tf_service->addTf2Interface(tf_service);
  if (!ros->addService(std::move(tf_service))) {
    // addService() can fail if a service of the same name already exists in ros
    RTT::log(RTT::Error) << "The global ROS service could not load tf2 "
      "support" << RTT::endlog();
    return false;
  }

  RTT::log(RTT::Info) <<
    "Initializing interface to ROS TF2" <<
    RTT::endlog();
  return true;
}

extern "C" {
bool loadRTTPlugin(RTT::TaskContext * tc)
{
  if (tc == nullptr) {
    return loadGlobalROSService();
  } else {
    return loadROSServiceIntoTaskContext(tc);
  }
}
std::string getRTTPluginName() {return "tf2";}
std::string getRTTTargetName() {return OROCOS_TARGET_NAME;}
}  // extern "C"

}  // namespace rtt_ros2_tf2
