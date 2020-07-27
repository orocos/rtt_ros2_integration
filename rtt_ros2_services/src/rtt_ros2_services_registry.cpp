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

#include <memory>
#include <string>
#include <utility>

#include "rtt_ros2_services/rtt_ros2_services_proxy.hpp"
#include "rtt_ros2_services/rtt_ros2_services_registry.hpp"

#include "rtt/Logger.hpp"
#include "rtt/os/MutexLock.hpp"

namespace rtt_ros2_services
{

RosServiceRegistryPtr RosServiceRegistry::s_instance_;

RosServiceRegistryPtr RosServiceRegistry::Instance()
{
  if (!s_instance_) {
    s_instance_ = std::make_shared<RosServiceRegistry>();
  }
  return s_instance_;
}

void RosServiceRegistry::Release()
{
  s_instance_.reset();
}

/** \brief Register a ROS service proxy factory
 *
 * This enables the RosServiceRegistry to construct ROS service clients and
 * servers from a string name.
 */
bool RosServiceRegistry::registerServiceFactory(RosServiceProxyFactoryBasePtr factory)
{
  RTT::os::MutexLock lock(factory_lock_);
  if (factory == nullptr) {
    RTT::log(RTT::Error) <<
      "Failed to register ROS service factory: nullptr given." <<
      RTT::endlog();
    return false;
  }

  const std::string & ros_service_type = factory->getType();
  factories_[ros_service_type] = std::move(factory);

  RTT::log(RTT::Info) <<
    "Successfully registered ROS service factory for \"" << ros_service_type << "\"." <<
    RTT::endlog();

  return true;
}

bool RosServiceRegistry::hasServiceFactory(const std::string & service_type)
{
  RTT::os::MutexLock lock(factory_lock_);
  return factories_.find(service_type) != factories_.end();
}

RosServiceProxyFactoryBasePtr RosServiceRegistry::getServiceFactory(
  const std::string & service_type)
{
  RTT::os::MutexLock lock(factory_lock_);
  if (factories_.find(service_type) != factories_.end()) {
    return factories_[service_type];
  }

  RTT::log(RTT::Error) <<
    "Service type \"" << service_type << "\" "
    "has not been registered with the rosservice_registry service." << RTT::endlog();
  return nullptr;
}

void RosServiceRegistry::listSrvs()
{
  RTT::os::MutexLock lock(factory_lock_);

  RTT::log(RTT::Info) << "Available ROS .srv types:" << RTT::endlog();
  for (const auto & factory : factories_) {
    RTT::log(RTT::Info) << " -- " << factory.first << RTT::endlog();
  }
}

}  // namespace rtt_ros2_services
