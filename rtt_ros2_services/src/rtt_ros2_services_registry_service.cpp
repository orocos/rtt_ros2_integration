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

#include "rtt/plugin/ServicePlugin.hpp"
#include "rtt/Service.hpp"

#include "rtt_ros2_services/rtt_ros2_services_registry.hpp"

namespace rtt_ros2_services
{

class RosServiceRegistryService : public RTT::Service
{
public:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  explicit RosServiceRegistryService(RTT::TaskContext * owner)
  : RTT::Service("rosservice_registry", owner), instance_(RosServiceRegistry::Instance())
  {
    this->doc("Global RTT Service for registering ROS service types.");
    this->addOperation(
      "registerServiceFactory", &RosServiceRegistry::registerServiceFactory, instance_.get(),
      RTT::ClientThread);
    this->addOperation(
      "hasServiceFactory", &RosServiceRegistry::hasServiceFactory, instance_.get(),
      RTT::ClientThread);
    this->addOperation(
      "getServiceFactory", &RosServiceRegistry::getServiceFactory, instance_.get(),
      RTT::ClientThread);
    this->addOperation(
      "listSrvs", &RosServiceRegistry::listSrvs, instance_.get(),
      RTT::ClientThread);
  }

private:
  //! The singleton instance
  RosServiceRegistryPtr instance_;
};

}  // namespace rtt_ros2_services

ORO_GLOBAL_SERVICE_NAMED_PLUGIN(rtt_ros2_services::RosServiceRegistryService, "rosservice_registry")
