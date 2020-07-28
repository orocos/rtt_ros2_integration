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

#ifndef OROCOS__RTT_ROS2_SERVICES__RTT_ROS2_SERVICES_REGISTRY_HPP_
#define OROCOS__RTT_ROS2_SERVICES__RTT_ROS2_SERVICES_REGISTRY_HPP_

#include <memory>
#include <map>
#include <string>

#include "rtt/os/Mutex.hpp"

namespace rtt_ros2_services
{

class RosServiceRegistry;
using RosServiceRegistryPtr = std::shared_ptr<RosServiceRegistry>;

class RosServiceProxyFactoryBase;
using RosServiceProxyFactoryBasePtr = std::shared_ptr<RosServiceProxyFactoryBase>;

class RosServiceRegistry
{
public:
  static RosServiceRegistryPtr Instance();
  static void Release();

  /** \brief Register a ROS service proxy factory
   *
   * This enables the RosServiceRegistryService to construct ROS service clients and
   * servers from a string name.
   */
  bool registerServiceFactory(RosServiceProxyFactoryBasePtr factory);

  bool hasServiceFactory(const std::string & service_type);

  RosServiceProxyFactoryBasePtr getServiceFactory(const std::string & service_type);

  void listSrvs();

private:
  //! ROS service proxy factories
  std::map<std::string, RosServiceProxyFactoryBasePtr> factories_;
  RTT::os::MutexRecursive factory_lock_;

  //! The singleton instance
  static RosServiceRegistryPtr s_instance_;
};

}  // namespace rtt_ros2_services

#endif  // OROCOS__RTT_ROS2_SERVICES__RTT_ROS2_SERVICES_REGISTRY_HPP_
