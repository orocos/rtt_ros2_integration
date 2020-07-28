@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
}@
// generated from rtt_ros2_services/src/rtt_ros2_services/resource/ros_service_plugin.cpp.em
// generated code does not contain a copyright notice

#include "rtt/Logger.hpp"
#include "rtt/plugin/ServicePlugin.hpp"

#include "rtt_ros2_services/rtt_ros2_services_proxy.hpp"
#include "rtt_ros2_services/rtt_ros2_services_registry.hpp"

@[for service in services]@
#include "@(pkg_name)/srv/@(convert_camel_case_to_lower_case_underscore(service)).hpp"
@[end for]@
@[for extra_include in extra_includes]@
#include "@(extra_include)"
@[end for]@

namespace rtt_ros2_services
{

static bool registerRosServiceProxies()
{
  // Get the ros service registry service
  auto rosservice_registry = RosServiceRegistry::Instance();
  if (!rosservice_registry) {
    RTT::log(RTT::Error) <<
      "Could not get an instance of the RosServiceRegistry! Not registering service proxies "
      "for @(pkg_name)" << RTT::endlog();
    return false;
  }

  @[for service in services]@
  if (!rosservice_registry->registerServiceFactory(
      std::make_shared<RosServiceProxyFactory<@(pkg_name)::srv::@(service)>>(
        "@(pkg_name)/srv/@(service)")))
  {
    return false;
  }
  @[end for]@

  return true;
}

}  // namespace rtt_ros2_services

extern "C" {
bool loadRTTPlugin(RTT::TaskContext * c)
{
  if (c != nullptr) {return false;}
  return rtt_ros2_services::registerRosServiceProxies();
}
std::string getRTTPluginName()
{
  return "ros-@(pkg_name)-services";
}
std::string getRTTTargetName()
{
  return OROCOS_TARGET_NAME;
}
}
