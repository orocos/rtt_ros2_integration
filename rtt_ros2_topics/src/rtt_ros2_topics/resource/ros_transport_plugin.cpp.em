@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore 
}@
// generated from rtt_ros2_topics/src/rtt_ros2_topics/resource/ros_transport_plugin.cpp.em
// generated code does not contain a copyright notice

#include "rtt/types/TransportPlugin.hpp"
#include "rtt/types/TypekitPlugin.hpp"

#include "rtt_ros2_topics/protocol_id.h"
#include "rtt_ros2_topics/ros_type_transporter.hpp"

@[for message in messages]@
#include "@(pkg_name)/msg/@(convert_camel_case_to_lower_case_underscore(message)).hpp"
@[end for]@

namespace rtt_ros2_@(pkg_name)
{

class RosTransportPlugin : public RTT::types::TransportPlugin
{
 public:
  bool registerTransport(std::string, RTT::types::TypeInfo * ti)
  {
@[for msg_name in messages]@
    if (ti->getTypeId() == &typeid(@(pkg_name)::msg::@(msg_name))) {
      return ti->addProtocol(
        ORO_ROS2_PROTOCOL_ID,
        new rtt_ros2_topics::RosTypeTransporter<@(pkg_name)::msg::@(msg_name)>());
    }
@[end for]@
    return false;
  }

  std::string getTransportName() const
  {
    return "ros2";
  }

  std::string getTypekitName() const
  {
    return "ros2-@(pkg_name)";
  }

  std::string getName() const
  {
    return "ros2-@(pkg_name)-transport";
  }
};

}  // namespace rtt_ros2_@(pkg_name)

ORO_TYPEKIT_PLUGIN(rtt_ros2_@(pkg_name)::RosTransportPlugin)
