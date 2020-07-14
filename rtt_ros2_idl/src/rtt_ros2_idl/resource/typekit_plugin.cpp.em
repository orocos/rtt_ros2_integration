// generated from rtt_ros2_idl/src/rtt_ros2_idl/resource/typekit_plugin.cpp.em
// generated code does not contain a copyright notice

#include "rtt/types/TypekitPlugin.hpp"

#include "rtt_ros2_idl/RosIntrospectionTypeInfo.hpp"
#include "rtt/types/PrimitiveSequenceTypeInfo.hpp"
#include "rtt/types/CArrayTypeInfo.hpp"

#include "Types.hpp"

// extern template declarations for TypeInfo generators
@[for msg_name in messages]@
extern template class rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::msg::@(msg_name)>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<@(pkg_name)::msg::@(msg_name)>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<@(pkg_name)::msg::@(msg_name)>>;
@[end for]@
@[for srv_name in services]@
extern template class rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::srv::@(srv_name)::Request>;
extern template class rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::srv::@(srv_name)::Response>;
@[end for]@
@[for action_name in actions]@
extern template class rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::action::@(action_name)::Goal>;
extern template class rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::action::@(action_name)::Feedback>;
extern template class rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::action::@(action_name)::Result>;
@[end for]@

namespace rtt_ros2_@(pkg_name) {

class TypekitPlugin : public RTT::types::TypekitPlugin
{
public:
  bool loadTypes() override
  {
    // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages.
@[for msg_name in messages]@
    RTT::types::Types()->addType(new rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::msg::@(msg_name)>("/@(pkg_name)/msg/@(msg_name)"));
    RTT::types::Types()->addType(new RTT::types::PrimitiveSequenceTypeInfo<std::vector<@(pkg_name)::msg::@(msg_name)> >("/@(pkg_name)/msg/@(msg_name)[]"));
    RTT::types::Types()->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<@(pkg_name)::msg::@(msg_name)> >("/@(pkg_name)/msg/@(msg_name)[c]"));
@[end for]@
@[for srv_name in services]@
    RTT::types::Types()->addType(new rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::srv::@(srv_name)::Request>("/@(pkg_name)/srv/@(srv_name)_Request"));
    RTT::types::Types()->addType(new rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::srv::@(srv_name)::Response>("/@(pkg_name)/srv/@(srv_name)_Response"));
@[end for]@
@[for action_name in actions]@
    RTT::types::Types()->addType(new rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::action::@(action_name)::Goal>("/@(pkg_name)/action/@(action_name)_Goal"));
    RTT::types::Types()->addType(new rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::action::@(action_name)::Feedback>("/@(pkg_name)/action/@(action_name)_Feedback"));
    RTT::types::Types()->addType(new rtt_ros2_idl::RosIntrospectionTypeInfo<@(pkg_name)::action::@(action_name)::Result>("/@(pkg_name)/action/@(action_name)_Result"));
@[end for]@
    return true;
  }

  bool loadOperators() override { return true; }
  bool loadConstructors() override { return true; }
  bool loadGlobals() { return true; }

  std::string getName() override
  {
    return "ros2-@(pkg_name)";
  }
};

}  // namespace rtt_ros2_@(pkg_name)

ORO_TYPEKIT_PLUGIN(rtt_ros2_@(pkg_name)::TypekitPlugin)
