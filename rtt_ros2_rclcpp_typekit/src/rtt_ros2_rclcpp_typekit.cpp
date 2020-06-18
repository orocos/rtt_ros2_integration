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
#include <vector>

#include "rtt/types/EnumTypeInfo.hpp"
#include "rtt/types/GlobalsRepository.hpp"
#include "rtt/types/SequenceTypeInfo.hpp"
#include "rtt/types/OperatorTypes.hpp"
#include "rtt/types/TypekitPlugin.hpp"

#include "rtt_ros2_rclcpp_typekit/ros2_duration_type.hpp"
#include "rtt_ros2_rclcpp_typekit/ros2_node_options_type.hpp"
#include "rtt_ros2_rclcpp_typekit/ros2_parameter_type.hpp"
#include "rtt_ros2_rclcpp_typekit/ros2_parameter_value_type.hpp"
#include "rtt_ros2_rclcpp_typekit/ros2_publisher_options_type.hpp"
#include "rtt_ros2_rclcpp_typekit/ros2_qos_type.hpp"
#include "rtt_ros2_rclcpp_typekit/ros2_time_type.hpp"
#include "rtt_ros2_rclcpp_typekit/time_conversions.hpp"
#include "rtt_ros2_rclcpp_typekit/time_io.hpp"
#include "rtt_ros2_rclcpp_typekit/time_operators.hpp"
#include "rtt_ros2_rclcpp_typekit/wrapped_duration.hpp"

using namespace RTT::types;  // NOLINT(build/namespaces)

namespace rtt_ros2_rclcpp_typekit
{

class TypekitPlugin : public RTT::types::TypekitPlugin
{
public:
  bool loadTypes() override
  {
    const auto types = Types();

    types->addType(new DurationTypeInfo());
    types->addType(new TimeTypeInfo());
    types->addType(new NodeOptionsTypeInfo());
    types->addType(new ParameterTypeInfo());
    types->addType(new SequenceTypeInfo<std::vector<rclcpp::Parameter>>("/rclcpp/Parameter[]"));
    types->addType(new ParameterValueTypeInfo());
    types->addType(new PublisherOptionsBaseTypeInfo());
    types->addType(new QoSTypeInfo());

    types->addType(new EnumTypeInfo<rclcpp::IntraProcessSetting>("/rclcpp/IntraProcessSetting"));
    types->addType(
      new PrimitiveTypeInfo<rclcpp::PublisherEventCallbacks>(
        "/rclcpp/PublisherEventCallbacks"));
    types->addType(
      new PrimitiveTypeInfo<
        std::shared_ptr<rclcpp::callback_group::CallbackGroup>>(
        "/rclcpp/CallbackGroup"));
    types->addType(
      new PrimitiveTypeInfo<
        std::shared_ptr<rclcpp::detail::RMWImplementationSpecificPublisherPayload>>(
        "/rclcpp/RMWImplementationSpecificPublisherPayload"));

    types->addType(new PrimitiveTypeInfo<rmw_time_t, true>("rmw_time_t"));
    types->addType(new PrimitiveTypeInfo<rcl_allocator_t>("rcl_allocator_t"));
    types->addType(new EnumTypeInfo<rcl_clock_type_t>("rcl_clock_type_t"));
    types->addType(new EnumTypeInfo<rmw_qos_history_policy_t>("rmw_qos_history_policy_t"));
    types->addType(new EnumTypeInfo<rmw_qos_reliability_policy_t>("rmw_qos_reliability_policy_t"));
    types->addType(new EnumTypeInfo<rmw_qos_durability_policy_t>("rmw_qos_durability_policy_t"));
    types->addType(new EnumTypeInfo<rmw_qos_liveliness_policy_t>("rmw_qos_liveliness_policy_t"));

    return true;
  }

  bool loadOperators() override
  {
    const auto operators = OperatorRepository::Instance();

    // Duration
    operators->add(newBinaryOperator("+", duration_plus_duration()));
    operators->add(newBinaryOperator("-", duration_minus_duration()));

    // Time +/- Duration
    operators->add(newBinaryOperator("+", time_plus_duration()));
    operators->add(newBinaryOperator("-", time_minus_duration()));

    // Time - Time
    operators->add(newBinaryOperator("-", time_minus_time()));

    return true;
  }

  bool loadConstructors() override
  {
    const auto types = Types();

    // rmw_time_t <-> double
    types->getTypeInfo<rmw_time_t>()->addConstructor(newConstructor(&double_to_rmw_time_t));
    types->getTypeInfo<double>()->addConstructor(newConstructor(&rmw_time_t_to_double));

    // rmw_time_t <-> uint64
    types->getTypeInfo<rmw_time_t>()->addConstructor(newConstructor(&uint64_to_rmw_time_t));
    types->getTypeInfo<uint64_t>()->addConstructor(newConstructor(&rmw_time_t_to_uint64));

    return true;
  }

  bool loadGlobals() override
  {
    const auto globals = GlobalsRepository::Instance();

    // rclcpp::IntraProcessSetting
    globals->addConstant<rclcpp::IntraProcessSetting>(
      "IntraProcessSettingEnable",
      rclcpp::IntraProcessSetting::Enable);
    globals->addConstant<rclcpp::IntraProcessSetting>(
      "IntraProcessSettingDisable",
      rclcpp::IntraProcessSetting::Disable);
    globals->addConstant<rclcpp::IntraProcessSetting>(
      "IntraProcessSettingNodeDefault",
      rclcpp::IntraProcessSetting::NodeDefault);

    // rcl_clock_type_t
    globals->addConstant<rcl_clock_type_t>("RCL_CLOCK_UNINITIALIZED", RCL_CLOCK_UNINITIALIZED);
    globals->addConstant<rcl_clock_type_t>("RCL_ROS_TIME", RCL_ROS_TIME);
    globals->addConstant<rcl_clock_type_t>("RCL_SYSTEM_TIME", RCL_SYSTEM_TIME);
    globals->addConstant<rcl_clock_type_t>("RCL_STEADY_TIME", RCL_STEADY_TIME);

    // rmw_qos_reliability_policy_t
    globals->addConstant<rmw_qos_reliability_policy_t>(
      "RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT",
      RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    globals->addConstant<rmw_qos_reliability_policy_t>(
      "RMW_QOS_POLICY_RELIABILITY_RELIABLE",
      RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    globals->addConstant<rmw_qos_reliability_policy_t>(
      "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT",
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    globals->addConstant<rmw_qos_reliability_policy_t>(
      "RMW_QOS_POLICY_RELIABILITY_UNKNOWN",
      RMW_QOS_POLICY_RELIABILITY_UNKNOWN);

    // rmw_qos_history_policy_t
    globals->addConstant<rmw_qos_history_policy_t>(
      "RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT",
      RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);
    globals->addConstant<rmw_qos_history_policy_t>(
      "RMW_QOS_POLICY_HISTORY_KEEP_LAST",
      RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    globals->addConstant<rmw_qos_history_policy_t>(
      "RMW_QOS_POLICY_HISTORY_KEEP_ALL",
      RMW_QOS_POLICY_HISTORY_KEEP_ALL);
    globals->addConstant<rmw_qos_history_policy_t>(
      "RMW_QOS_POLICY_HISTORY_UNKNOWN",
      RMW_QOS_POLICY_HISTORY_UNKNOWN);

    // rmw_qos_durability_policy_t
    globals->addConstant<rmw_qos_durability_policy_t>(
      "RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT",
      RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
    globals->addConstant<rmw_qos_durability_policy_t>(
      "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL",
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    globals->addConstant<rmw_qos_durability_policy_t>(
      "RMW_QOS_POLICY_DURABILITY_VOLATILE",
      RMW_QOS_POLICY_DURABILITY_VOLATILE);
    globals->addConstant<rmw_qos_durability_policy_t>(
      "RMW_QOS_POLICY_DURABILITY_UNKNOWN",
      RMW_QOS_POLICY_DURABILITY_UNKNOWN);

    // rmw_qos_liveliness_policy_t
    globals->addConstant<rmw_qos_liveliness_policy_t>(
      "RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT",
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
    globals->addConstant<rmw_qos_liveliness_policy_t>(
      "RMW_QOS_POLICY_LIVELINESS_AUTOMATIC",
      RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    globals->addConstant<rmw_qos_liveliness_policy_t>(
      "RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE",
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE);
    globals->addConstant<rmw_qos_liveliness_policy_t>(
      "RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC",
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    globals->addConstant<rmw_qos_liveliness_policy_t>(
      "RMW_QOS_POLICY_LIVELINESS_UNKNOWN",
      RMW_QOS_POLICY_LIVELINESS_UNKNOWN);

    return true;
  }

  std::string getName() override
  {
    static const std::string name = "ros2-rclcpp";
    return name;
  }
};

}  // namespace rtt_ros2_rclcpp_typekit

ORO_TYPEKIT_PLUGIN(rtt_ros2_rclcpp_typekit::TypekitPlugin)
