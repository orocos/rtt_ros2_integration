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

#include "rtt_ros2_rclcpp_typekit/ros2_node_options_type.hpp"

#include <cassert>
#include <string>
#include <vector>

#include "rtt/types/MemberFactory.hpp"
#include "rtt/types/PrimitiveTypeInfo.hpp"

#include "rtt_ros2/getter_setter_datasource.hpp"

#include "rtt_ros2_rclcpp_typekit/wrapped_qos.hpp"

using namespace RTT;  // NOLINT(build/namespaces)
using namespace RTT::types;  // NOLINT(build/namespaces)

namespace rtt_ros2_rclcpp_typekit
{

NodeOptionsTypeInfo::NodeOptionsTypeInfo()
: PrimitiveTypeInfo<rclcpp::NodeOptions>("/rclcpp/NodeOptions")
{}

bool NodeOptionsTypeInfo::installTypeInfoObject(TypeInfo * ti)
{
  // aquire a shared reference to the this object
  boost::shared_ptr<NodeOptionsTypeInfo> mthis =
    boost::dynamic_pointer_cast<NodeOptionsTypeInfo>(this->getSharedPtr());
  assert(mthis);
  // Allow base to install first
  PrimitiveTypeInfo<rclcpp::NodeOptions>::installTypeInfoObject(ti);
  // Install the factories specific for this type
  ti->setMemberFactory(mthis);

  // Don't delete us, we're memory-managed.
  return false;
}

/**
 * Returns the list of struct member names of this type.
 * In case this type is not a struct, returns an empty list.
 */
std::vector<std::string> NodeOptionsTypeInfo::getMemberNames() const
{
  return {
    "context",
    "arguments",
    "parameter_overrides",
    "use_global_arguments",
    "use_intra_process_comms",
    "start_parameter_services",
    "start_parameter_event_publisher",
    "parameter_event_qos",
    "parameter_event_publisher_options",
    "allow_undeclared_parameters",
    "automatically_declare_parameters_from_overrides",
    "allocator"
  };
}

base::DataSourceBase::shared_ptr NodeOptionsTypeInfo::getMember(
  base::DataSourceBase::shared_ptr item,
  const std::string & part_name) const
{
  typename internal::AssignableDataSource<rclcpp::NodeOptions>::shared_ptr adata =
    boost::dynamic_pointer_cast<internal::AssignableDataSource<rclcpp::NodeOptions>>(item);
  // Use a copy in case our parent is not assignable:
  if (!adata) {
    // is it non-assignable ?
    typename internal::DataSource<rclcpp::NodeOptions>::shared_ptr data =
      boost::dynamic_pointer_cast<internal::DataSource<rclcpp::NodeOptions>>(item);
    if (data) {
      // create a copy
      adata = new internal::ValueDataSource<rclcpp::NodeOptions>(data->get() );
    }
  }
  if (!adata) {
    log(Error) <<
      "Wrong call to type info function " + this->getTypeName() <<
      "'s getMember() can not process " << item->getTypeName() << endlog();
    return base::DataSourceBase::shared_ptr();
  }

  if (part_name == "context") {
    using FieldT = rclcpp::Context::SharedPtr;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<FieldT (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::context);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::context);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "arguments") {
    using FieldT = std::vector<std::string>;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<const FieldT & (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::arguments);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(const FieldT &)>(
      &rclcpp::NodeOptions::arguments);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "parameter_overrides") {
    using FieldT = std::vector<rclcpp::Parameter>;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<const FieldT & (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::parameter_overrides);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(const FieldT &)>(
      &rclcpp::NodeOptions::parameter_overrides);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "use_global_arguments") {
    using FieldT = bool;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<FieldT (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::use_global_arguments);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::use_global_arguments);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "use_intra_process_comms") {
    using FieldT = bool;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<FieldT (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::use_intra_process_comms);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::use_intra_process_comms);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "start_parameter_services") {
    using FieldT = bool;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<FieldT (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::start_parameter_services);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::start_parameter_services);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "start_parameter_event_publisher") {
    using FieldT = bool;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<FieldT (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::start_parameter_event_publisher);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::start_parameter_event_publisher);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "parameter_event_qos") {
    using FieldT = WrappedQoS;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      [](const rclcpp::NodeOptions & node_options) -> WrappedQoS {
        return WrappedQoS(node_options.parameter_event_qos());
      };
    const auto setter =
      [](rclcpp::NodeOptions & node_options, const WrappedQoS & qos) -> rclcpp::NodeOptions & {
        return node_options.parameter_event_qos(qos);
      };
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "parameter_event_publisher_options") {
    using FieldT = rclcpp::PublisherOptionsBase;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<const FieldT & (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::parameter_event_publisher_options);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(const FieldT &)>(
      &rclcpp::NodeOptions::parameter_event_publisher_options);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "allow_undeclared_parameters") {
    using FieldT = bool;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<FieldT (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::allow_undeclared_parameters);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::allow_undeclared_parameters);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "automatically_declare_parameters_from_overrides") {
    using FieldT = bool;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<FieldT (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::automatically_declare_parameters_from_overrides);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::automatically_declare_parameters_from_overrides);
    return new DataSourceT(adata, getter, setter);
  }

  if (part_name == "allocator") {
    using FieldT = rcl_allocator_t;
    using DataSourceT = rtt_ros2::GetterSetterDataSource<rclcpp::NodeOptions, FieldT>;
    const auto getter =
      static_cast<const FieldT & (rclcpp::NodeOptions::*)() const>(
      &rclcpp::NodeOptions::allocator);
    const auto setter =
      static_cast<rclcpp::NodeOptions & (rclcpp::NodeOptions::*)(FieldT)>(
      &rclcpp::NodeOptions::allocator);
    return new DataSourceT(adata, getter, setter);
  }

  return base::DataSourceBase::shared_ptr();
}

}  // namespace rtt_ros2_rclcpp_typekit
