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

#include "rtt_ros2_rclcpp_typekit/ros2_qos_type.hpp"

#include <cassert>
#include <string>
#include <vector>

#include "rtt/internal/PartDataSource.hpp"
#include "rtt/types/MemberFactory.hpp"
#include "rtt/types/PrimitiveTypeInfo.hpp"

using namespace RTT;  // NOLINT(build/namespaces)
using namespace RTT::types;  // NOLINT(build/namespaces)

namespace rtt_ros2_rclcpp_typekit
{

WrappedQoS::WrappedQoS()
: rclcpp::QoS(rclcpp::KeepLast(0)) {}

QoSTypeInfo::QoSTypeInfo()
: PrimitiveTypeInfo<WrappedQoS>("/rclcpp/QoS")
{}

bool QoSTypeInfo::installTypeInfoObject(TypeInfo * ti)
{
  // aquire a shared reference to the this object
  boost::shared_ptr<QoSTypeInfo> mthis = boost::dynamic_pointer_cast<QoSTypeInfo>(
    this->getSharedPtr());
  assert(mthis);
  // Allow base to install first
  PrimitiveTypeInfo<WrappedQoS>::installTypeInfoObject(ti);
  // Install the factories specific for this type
  ti->setMemberFactory(mthis);

  // Don't delete us, we're memory-managed.
  return false;
}

// copied from StructTypeInfo<T>
std::vector<std::string> QoSTypeInfo::getMemberNames() const
{
  return {
    "history",
    "depth",
    "reliability",
    "durability",
    "deadline",
    "lifespan",
    "liveliness",
    "liveliness_lease_duration",
    "avoid_ros_namespace_conventions",
  };
}

// copied from StructTypeInfo<T>
base::DataSourceBase::shared_ptr QoSTypeInfo::getMember(
  base::DataSourceBase::shared_ptr item, const std::string & name) const
{
  typename internal::AssignableDataSource<WrappedQoS>::shared_ptr adata =
    boost::dynamic_pointer_cast<internal::AssignableDataSource<WrappedQoS>>(item);
  // Use a copy in case our parent is not assignable:
  if (!adata) {
    // is it non-assignable ?
    typename internal::DataSource<WrappedQoS>::shared_ptr data =
      boost::dynamic_pointer_cast<internal::DataSource<WrappedQoS>>(item);
    if (data) {
      // create a copy
      adata = new internal::ValueDataSource<WrappedQoS>(data->get() );
    }
  }
  if (!adata) {
    log(Error) <<
      "Wrong call to type info function " + this->getTypeName() <<
      "'s getMember() can not process " << item->getTypeName() << endlog();
    return base::DataSourceBase::shared_ptr();
  }

  if (name == "history") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::history)>(
      adata->set().get_rmw_qos_profile().history, item);
  }
  if (name == "depth") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::depth)>(
      adata->set().get_rmw_qos_profile().depth, item);
  }
  if (name == "reliability") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::reliability)>(
      adata->set().get_rmw_qos_profile().reliability, item);
  }
  if (name == "durability") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::durability)>(
      adata->set().get_rmw_qos_profile().durability, item);
  }
  if (name == "deadline") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::deadline)>(
      adata->set().get_rmw_qos_profile().deadline, item);
  }
  if (name == "lifespan") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::lifespan)>(
      adata->set().get_rmw_qos_profile().lifespan, item);
  }
  if (name == "liveliness") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::liveliness)>(
      adata->set().get_rmw_qos_profile().liveliness, item);
  }
  if (name == "liveliness_lease_duration") {
    return new internal::PartDataSource<decltype(rmw_qos_profile_t::liveliness_lease_duration)>(
      adata->set().get_rmw_qos_profile().liveliness_lease_duration, item);
  }
  if (name == "avoid_ros_namespace_conventions") {
    return new internal::PartDataSource<
      decltype(rmw_qos_profile_t::avoid_ros_namespace_conventions)>(
      adata->set().get_rmw_qos_profile().avoid_ros_namespace_conventions, item);
  }

  return base::DataSourceBase::shared_ptr();
}

}  // namespace rtt_ros2_rclcpp_typekit
