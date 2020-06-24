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

#include "rtt_ros2_rclcpp_typekit/ros2_publisher_options_type.hpp"

#include <cassert>
#include <string>
#include <vector>

#include "rtt/internal/PartDataSource.hpp"
#include "rtt/types/MemberFactory.hpp"
#include "rtt/types/PrimitiveTypeInfo.hpp"

#include "rtt_ros2_rclcpp_typekit/rclcpp_version.h"
#include "rtt_ros2_rclcpp_typekit/wrapped_qos.hpp"

using namespace RTT;  // NOLINT(build/namespaces)
using namespace RTT::types;  // NOLINT(build/namespaces)

namespace rtt_ros2_rclcpp_typekit
{

PublisherOptionsBaseTypeInfo::PublisherOptionsBaseTypeInfo()
: PrimitiveTypeInfo<rclcpp::PublisherOptionsBase>("/rclcpp/PublisherOptionsBase")
{}

bool PublisherOptionsBaseTypeInfo::installTypeInfoObject(TypeInfo * ti)
{
  // aquire a shared reference to the this object
  boost::shared_ptr<PublisherOptionsBaseTypeInfo> mthis =
    boost::dynamic_pointer_cast<PublisherOptionsBaseTypeInfo>(this->getSharedPtr());
  assert(mthis);
  // Allow base to install first
  PrimitiveTypeInfo<rclcpp::PublisherOptionsBase>::installTypeInfoObject(ti);
  // Install the factories specific for this type
  ti->setMemberFactory(mthis);

  // Don't delete us, we're memory-managed.
  return false;
}

/**
 * Returns the list of struct member names of this type.
 * In case this type is not a struct, returns an empty list.
 */
std::vector<std::string> PublisherOptionsBaseTypeInfo::getMemberNames() const
{
  return {
    "use_intra_process_comm",
    "event_callbacks",
    "callback_group",
#if rclcpp_VERSION_GTE(0, 8, 1)
    "rmw_implementation_payload",
#endif
  };
}

base::DataSourceBase::shared_ptr PublisherOptionsBaseTypeInfo::getMember(
  base::DataSourceBase::shared_ptr item,
  const std::string & part_name) const
{
  typename internal::AssignableDataSource<rclcpp::PublisherOptionsBase>::shared_ptr adata =
    boost::dynamic_pointer_cast<internal::AssignableDataSource<rclcpp::PublisherOptionsBase>>(item);
  // Use a copy in case our parent is not assignable:
  if (!adata) {
    // is it non-assignable ?
    typename internal::DataSource<rclcpp::PublisherOptionsBase>::shared_ptr data =
      boost::dynamic_pointer_cast<internal::DataSource<rclcpp::PublisherOptionsBase>>(item);
    if (data) {
      // create a copy
      adata = new internal::ValueDataSource<rclcpp::PublisherOptionsBase>(data->get() );
    }
  }
  if (!adata) {
    log(Error) <<
      "Wrong call to type info function " + this->getTypeName() <<
      "'s getMember() can not process " << item->getTypeName() << endlog();
    return base::DataSourceBase::shared_ptr();
  }

  if (part_name == "use_intra_process_comm") {
    using FieldT = decltype(rclcpp::PublisherOptionsBase::use_intra_process_comm);
    using DataSourceT = internal::PartDataSource<FieldT>;
    return new DataSourceT(adata->set().use_intra_process_comm, item);
  }

  if (part_name == "event_callbacks") {
    using FieldT = decltype(rclcpp::PublisherOptionsBase::event_callbacks);
    using DataSourceT = internal::PartDataSource<FieldT>;
    return new DataSourceT(adata->set().event_callbacks, item);
  }

  if (part_name == "callback_group") {
    using FieldT = decltype(rclcpp::PublisherOptionsBase::callback_group);
    using DataSourceT = internal::PartDataSource<FieldT>;
    return new DataSourceT(adata->set().callback_group, item);
  }

#if rclcpp_VERSION_GTE(0, 8, 1)
  if (part_name == "rmw_implementation_payload") {
    using FieldT = decltype(rclcpp::PublisherOptionsBase::rmw_implementation_payload);
    using DataSourceT = internal::PartDataSource<FieldT>;
    return new DataSourceT(adata->set().rmw_implementation_payload, item);
  }
#endif

  return base::DataSourceBase::shared_ptr();
}

}  // namespace rtt_ros2_rclcpp_typekit
