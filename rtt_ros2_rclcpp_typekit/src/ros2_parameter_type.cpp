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

#include "rtt_ros2_rclcpp_typekit/ros2_parameter_type.hpp"

#include <cassert>
#include <string>
#include <vector>

#include "rtt/internal/FusedFunctorDataSource.hpp"
#include "rtt/types/MemberFactory.hpp"
#include "rtt/types/PrimitiveTypeInfo.hpp"
#include "rtt/types/TemplateConstructor.hpp"

using namespace RTT;  // NOLINT(build/namespaces)
using namespace RTT::types;  // NOLINT(build/namespaces)

namespace rtt_ros2_rclcpp_typekit
{

namespace
{
inline std::string get_type_name(const rclcpp::Parameter & p)
{
  return p.get_type_name();
}

inline const std::string & get_name(const rclcpp::Parameter & p)
{
  return p.get_name();
}

inline const rclcpp::ParameterValue & get_parameter_value(const rclcpp::Parameter & p)
{
  return p.get_parameter_value();
}

inline rclcpp::Parameter construct1(const std::string & name)
{
  return rclcpp::Parameter(name);
}

inline rclcpp::Parameter construct2(const std::string & name, const rclcpp::ParameterValue & value)
{
  return rclcpp::Parameter(name, value);
}

}  // namespace

ParameterTypeInfo::ParameterTypeInfo()
: PrimitiveTypeInfo<rclcpp::Parameter>("/rclcpp/Parameter")
{}

bool ParameterTypeInfo::installTypeInfoObject(TypeInfo * ti)
{
  // aquire a shared reference to the this object
  boost::shared_ptr<ParameterTypeInfo> mthis = boost::dynamic_pointer_cast<ParameterTypeInfo>(
    this->getSharedPtr());
  assert(mthis);
  // Allow base to install first
  PrimitiveTypeInfo<rclcpp::Parameter>::installTypeInfoObject(ti);
  // Install the factories specific for this type
  ti->setMemberFactory(mthis);

  // Constructors
  ti->addConstructor(newConstructor(&construct1));
  ti->addConstructor(newConstructor(&construct2));

  // Don't delete us, we're memory-managed.
  return false;
}

// copied from StructTypeInfo<T>
std::vector<std::string> ParameterTypeInfo::getMemberNames() const
{
  return {
    "type_name",
    "name",
    "value",
  };
}

// copied from StructTypeInfo<T>
base::DataSourceBase::shared_ptr ParameterTypeInfo::getMember(
  base::DataSourceBase::shared_ptr item, const std::string & name) const
{
  typename internal::AssignableDataSource<rclcpp::Parameter>::shared_ptr adata =
    boost::dynamic_pointer_cast<internal::AssignableDataSource<rclcpp::Parameter>>(item);
  // Use a copy in case our parent is not assignable:
  if (!adata) {
    // is it non-assignable ?
    typename internal::DataSource<rclcpp::Parameter>::shared_ptr data =
      boost::dynamic_pointer_cast<internal::DataSource<rclcpp::Parameter>>(item);
    if (data) {
      // create a copy
      adata = new internal::ValueDataSource<rclcpp::Parameter>(data->get() );
    }
  }
  if (!adata) {
    log(Error) <<
      "Wrong call to type info function " + this->getTypeName() <<
      "'s getMember() can not process " << item->getTypeName() << endlog();
    return base::DataSourceBase::shared_ptr();
  }

  if (name == "type_name") {
    return internal::newFunctorDataSource(&get_type_name, {adata});
  }
  if (name == "name") {
    return internal::newFunctorDataSource(&get_name, {adata});
  }
  if (name == "value") {
    return internal::newFunctorDataSource(&get_parameter_value, {adata});
  }

  return base::DataSourceBase::shared_ptr();
}

}  // namespace rtt_ros2_rclcpp_typekit
