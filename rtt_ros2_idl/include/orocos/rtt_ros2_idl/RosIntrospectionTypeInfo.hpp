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

#ifndef OROCOS__RTT_ROS2_IDL__ROSINTROSPECTIONTYPEINFO_HPP_
#define OROCOS__RTT_ROS2_IDL__ROSINTROSPECTIONTYPEINFO_HPP_

#include <algorithm>
#include <cassert>
#include <map>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "rtt/types/TemplateTypeInfo.hpp"

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"

namespace rtt_ros2_idl
{

static inline const rosidl_typesupport_introspection_cpp::MessageMember * findMessageMember(
  const rosidl_typesupport_introspection_cpp::MessageMembers * rosidl_message_members,
  const std::string & name)
{
  if (rosidl_message_members == nullptr) {return nullptr;}
  return std::find_if(
    rosidl_message_members->members_,
    rosidl_message_members->members_ + rosidl_message_members->member_count_,
    [&](const rosidl_typesupport_introspection_cpp::MessageMember & member) {
      return member.name_ == name;
    });
}

template<typename ValueT>
static inline const RTT::types::TypeInfo * findTypeInfoByValueType(
  const rosidl_typesupport_introspection_cpp::MessageMember * rosidl_message_member)
{
  static const auto types = RTT::types::Types();
  if (rosidl_message_member == nullptr) {return nullptr;}

  if (rosidl_message_member->is_array_) {
    if (rosidl_message_member->array_size_ == 0) {
      // dynamic array
      return types->getTypeInfo<std::vector<ValueT>>();
    } else if (!rosidl_message_member->is_upper_bound_) {
      // fixed-size array
      return types->getTypeInfo<RTT::types::carray<ValueT>>();
    } else if (rosidl_message_member->is_upper_bound_) {
      // bounded array
      // not yet supported
      return types->getTypeInfo<std::vector<ValueT>>();
      // return nullptr;
    }
  } else {
    return types->getTypeInfo<ValueT>();
  }
}

static inline const RTT::types::TypeInfo * findTypeInfoByName(
  const std::string & name,
  const rosidl_typesupport_introspection_cpp::MessageMember * rosidl_message_member)
{
  static const auto types = RTT::types::Types();
  if (rosidl_message_member == nullptr) {return nullptr;}

  if (rosidl_message_member->is_array_) {
    if (rosidl_message_member->array_size_ == 0) {
      // dynamic array
      return types->type(name + "[]");
    } else if (!rosidl_message_member->is_upper_bound_) {
      // fixed-size array
      return types->type(name + "[c]");
    } else if (rosidl_message_member->is_upper_bound_) {
      // bounded array
      // not yet supported
      return types->type(name + "[]");
      // return nullptr;
    }
  } else {
    return types->type(name);
  }
}

static inline const RTT::types::TypeInfo * findTypeInfo(
  const rosidl_typesupport_introspection_cpp::MessageMember * rosidl_message_member)
{
  const auto types = RTT::types::Types();
  const RTT::types::TypeInfo * ti = nullptr;

  // Handle primitive types
  using namespace rosidl_typesupport_introspection_cpp;  // NOLINT
  switch (rosidl_message_member->type_id_) {
    case ROS_TYPE_FLOAT:
      ti = findTypeInfoByValueType<float>(rosidl_message_member);
      break;
    case ROS_TYPE_DOUBLE:
      ti = findTypeInfoByValueType<double>(rosidl_message_member);
      break;
    case ROS_TYPE_LONG_DOUBLE:
      ti = findTypeInfoByValueType<long double>(rosidl_message_member);
      break;
    case ROS_TYPE_CHAR:
      ti = findTypeInfoByValueType<char>(rosidl_message_member);
      break;
    case ROS_TYPE_WCHAR:
      ti = findTypeInfoByValueType<uint16_t>(rosidl_message_member);
      break;
    case ROS_TYPE_BOOLEAN:
      ti = findTypeInfoByValueType<bool>(rosidl_message_member);
      break;
    case ROS_TYPE_OCTET:
      ti = findTypeInfoByValueType<unsigned char>(rosidl_message_member);
      break;
    case ROS_TYPE_UINT8:
      ti = findTypeInfoByValueType<uint8_t>(rosidl_message_member);
      break;
    case ROS_TYPE_INT8:
      ti = findTypeInfoByValueType<int8_t>(rosidl_message_member);
      break;
    case ROS_TYPE_UINT16:
      ti = findTypeInfoByValueType<uint16_t>(rosidl_message_member);
      break;
    case ROS_TYPE_INT16:
      ti = findTypeInfoByValueType<int16_t>(rosidl_message_member);
      break;
    case ROS_TYPE_UINT32:
      ti = findTypeInfoByValueType<uint32_t>(rosidl_message_member);
      break;
    case ROS_TYPE_INT32:
      ti = findTypeInfoByValueType<int32_t>(rosidl_message_member);
      break;
    case ROS_TYPE_UINT64:
      ti = findTypeInfoByValueType<uint64_t>(rosidl_message_member);
      break;
    case ROS_TYPE_INT64:
      ti = findTypeInfoByValueType<int64_t>(rosidl_message_member);
      break;
    case ROS_TYPE_STRING:
      ti = findTypeInfoByValueType<std::string>(rosidl_message_member);
      break;
    case ROS_TYPE_WSTRING:
      ti = findTypeInfoByValueType<std::u16string>(rosidl_message_member);
      break;
    case ROS_TYPE_MESSAGE:
      if (rosidl_message_member->members_ != nullptr) {
        static const auto namespace_separator = std::regex("::");
        const auto * const members =
          static_cast<const MessageMembers *>(rosidl_message_member->members_->data);
        const std::string type_name =
          "/" + std::regex_replace(members->message_namespace_,
            namespace_separator,
            "/",
            std::regex_constants::match_any) +
          "/" + members->message_name_;
        ti = findTypeInfoByName(type_name, rosidl_message_member);
      }
      break;
    default:
      RTT::log(RTT::Error) <<
        "Unkown type_id in struct rosidl_typesupport_introspection_cpp::MessageMember for "
        "member '" << rosidl_message_member->name_ << ": " <<
        static_cast<int>(rosidl_message_member->type_id_) <<
        RTT::endlog();
      return nullptr;
  }

  if (ti == nullptr) {
    RTT::log(RTT::Error) <<
      "Failed to lookup TypeInfo for member '" <<
      rosidl_message_member->name_ << "' with type_id " <<
      static_cast<int>(rosidl_message_member->type_id_) << ": "
      "The type is not known by the Orocos type system." << RTT::endlog();
    return nullptr;
  }

  RTT::log(RTT::Debug) <<
    "Found TypeInfo " << ti->getTypeName() << " for field " <<
    rosidl_message_member->name_ << RTT::endlog();
  return ti;
}

/**
 * Type Information for ROS 2 message types for which a rosidl_typesupport_introspection_cpp
 * support library is available.
 *
 * Use this class to register your data type to the Orocos type system.
 */
template<typename T>
class RosIntrospectionTypeInfo
  : public RTT::types::TemplateTypeInfo<T, false>,
  public RTT::types::MemberFactory
{
public:
  using Base = typename RTT::types::TemplateTypeInfo<T, false>;
  static_assert(
    std::is_standard_layout<T>::value,
    "RosIntrospectionTypeInfo<T> requires that type T satisfies StandardLayoutType.");

  explicit RosIntrospectionTypeInfo(std::string name)
  : Base(std::move(name)),
    rosidl_typesupport_(rosidl_typesupport_introspection_cpp::get_message_type_support_handle<T>())
  {
    assert(rosidl_typesupport_);
    if (rosidl_typesupport_) {
      rosidl_message_members_ =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        rosidl_typesupport_->data);
    }
  }

  bool installTypeInfoObject(RTT::types::TypeInfo * ti) override
  {
    // aquire a shared reference to the this object
    boost::shared_ptr<RosIntrospectionTypeInfo<T>> mthis =
      boost::dynamic_pointer_cast<RosIntrospectionTypeInfo<T>>(this->getSharedPtr());
    assert(mthis);
    // Allow base to install first
    Base::installTypeInfoObject(ti);
    // Install the member factory
    if (rosidl_message_members_ != nullptr) {
      ti->setMemberFactory(mthis);
    }

    // Don't delete us, we're memory-managed.
    return false;
  }

  std::vector<std::string> getMemberNames() const override
  {
    if (rosidl_message_members_ == nullptr) {return {};}

    std::vector<std::string> names;
    names.reserve(rosidl_message_members_->member_count_);
    for (uint32_t i = 0; i < rosidl_message_members_->member_count_; ++i) {
      names.push_back(rosidl_message_members_->members_[i].name_);
    }

    return names;
  }

  /**
   * Returns a member of a given data source identified by a data source \a id. This will be an
   * int (for indexing) or a string (for the member name).
   *
   * Only required for the index operator [] in scripting.
   * Not implemented for \ref RosIntrospectionTypeInfo<T>.
   */
  RTT::base::DataSourceBase::shared_ptr getMember(
    RTT::base::DataSourceBase::shared_ptr /*item*/,
    RTT::base::DataSourceBase::shared_ptr /*id*/) const override
  {
    return {};
  }

  /**
   * Returns a member of a given data source struct identified by its name.
   *
   * @param item The item of which to return a reference to a member
   * @param name The name of a member within \a item
   * @return null if no such member exists, an assignable datasource referencing that member otherwise.
   * @sa \ref RTT::types::MemberFactory::getMember(base::DataSourceBase::shared_ptr item, const std::string& name) const
   */
  RTT::base::DataSourceBase::shared_ptr getMember(
    RTT::base::DataSourceBase::shared_ptr item,
    const std::string & name) const override
  {
    const auto rosidl_message_member = findMessageMember(rosidl_message_members_, name);
    if (rosidl_message_member == nullptr) {return {};}
    const auto ti = findTypeInfo(rosidl_message_member);
    if (ti == nullptr) {return {};}

    typename RTT::internal::AssignableDataSource<T>::shared_ptr adata =
      boost::dynamic_pointer_cast<RTT::internal::AssignableDataSource<T>>(item);

    // Use a copy in case our parent is not assignable:
    if (!adata) {
      // is it non-assignable ?
      typename RTT::internal::DataSource<T>::shared_ptr data =
        boost::dynamic_pointer_cast<RTT::internal::DataSource<T>>(item);
      if (data) {
        // Create a copy
        // TODO(meyerj): Isn't there a better way to handle non-assignable data?
        // Why does getMember() need to return an assignable data source at all?
        adata = new RTT::internal::ValueDataSource<T>(data->get());
      }
    }

    const auto member_address =
      reinterpret_cast<std::uintptr_t>(adata->getRawPointer()) +
      rosidl_message_member->offset_;
    return ti->buildPart(
      reinterpret_cast<void *>(member_address),
      static_cast<int>(rosidl_message_member->array_size_),
      item);
  }

  bool resize(RTT::base::DataSourceBase::shared_ptr /*arg*/, int /*size*/) const override
  {
    return false;
  }

private:
  const rosidl_message_type_support_t * rosidl_typesupport_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * rosidl_message_members_;
};

}  // namespace rtt_ros2_idl

#endif  // OROCOS__RTT_ROS2_IDL__ROSINTROSPECTIONTYPEINFO_HPP_
