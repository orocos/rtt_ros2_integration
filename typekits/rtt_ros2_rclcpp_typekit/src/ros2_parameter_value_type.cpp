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

#include "rtt_ros2_rclcpp_typekit/ros2_parameter_value_type.hpp"

#include <cassert>
#include <string>
#include <vector>

#include "rtt/types/TemplateConstructor.hpp"
#include "rtt/types/TypeInfoRepository.hpp"

#include "rtt_ros2_rclcpp_typekit/parameter_value_conversions.hpp"

using namespace RTT;  // NOLINT(build/namespaces)
using namespace RTT::types;  // NOLINT(build/namespaces)

namespace rtt_ros2_rclcpp_typekit
{

ParameterValueTypeInfo::ParameterValueTypeInfo()
: PrimitiveTypeInfo<rclcpp::ParameterValue>("/rclcpp/ParameterValue")
{}

bool ParameterValueTypeInfo::installTypeInfoObject(TypeInfo * ti)
{
  // aquire a shared reference to the this object
  boost::shared_ptr<ParameterValueTypeInfo> mthis =
    boost::dynamic_pointer_cast<ParameterValueTypeInfo>(
    this->getSharedPtr());
  assert(mthis);
  // Allow base to install first
  PrimitiveTypeInfo<rclcpp::ParameterValue>::installTypeInfoObject(ti);
  ti->setStreamFactory(mthis);

  // Conversions
  const auto types = TypeInfoRepository::Instance();
  {
    const auto bool_ti = types->getTypeInfo<bool>();
    assert(bool_ti != nullptr);
    if (bool_ti != nullptr) {
      bool_ti->addConstructor(newConstructor(&parameter_value_to_bool));
    }
    ti->addConstructor(newConstructor(&bool_to_parameter_value, true));
  }
  {
    const auto int_ti = types->getTypeInfo<int>();
    assert(int_ti != nullptr);
    if (int_ti != nullptr) {
      int_ti->addConstructor(newConstructor(&parameter_value_to_int));
    }
    ti->addConstructor(newConstructor(&int_to_parameter_value, true));
  }
  {
    const auto int64_ti = types->getTypeInfo<int64_t>();
    assert(int64_ti != nullptr);
    if (int64_ti != nullptr) {
      int64_ti->addConstructor(newConstructor(&parameter_value_to_int64));
    }
    ti->addConstructor(newConstructor(&int64_to_parameter_value, true));
  }
  {
    const auto float_ti = types->getTypeInfo<float>();
    assert(float_ti != nullptr);
    if (float_ti != nullptr) {
      float_ti->addConstructor(newConstructor(&parameter_value_to_float));
    }
    ti->addConstructor(newConstructor(&float_to_parameter_value, true));
  }
  {
    const auto double_ti = types->getTypeInfo<double>();
    assert(double_ti != nullptr);
    if (double_ti != nullptr) {
      double_ti->addConstructor(newConstructor(&parameter_value_to_double));
    }
    ti->addConstructor(newConstructor(&double_to_parameter_value, true));
  }
  {
    const auto string_ti = types->getTypeInfo<std::string>();
    assert(string_ti != nullptr);
    if (string_ti != nullptr) {
      string_ti->addConstructor(newConstructor(&parameter_value_to_string));
    }
    ti->addConstructor(newConstructor(&string_to_parameter_value, true));
  }
  {
    const auto byte_array_ti = types->getTypeInfo<std::vector<uint8_t>>();
    assert(byte_array_ti != nullptr);
    if (byte_array_ti != nullptr) {
      byte_array_ti->addConstructor(newConstructor(&parameter_value_to_byte_array));
    }
    ti->addConstructor(newConstructor(&byte_array_to_parameter_value, true));
  }
  {
    const auto bool_array_ti = types->getTypeInfo<std::vector<bool>>();
    assert(bool_array_ti != nullptr);
    if (bool_array_ti != nullptr) {
      bool_array_ti->addConstructor(newConstructor(&parameter_value_to_bool_array));
    }
    ti->addConstructor(newConstructor(&bool_array_to_parameter_value, true));
  }
  {
    const auto bool_array_ti = types->getTypeInfo<std::vector<bool>>();
    assert(bool_array_ti != nullptr);
    if (bool_array_ti != nullptr) {
      bool_array_ti->addConstructor(newConstructor(&parameter_value_to_bool_array));
    }
    ti->addConstructor(newConstructor(&bool_array_to_parameter_value, true));
  }
  {
    const auto int64_array_ti = types->getTypeInfo<std::vector<int64_t>>();
    assert(int64_array_ti != nullptr);
    if (int64_array_ti != nullptr) {
      int64_array_ti->addConstructor(newConstructor(&parameter_value_to_int64_array));
    }
    ti->addConstructor(newConstructor(&int64_array_to_parameter_value, true));
  }
  {
    const auto int_array_ti = types->getTypeInfo<std::vector<int>>();
    assert(int_array_ti != nullptr);
    if (int_array_ti != nullptr) {
      int_array_ti->addConstructor(newConstructor(&parameter_value_to_int_array));
    }
    ti->addConstructor(newConstructor(&int_array_to_parameter_value, true));
  }
  {
    const auto float_array_ti = types->getTypeInfo<std::vector<float>>();
    assert(float_array_ti != nullptr);
    if (float_array_ti != nullptr) {
      float_array_ti->addConstructor(newConstructor(&parameter_value_to_float_array));
    }
    ti->addConstructor(newConstructor(&float_array_to_parameter_value, true));
  }
  {
    const auto double_array_ti = types->getTypeInfo<std::vector<double>>();
    assert(double_array_ti != nullptr);
    if (double_array_ti != nullptr) {
      double_array_ti->addConstructor(newConstructor(&parameter_value_to_double_array));
    }
    ti->addConstructor(newConstructor(&double_array_to_parameter_value, true));
  }
  {
    const auto string_array_ti = types->getTypeInfo<std::vector<std::string>>();
    assert(string_array_ti != nullptr);
    if (string_array_ti != nullptr) {
      string_array_ti->addConstructor(newConstructor(&parameter_value_to_string_array));
    }
    ti->addConstructor(newConstructor(&string_array_to_parameter_value, true));
  }

  // Don't delete us, we're memory-managed.
  return false;
}

std::ostream & ParameterValueTypeInfo::write(
  std::ostream & os,
  RTT::base::DataSourceBase::shared_ptr in) const
{
  typename internal::DataSource<rclcpp::ParameterValue>::shared_ptr d =
    boost::dynamic_pointer_cast<internal::DataSource<rclcpp::ParameterValue>>(in);
  if (d) {
    os << to_string(d->rvalue());
  }
  return os;
}

std::istream & ParameterValueTypeInfo::read(
  std::istream & is,
  RTT::base::DataSourceBase::shared_ptr /*out*/) const
{
  // not supported
  is.setstate(std::ios_base::failbit);
  return is;
}

}  // namespace rtt_ros2_rclcpp_typekit
