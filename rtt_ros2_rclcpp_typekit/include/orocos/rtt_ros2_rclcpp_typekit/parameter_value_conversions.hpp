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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__PARAMETER_VALUE_CONVERSIONS_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__PARAMETER_VALUE_CONVERSIONS_HPP_

#include <string>
#include <vector>

#include "rclcpp/parameter_value.hpp"

namespace rtt_ros2_rclcpp_typekit
{

static inline const bool & parameter_value_to_bool(const rclcpp::ParameterValue & t)
{
  return t.get<bool>();
}

static inline rclcpp::ParameterValue bool_to_parameter_value(const bool bool_value)
{
  return rclcpp::ParameterValue(bool_value);
}

static inline int parameter_value_to_int(const rclcpp::ParameterValue & t)
{
  return static_cast<int>(t.get<int64_t>());
}

static inline rclcpp::ParameterValue int_to_parameter_value(const int int_value)
{
  return rclcpp::ParameterValue(int_value);
}

static inline const int64_t & parameter_value_to_int64(const rclcpp::ParameterValue & t)
{
  return t.get<int64_t>();
}

static inline rclcpp::ParameterValue int64_to_parameter_value(const int64_t int_value)
{
  return rclcpp::ParameterValue(int_value);
}

static inline float parameter_value_to_float(const rclcpp::ParameterValue & t)
{
  return static_cast<float>(t.get<double>());
}

static inline rclcpp::ParameterValue float_to_parameter_value(const float float_value)
{
  return rclcpp::ParameterValue(float_value);
}

static inline const double & parameter_value_to_double(const rclcpp::ParameterValue & t)
{
  return t.get<double>();
}

static inline rclcpp::ParameterValue double_to_parameter_value(const double double_value)
{
  return rclcpp::ParameterValue(double_value);
}

static inline const std::string & parameter_value_to_string(const rclcpp::ParameterValue & t)
{
  return t.get<std::string>();
}

static inline rclcpp::ParameterValue string_to_parameter_value(const std::string & string_value)
{
  return rclcpp::ParameterValue(string_value);
}

static inline const std::vector<uint8_t> & parameter_value_to_byte_array(
  const rclcpp::ParameterValue & t)
{
  return t.get<std::vector<uint8_t>>();
}

static inline rclcpp::ParameterValue byte_array_to_parameter_value(
  const std::vector<uint8_t> & byte_array_value)
{
  return rclcpp::ParameterValue(byte_array_value);
}

static inline const std::vector<bool> & parameter_value_to_bool_array(
  const rclcpp::ParameterValue & t)
{
  return t.get<std::vector<bool>>();
}

static inline rclcpp::ParameterValue bool_array_to_parameter_value(
  const std::vector<bool> & byte_array_value)
{
  return rclcpp::ParameterValue(byte_array_value);
}

static inline const std::vector<int64_t> & parameter_value_to_int64_array(
  const rclcpp::ParameterValue & t)
{
  return t.get<std::vector<int64_t>>();
}

static inline rclcpp::ParameterValue int64_array_to_parameter_value(
  const std::vector<int64_t> & int_array_value)
{
  return rclcpp::ParameterValue(int_array_value);
}

static inline std::vector<int> parameter_value_to_int_array(
  const rclcpp::ParameterValue & t)
{
  const auto & int64_array = t.get<std::vector<int64_t>>();
  return std::vector<int>(int64_array.begin(), int64_array.end());
}

static inline rclcpp::ParameterValue int_array_to_parameter_value(
  const std::vector<int> & int_array_value)
{
  return rclcpp::ParameterValue(int_array_value);
}

static inline std::vector<float> parameter_value_to_float_array(
  const rclcpp::ParameterValue & t)
{
  const auto & double_array = t.get<std::vector<double>>();
  return std::vector<float>(double_array.begin(), double_array.end());
}

static inline rclcpp::ParameterValue float_array_to_parameter_value(
  const std::vector<float> & float_array_value)
{
  return rclcpp::ParameterValue(float_array_value);
}

static inline const std::vector<double> & parameter_value_to_double_array(
  const rclcpp::ParameterValue & t)
{
  return t.get<std::vector<double>>();
}

static inline rclcpp::ParameterValue double_array_to_parameter_value(
  const std::vector<double> & double_array_value)
{
  return rclcpp::ParameterValue(double_array_value);
}

static inline const std::vector<std::string> & parameter_value_to_string_array(
  const rclcpp::ParameterValue & t)
{
  return t.get<std::vector<std::string>>();
}

static inline rclcpp::ParameterValue string_array_to_parameter_value(
  const std::vector<std::string> & string_array_value)
{
  return rclcpp::ParameterValue(string_array_value);
}

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__PARAMETER_VALUE_CONVERSIONS_HPP_
