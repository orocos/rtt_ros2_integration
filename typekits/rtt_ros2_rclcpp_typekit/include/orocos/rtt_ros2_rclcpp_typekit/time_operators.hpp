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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_OPERATORS_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_OPERATORS_HPP_

#include <chrono>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "wrapped_duration.hpp"

namespace rtt_ros2_rclcpp_typekit
{

struct duration_plus_duration
{
  typedef WrappedDuration first_argument_type;
  typedef WrappedDuration second_argument_type;
  typedef WrappedDuration result_type;
  WrappedDuration operator()(const WrappedDuration & da, const WrappedDuration & db) const
  {
    return da + db;
  }
};

struct duration_minus_duration
{
  typedef WrappedDuration first_argument_type;
  typedef WrappedDuration second_argument_type;
  typedef WrappedDuration result_type;
  WrappedDuration operator()(const WrappedDuration & da, const WrappedDuration & db) const
  {
    return da - db;
  }
};

struct time_plus_duration
{
  typedef rclcpp::Time first_argument_type;
  typedef WrappedDuration second_argument_type;
  typedef rclcpp::Time result_type;
  rclcpp::Time operator()(const rclcpp::Time & t, const WrappedDuration & d) const
  {
    return t + d;
  }
};

struct time_minus_duration
{
  typedef rclcpp::Time first_argument_type;
  typedef WrappedDuration second_argument_type;
  typedef rclcpp::Time result_type;
  rclcpp::Time operator()(const rclcpp::Time & t, const WrappedDuration & d) const
  {
    return t - d;
  }
};

struct time_minus_time
{
  typedef rclcpp::Time first_argument_type;
  typedef rclcpp::Time second_argument_type;
  typedef WrappedDuration result_type;
  WrappedDuration operator()(const rclcpp::Time & ta, const rclcpp::Time & tb) const
  {
    return ta - tb;
  }
};

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_OPERATORS_HPP_
