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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_CONVERSIONS_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_CONVERSIONS_HPP_

#include "rmw/types.h"

namespace rtt_ros2_rclcpp_typekit
{

static inline double rmw_time_t_to_double(const rmw_time_t & t)
{
  return
    static_cast<double>(t.sec) + static_cast<double>(t.nsec) * 1e-9;
}

static inline rmw_time_t double_to_rmw_time_t(const double d)
{
  rmw_time_t t;
  t.sec = static_cast<uint64_t>(d);
  t.nsec = static_cast<uint64_t>((d - static_cast<double>(t.sec)) * 1e9);
  return t;
}

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_CONVERSIONS_HPP_
