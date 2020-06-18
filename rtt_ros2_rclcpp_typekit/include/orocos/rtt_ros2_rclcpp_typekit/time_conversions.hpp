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

#include <chrono>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rmw/types.h"

#include "wrapped_duration.hpp"

namespace rtt_ros2_rclcpp_typekit
{

static inline double time_to_double(const rclcpp::Time & t)
{
  return t.seconds();
}

static inline rclcpp::Time double_to_time(const double d)
{
  return rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(d)).count());
}

static inline rclcpp::Time double_to_time2(const double d, rcl_clock_type_t clock)
{
  return rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(d)).count(),
    clock);
}

static inline int64_t time_to_int64(const rclcpp::Time & t)
{
  return t.nanoseconds();
}

static inline rclcpp::Time int64_to_time(const int64_t i)
{
  return rclcpp::Time(i);
}

static inline rclcpp::Time int64_to_time2(const int64_t i, rcl_clock_type_t clock)
{
  return rclcpp::Time(i, clock);
}

static inline double duration_to_double(const WrappedDuration & t)
{
  return t.seconds();
}

static inline WrappedDuration double_to_duration(const double d)
{
  return WrappedDuration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(d)).count());
}

static inline int64_t duration_to_int64(const WrappedDuration & t)
{
  return t.nanoseconds();
}

static inline WrappedDuration int64_to_duration(const int64_t i)
{
  return WrappedDuration(i);
}

static inline double rmw_time_t_to_double(const rmw_time_t & t)
{
  return
    static_cast<double>(t.sec) + static_cast<double>(t.nsec) * 1e-9;
}

static inline rmw_time_t double_to_rmw_time_t(const double d)
{
  if (d < 0) {throw std::invalid_argument("rmw_time_t cannot be negative");}
  rmw_time_t t;
  t.sec = static_cast<uint64_t>(d);
  t.nsec = static_cast<uint64_t>((d - static_cast<double>(t.sec)) * 1e9);
  return t;
}

static inline uint64_t rmw_time_t_to_uint64(const rmw_time_t & t)
{
  return
    static_cast<uint64_t>(t.sec) * 1000000000ull + static_cast<uint64_t>(t.nsec);
}

static inline rmw_time_t uint64_to_rmw_time_t(const uint64_t d)
{
  rmw_time_t t;
  t.sec = static_cast<uint64_t>(d / 1000000000L);
  t.nsec = static_cast<uint64_t>(d % 1000000000L);
  return t;
}

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__TIME_CONVERSIONS_HPP_
