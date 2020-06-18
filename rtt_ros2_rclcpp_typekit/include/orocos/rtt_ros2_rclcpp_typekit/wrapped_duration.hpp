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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_DURATION_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_DURATION_HPP_

#include <utility>

#include "rclcpp/duration.hpp"

namespace rtt_ros2_rclcpp_typekit
{

// Derived type for RTT, which is default constructible.
class WrappedDuration : public rclcpp::Duration
{
public:
  WrappedDuration();
  WrappedDuration(const rclcpp::Duration & duration)  // NOLINT(runtime/explicit)
  : rclcpp::Duration(duration) {}
  WrappedDuration(rclcpp::Duration && duration) noexcept  // NOLINT(runtime/explicit)
  : rclcpp::Duration(std::move(duration)) {}
  template<typename ... Args>
  WrappedDuration(Args && ... args)  // NOLINT(runtime/explicit)
  : rclcpp::Duration(std::forward<Args>(args)...) {}
  WrappedDuration & operator=(const rclcpp::Duration & duration)
  {
    static_cast<rclcpp::Duration &>(*this) = duration;
    return *this;
  }
  WrappedDuration & operator=(rclcpp::Duration && duration) noexcept
  {
    static_cast<rclcpp::Duration &>(*this) = std::move(duration);
    return *this;
  }
};

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_DURATION_HPP_
