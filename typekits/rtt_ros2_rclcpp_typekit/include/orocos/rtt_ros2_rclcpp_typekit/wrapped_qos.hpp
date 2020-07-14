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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_QOS_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_QOS_HPP_

#include <utility>

#include "rclcpp/qos.hpp"

namespace rtt_ros2_rclcpp_typekit
{

// Derived type for RTT, which is default constructible.
class WrappedQoS : public rclcpp::QoS
{
public:
  WrappedQoS();
  WrappedQoS(const rclcpp::QoS & qos)  // NOLINT(runtime/explicit)
  : rclcpp::QoS(qos) {}
  WrappedQoS(rclcpp::QoS && qos) noexcept  // NOLINT(runtime/explicit)
  : rclcpp::QoS(std::move(qos)) {}
  template<typename ... Args>
  WrappedQoS(Args && ... args)  // NOLINT(runtime/explicit)
  : rclcpp::QoS(std::forward<Args>(args)...) {}
  WrappedQoS & operator=(const rclcpp::QoS & qos)
  {
    static_cast<rclcpp::QoS &>(*this) = qos;
    return *this;
  }
  WrappedQoS & operator=(rclcpp::QoS && qos) noexcept
  {
    static_cast<rclcpp::QoS &>(*this) = std::move(qos);
    return *this;
  }
};

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__WRAPPED_QOS_HPP_
