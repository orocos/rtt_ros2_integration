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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__ROS2_TIME_TYPE_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__ROS2_TIME_TYPE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/time.hpp"

#include "rtt/types/PrimitiveTypeInfo.hpp"

#include "time_io.hpp"

namespace rtt_ros2_rclcpp_typekit
{

class TimeTypeInfo
  : public RTT::types::PrimitiveTypeInfo<rclcpp::Time, true>
{
public:
  TimeTypeInfo();

  bool installTypeInfoObject(RTT::types::TypeInfo * ti) override;
};

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__ROS2_TIME_TYPE_HPP_
