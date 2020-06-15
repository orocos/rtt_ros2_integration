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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__ROS2_QOS_TYPE_HPP_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__ROS2_QOS_TYPE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"

#include "rtt/types/MemberFactory.hpp"
#include "rtt/types/PrimitiveTypeInfo.hpp"

namespace rtt_ros2_rclcpp_typekit
{

// Derived type for RTT, which is default constructible.
class QoS : public rclcpp::QoS
{
public:
  QoS();
  explicit QoS(const rclcpp::QoS & qos)
  : rclcpp::QoS(qos) {}
  explicit QoS(rclcpp::QoS && qos) noexcept
  : rclcpp::QoS(std::move(qos)) {}
  template<typename ... Args>
  explicit QoS(Args && ... args)
  : rclcpp::QoS(std::forward<Args>(args)...) {}
  QoS & operator=(const rclcpp::QoS & qos)
  {
    static_cast<rclcpp::QoS &>(*this) = qos;
    return *this;
  }
  QoS & operator=(rclcpp::QoS && qos) noexcept
  {
    static_cast<rclcpp::QoS &>(*this) = std::move(qos);
    return *this;
  }
};

class QoSTypeInfo
  : public RTT::types::PrimitiveTypeInfo<QoS>,
  public RTT::types::MemberFactory
{
public:
  QoSTypeInfo();

  bool installTypeInfoObject(RTT::types::TypeInfo * ti) override;

  std::vector<std::string> getMemberNames() const override;

  RTT::base::DataSourceBase::shared_ptr getMember(
    RTT::base::DataSourceBase::shared_ptr item,
    const std::string & part_name) const override;
};

}  // namespace rtt_ros2_rclcpp_typekit

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__ROS2_QOS_TYPE_HPP_
