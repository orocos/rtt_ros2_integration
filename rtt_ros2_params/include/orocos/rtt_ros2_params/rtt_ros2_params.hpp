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

#ifndef OROCOS__RTT_ROS2_PARAMS__RTT_ROS2_PARAMS_HPP_
#define OROCOS__RTT_ROS2_PARAMS__RTT_ROS2_PARAMS_HPP_

#include <map>
#include <string>

#include "rtt/Service.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"

namespace rtt_ros2_params
{

class Params : public RTT::Service
{
public:
  explicit Params(RTT::TaskContext * owner);
  virtual ~Params();

protected:
  rclcpp::ParameterValue getParameter(const std::string& name);
  bool setParameter(const std::string& name, const rclcpp::ParameterValue& value);
  bool loadProperty(const std::string& property_name, const std::string& param_name = std::string());
  bool storeProperty(const std::string& property_name, const std::string& param_name);

private:
  bool check_ros2_node_in_component();
  bool check_ros2_node_in_global();
  // bool get_ros2_node(rclcpp::Node::SharedPtr& node_ptr);

  std::map<std::string, rclcpp::ParameterValue> orphan_properties_;

  RTT::TaskContext * owner_;
};  // class Params

}  // namespace rtt_ros2_params

#endif  // OROCOS__RTT_ROS2_PARAMS__RTT_ROS2_PARAMS_HPP_
