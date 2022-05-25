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

#ifndef OROCOS__RTT_ROS2_PARAMS__RTT_ROS2_PARAMS_SERVICE_HPP_
#define OROCOS__RTT_ROS2_PARAMS__RTT_ROS2_PARAMS_SERVICE_HPP_

#include <map>
#include <string>

#include "rtt/Service.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"

namespace rtt_ros2_params
{

class RosParamService : public RTT::Service
{
public:
  typedef boost::shared_ptr<RosParamService> shared_ptr;

  explicit RosParamService(RTT::TaskContext * owner);
  virtual ~RosParamService();

protected:
  rclcpp::ParameterValue getParameter(const std::string & name);
  bool setParameter(
    const std::string & name,
    const rclcpp::ParameterValue & value);
  bool declareParameter(
    const std::string & name,
    const rclcpp::ParameterValue & default_value);
  bool setOrDeclareParameter(
    const std::string & name,
    const rclcpp::ParameterValue & value);
  bool loadProperty(
    const std::string & property_name,
    const std::string & param_name = std::string());
  bool storeProperty(
    const std::string & property_name,
    const std::string & param_name);
};  // class RosParamService

}  // namespace rtt_ros2_params

#endif  // OROCOS__RTT_ROS2_PARAMS__RTT_ROS2_PARAMS_SERVICE_HPP_
