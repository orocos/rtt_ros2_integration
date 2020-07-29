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

#ifndef OROCOS__RTT_ROS2_PARAMS__ROSPARAM_HPP_
#define OROCOS__RTT_ROS2_PARAMS__ROSPARAM_HPP_

#include <string>

#include "rtt/OperationCaller.hpp"
#include "rtt/ServiceRequester.hpp"

#include "rclcpp/parameter_value.hpp"

namespace rtt_ros2_params
{

class RosParam : public RTT::ServiceRequester
{
public:
  typedef boost::shared_ptr<RosParam> shared_ptr;

  RosParam(RTT::TaskContext * owner)
  : RTT::ServiceRequester("rosparam", owner),
    getParameter("getParameter"),
    setParameter("setParameter"),
    setOrDeclareParameter("setOrDeclareParameter"),
    loadProperty("loadProperty"),
    storeProperty("storeProperty")
  {
    this->addOperationCaller(getParameter);
    this->addOperationCaller(setParameter);
    this->addOperationCaller(setOrDeclareParameter);

    // Operations loadProperty and storeProperty are only available if the requester is connected
    // to a service loaded into a component. They are added dynamically in connectTo().
  }
  virtual ~RosParam() = default;

  bool connectTo(RTT::Service::shared_ptr sp) override
  {
    const bool has_owner = (sp->getOwner() != nullptr);
    if (has_owner) {
      this->addOperationCaller(loadProperty);
      this->addOperationCaller(storeProperty);
    }
    return RTT::ServiceRequester::connectTo(sp);
  }

  RTT::OperationCaller<rclcpp::ParameterValue(const std::string & name)> getParameter;
  RTT::OperationCaller<bool(
      const std::string & name,
      const rclcpp::ParameterValue & value)> setParameter;
  RTT::OperationCaller<bool(
      const std::string & name,
      const rclcpp::ParameterValue & value)> setOrDeclareParameter;
  RTT::OperationCaller<bool(
      const std::string & property_name,
      const std::string & param_name)> loadProperty;
  RTT::OperationCaller<bool(
      const std::string & property_name,
      const std::string & param_name)> storeProperty;
};

}  // namespace rtt_ros2_params

#endif  // OROCOS__RTT_ROS2_PARAMS__ROSPARAM_HPP_
