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

#ifndef OROCOS__RTT_ROS2_SERVICES__ROSSERVICE_HPP_
#define OROCOS__RTT_ROS2_SERVICES__ROSSERVICE_HPP_

#include <string>

#include "rtt/OperationCaller.hpp"
#include "rtt/ServiceRequester.hpp"

namespace rtt_ros2_services
{

class RosService : public RTT::ServiceRequester
{
public:
  typedef boost::shared_ptr<RosService> SharedPtr;

  explicit RosService(RTT::TaskContext * owner)
  : RTT::ServiceRequester("rosservice", owner),
    connect("connect"),
    disconnect("disconnect"),
    disconnectAll("disconnectAll")
  {
    this->addOperationCaller(connect);
    this->addOperationCaller(disconnect);
    this->addOperationCaller(disconnectAll);
  }

  RTT::OperationCaller<bool(const std::string &, const std::string &, const std::string &)> connect;
  RTT::OperationCaller<bool(const std::string &)> disconnect;
  RTT::OperationCaller<void()> disconnectAll;
};

}  // namespace rtt_ros2_services

#endif  // OROCOS__RTT_ROS2_SERVICES__ROSSERVICE_HPP_
