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


#include "rtt/Service.hpp"

namespace rtt_ros2_params {

class Params : public RTT::Service {
  public:
    Params(RTT::TaskContext *owner);
    virtual ~Params();

  protected:
    bool get_parameter(const std::string name, const std::string ns);

  private:
    bool check_ros2_node_in_component();
    bool check_ros2_node_in_global();

    RTT::TaskContext *owner_;

}; // class Params

} // namespace rtt_ros2_params

#endif // OROCOS__RTT_ROS2_PARAMS__RTT_ROS2_PARAMS_HPP_