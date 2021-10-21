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

#include "rtt_ros2_tf2/rtt_ros2_tf2_component.hpp"

#include <string>

#include "rtt/Component.hpp"

namespace rtt_ros2_tf2
{

TF2_Component::TF2_Component(const std::string & name)
: RTT::TaskContext(name, PreOperational)
{
  RTT::Logger::In(this->getName());
  RTT::log(RTT::Info) << "Constructing component!" << RTT::endlog();
}


bool TF2_Component::configureHook()
{
  RTT::Logger::In(this->getName());

  RTT::log(RTT::Info) << "Configuring component!" << RTT::endlog();
  return true;
}


void TF2_Component::updateHook()
{
  RTT::Logger::In(this->getName());
  RTT::log(RTT::Info) << "Running component!" << RTT::endlog();
}

void TF2_Component::cleanupHook()
{
  RTT::Logger::In(this->getName());
  RTT::log(RTT::Info) << "Cleaning up component!" << RTT::endlog();
}
}  // namespace rtt_ros2_tf2

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Rtt_tf)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(rtt_ros2_tf2::TF2_Component)
