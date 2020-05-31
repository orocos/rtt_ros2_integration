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

#include <rtt/Attribute.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

namespace rtt_ros2_tests
{

class TestService : public RTT::Service
{
public:
  explicit TestService(RTT::TaskContext * owner = 0)
  : RTT::Service("TestService", owner)
  {
    this->setValue(new RTT::Constant<double>("constant", 5.0));

    if (owner == nullptr) {
      RTT::log(RTT::Info) <<
        "Loaded service " << getName() << " into the global service." <<
        RTT::endlog();
    } else {
      RTT::log(RTT::Info) <<
        "Loaded service " << getName() << " into component " <<
        owner->getName() << "." <<
        RTT::endlog();
    }
  }
};

}  // namespace rtt_ros2_tests

ORO_SERVICE_PLUGIN(rtt_ros2_tests::TestService)
