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

#include <string>
#include <utility>

#include "rtt/deployment/ComponentLoader.hpp"
#include "rtt/internal/GlobalService.hpp"
#include "rtt/plugin/ServicePlugin.hpp"

#include "boost/make_shared.hpp"

#include "rtt_ros2_tf2/rtt_ros2_tf2.hpp"


namespace rtt_ros2_tf2
{

static bool loadROSServiceIntoTaskContext(RTT::TaskContext * tc) {
  return true;
}

static bool loadGlobalROSService() {
  return true;
}

extern "C" {
bool loadRTTPlugin(RTT::TaskContext * tc)
{
  if (tc == nullptr) {
    return loadGlobalROSService();
  } else {
    return loadROSServiceIntoTaskContext(tc);
  }
}
std::string getRTTPluginName() {return "tf2";}
std::string getRTTTargetName() {return OROCOS_TARGET_NAME;}
}  // extern "C"

} // namespace rtt_ros2_tf2
