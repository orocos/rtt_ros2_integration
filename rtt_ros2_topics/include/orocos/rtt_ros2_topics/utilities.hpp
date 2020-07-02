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

#ifndef OROCOS__RTT_ROS2_TOPICS__UTILITIES_HPP_
#define OROCOS__RTT_ROS2_TOPICS__UTILITIES_HPP_

#include <cctype>
#include <string>

#include "rtt/ConnPolicy.hpp"
#include "rtt/base/PortInterface.hpp"

namespace rtt_ros2_topics
{

namespace utilities
{

static std::string convert_camel_case_to_lower_case_underscore(std::string str)
{
  // insert _ before capitals
  for (std::size_t i = 1; i < str.size(); ++i) {
    if (std::isupper(str[i])) {
      str.insert(i, "_");
      ++i;
    }
  }

  // convert to lower case
  std::transform(
    str.begin(), str.end(), str.begin(),
    [](char c) {return static_cast<char>(std::tolower(c));});

  return str;
}

static std::string find_topic(
  RTT::base::PortInterface * port,
  const RTT::ConnPolicy & policy)
{
  std::string topic = policy.name_id;
  if (topic.empty()) {
    // fall back to the port name, converted to lower-case with underscores
    topic = convert_camel_case_to_lower_case_underscore(port->getName());
  }
  if (topic.empty()) {
    throw std::runtime_error("policy.name_id cannot be empty for an unnamed port");
  }

  // TODO(meyerj): Check whether topic is a valid ROS topic name
  return topic;
}

}  // namespace utilities
}  // namespace rtt_ros2_topics

#endif  // OROCOS__RTT_ROS2_TOPICS__UTILITIES_HPP_
