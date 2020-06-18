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

#ifndef OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__RCLCPP_VERSION_H_
#define OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__RCLCPP_VERSION_H_

/* True if the version of rclcpp is at least major.minor.patch */
// (from https://github.com/ros2/rmw_cyclonedds/pull/51)
#define rclcpp_VERSION_GTE(major, minor, patch) ( \
    major<rclcpp_VERSION_MAJOR ? true \
    : major> rclcpp_VERSION_MAJOR ? false \
    : minor<rclcpp_VERSION_MINOR ? true \
    : minor> rclcpp_VERSION_MINOR ? false \
    : patch<rclcpp_VERSION_PATCH ? true \
    : patch> rclcpp_VERSION_PATCH ? false \
    : true)

#endif  // OROCOS__RTT_ROS2_RCLCPP_TYPEKIT__RCLCPP_VERSION_H_
