# Orocos RTT / ROS Integration Packages

[![Build Status](https://travis-ci.org/orocos/rtt_ros2_integration.svg?branch=master)](https://travis-ci.org/orocos/rtt_ros2_integration)

## Introduction

This repository contains ROS 2 packages necessary for building [Orocos Real-Time Toolkit (RTT)](https://www.orocos.org)
libraries, plugins, and components to communicate with ROS 2 nodes and to integrate with
higher-level ROS concepts.

It is the successor of [rtt_ros_integration](https://github.com/orocos/rtt_ros_integration) for ROS 1.

## Packages

The packages in this repository provide:

* [**rtt\_ros2**](rtt_ros2) CMake helpers, scripts and launch files to find and use
  Orocos RTT and OCL in an [ament_cmake](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/)
  package and an RTT service plugin to find and import those packages into Orocos.

* [**rtt\_ros2\_idl**](rtt_ros2_idl) Generate RTT typekits from ROS 2 message and service
  definitions specified in the [ROS interface definition language (IDL)](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/).

* [**rtt\_ros2\_node**](rtt_ros2_node) Instantiate a per-process or per-component
  [ROS 2 node](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Nodes/) from within an
  Orocos deployment.

* [**rtt\_ros2\_topics**](rtt_ros2_topics) Connect Orocos ports to
  [ROS 2 topics](https://index.ros.org/doc/ros2/Tutorials/Topics/Understanding-ROS2-Topics/) for
  real-time safe inter-process communication.

## Contributing

Any contribution to this repository is appreciated, whether they are new features, bug fixes,
improved documentation or just questions. However, it is recommended to first discuss the change
you wish to make via a [GitHub issue](https://github.com/orocos/rtt_ros2_integration/issues).

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

## Acknowledgement

Many thanks to all contributors to the original [rtt_ros_integration](https://github.com/orocos/rtt_ros_integration) for ROS 1 over the years:

- Peter Soetens
- Ruben Smits
- Jonathan Bohren
- Johannes Meyer
- Antoine Hoarau
- ...and [others](https://github.com/orocos/rtt_ros_integration/graphs/contributors).

Your work and your ideas have been the foundation of this new implementation for ROS 2.
