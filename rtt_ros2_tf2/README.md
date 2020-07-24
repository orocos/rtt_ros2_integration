RTT ROS2 TF2
============

This package provides an Orocos RTT service for utilizing the [TF2 rigid body
transform library](https://index.ros.org/doc/ros2/Tutorials/tf2) from within
Orocos.
This service has operations for requesting and broadcasting sing and
batch transforms.

## Service description

The service provided by the library exposes some operations to interact with TF.
The operations available are:
* `sendTransform()`:
* `sendTransforms()`:
* `sendStaticTransform()`:
* `sendStaticTransforms()`:
* `lookupTransform()`:
* `clear()`:


### Usage

#### Scripting API (*.ops)

The package can be loaded independently form other related `ros` packages, but it requires the `rosnode` service
to work.

The service can be loaded globally by importing the package:

```
import("rtt_ros2")
ros.import("rtt_ros2_tf2")
```
This will create a global service under the `ros` namespace. The operations run by this
service will be executed in the `ClientThread` and are NOT real-time safe.

To load the service into a component, after importing the package, it can be loaded with:

```
loadService("<component-name>", "tf2")
```

The component version will run the calls in the `OwnThread`. The component should not be
real time.

### C++ API

*ToDo*
