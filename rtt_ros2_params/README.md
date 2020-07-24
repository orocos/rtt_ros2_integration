
`rtt_ros2_params` - ROS parameter support
=========================================

# Description

ROS2 provides a slightly different infrastructure to deal with parameters compared to ROS 1. An introduction guide can be found in [Understanding ROS2 parameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/).
The main difference is that there is no longer a centralized parameter server, but instead, parameters are maintained per node.

## `rosparam` service

This package provides a new Orocos service called `rosparam` that can be loaded globally and per Orocos component. The service provides 4 operations:
* `getParameter()`: loads a parameter from the ROS2 node parameter facility and returns its value.
* `loadProperty()`:  loads a parameter from the ROS2 node parameter facility into an Orocos property.
* `setParameter()`: sets a parameter into the ROS2 node parameter facility. If the parameter does not exist, it declares a new one.
* `storeProperty()`: sets a parameter into the ROS2 node parameter facility from the value of an Orocos property. If the parameter doesn't exists, it declares a new one.

## Scripting Interface

This package can be imported with:
```
import("rtt_ros2_params")
```

Then, the service can be loaded into a component named `<component>` with:
```
<component>.loadService('rosparam')
```

`<component>` will have a new service named `Params` that provides the operations described. These operations can be called with:
```
<component>.rosparam.setParameter( <args> )
<component>.rosparam.getParameter( <args> )
<component>.rosparam.loadProperty( <args> )
<component>.rosparam.storeProperty( <args> )
```

## Requirements

In order to work, this service requires two other elements being loaded before calling the operations.
* A ROS2 node service must exist for the component where this service is loaded or globally.
* The typekits provided in `rtt_ros2_rclcpp_typekit` and its dependencies must be loaded

These prerequirements can be achieved by:
```
import("rtt_ros2")
ros.import("rtt_ros2_params")
```

To load a ROS2 node specific for a component, please, read the [related documentation](../rtt_ros2_node/README.md).
