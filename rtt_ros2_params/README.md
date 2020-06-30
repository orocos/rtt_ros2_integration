
# Description

ROS2 provides a slightly different infrastructure to deal with parameters. An introduction guide can be found in [Understanding ROS2 prameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/).
The main difference is that there is no longer a centralized parameter server, but instead, parameters are maintained per node.

## `ros2_params` service

This PR provides a new Orocos service called `ros2_params` that can be loaded globally and per Orocos component. The service provides 4 operations:
* `getParameter()`: loads a parameter from the ROS2 node parameter facility and returns its value.
* `loadProperty()`:  loads a parameter from the ROS2 node parameter facility into an Orocos property. If the property doesn't exist, in creates a new one owned by the `Params` service.
* `setParameter()`: sets a parameter into the ROS2 node parameter facility. If the parameter doesn't exists, it declares a new one.
* `storeProperty()`: sets a parameter into the ROS2 node parameter facility from the value of an Orocos property. If the parameter doesn't exists, it declares a new one.

## Usage

This package can be imported with:
```
import("rtt_ros2_params")
```

Then, the service can be loaded into a component named `<component>` with:
```
<component>.loadService('ros2_params')
```

`<component>` will have a new service named `Params` that provides the operations described. These operations can be called with:
```
<component>.Params.setParameter( <args> )
<component>.Params.getParameter( <args> )
<component>.Params.loadProperty( <args> )
<component>.Params.storeProperty( <args> )
```

## Requriements

In order to work, this service requires two other elements being loaded before calling the operations.
* A ROS2 node service must exist for the component where this service is loaded or globally.
* The typekits provided in `rtt_ros2_rclcpp_typekit` and its dependencies must be loaded

These prerequirements can be achieved by:
```
import("rtt_ros2")
import("rtt_ros2_node")
ros.import("rtt_ros2_rclcpp_typekit")

# Optionally
<component>.loadService("ros2_node")
```
