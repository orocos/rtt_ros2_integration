@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore 
}@
// generated from rtt_ros2_idl/src/rtt_ros2_idl/resource/Types.hpp.em
// generated code does not contain a copyright notice

#ifndef OROCOS__@(pkg_name)__TYPES_HPP
#define OROCOS__@(pkg_name)__TYPES_HPP

@[for message in messages]@
#include "msg/@(convert_camel_case_to_lower_case_underscore(message))_Types.hpp"
@[end for]@

@[for service in services]@
#include "srv/@(convert_camel_case_to_lower_case_underscore(service))_Types.hpp"
@[end for]@

@[for action in actions]@
#include "action/@(convert_camel_case_to_lower_case_underscore(action))_Types.hpp"
@[end for]@

#endif  // OROCOS__@(pkg_name)__TYPES_HPP
