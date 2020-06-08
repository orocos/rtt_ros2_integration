// generated from rtt_ros2_idl/src/rtt_ros2_idl/resource/srv_Types.hpp.em
// generated code does not contain a copyright notice

#ifndef OROCOS__@(pkg_name)__SRV__@(srv_name)__TYPES_HPP
#define OROCOS__@(pkg_name)__SRV__@(srv_name)__TYPES_HPP

#include "rtt/internal/AssignCommand.hpp"
#include "rtt/internal/DataSources.hpp"
#include "rtt/Attribute.hpp"
#include "rtt/InputPort.hpp"
#include "rtt/OutputPort.hpp"
#include "rtt/Property.hpp"

#include "@(srv_header)"

extern template class RTT::internal::DataSource< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::internal::AssignableDataSource< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::internal::AssignCommand< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::internal::ValueDataSource< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::internal::ConstantDataSource< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::internal::ReferenceDataSource< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::OutputPort< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::InputPort< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::Property< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::Attribute< @(pkg_name)::srv::@(srv_name)::Request >;
extern template class RTT::Constant< @(pkg_name)::srv::@(srv_name)::Request >;

extern template class RTT::internal::DataSource< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::internal::AssignableDataSource< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::internal::AssignCommand< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::internal::ValueDataSource< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::internal::ConstantDataSource< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::internal::ReferenceDataSource< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::OutputPort< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::InputPort< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::Property< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::Attribute< @(pkg_name)::srv::@(srv_name)::Response >;
extern template class RTT::Constant< @(pkg_name)::srv::@(srv_name)::Response >;

#endif  // OROCOS__@(pkg_name)__SRV__@(srv_name)__TYPES_HPP
