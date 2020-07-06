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

#include "rtt_ros2_params/rtt_ros2_params.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"

#include "rtt/internal/GlobalService.hpp"
#include "rtt/Property.hpp"
#include "rtt/TaskContext.hpp"

#include "rtt_ros2_node/rtt_ros2_node.hpp"

namespace rtt_ros2_params
{

Params::Params(RTT::TaskContext * owner)
: RTT::Service("Params", owner)
{
  RTT::Logger::In in(getName());

  RTT::log(RTT::Info) <<
    "Parameter service instantiated! in " << getName() <<
    RTT::endlog();

  RTT::log(RTT::Debug) <<
    "Name of the owner: " << owner->getName() <<
    getName() << RTT::endlog();

  this->doc(
    "RTT Service for synchronizing ROS 2 parameters with the properties of a corresponding "
    "RTT component");

  addOperation("getParameter", &Params::getParameter, this, RTT::ClientThread)
  .doc("Gets the value of a ROS 2 parameter")
  .arg("name", "Name of the parameter to retrieve");

  addOperation("setParameter", &Params::setParameter, this, RTT::ClientThread)
  .doc("Sets a parameter to the node parameter server")
  .arg("name", "Name of the parameter to set")
  .arg("value", "The value of the parameter to set");

  if (nullptr != owner) {
    addOperation("loadProperty", &Params::loadProperty, this, RTT::ClientThread)
    .doc("Loads a parameter from the node parameter server into a Property")
    .arg("parameter_name", "Name of the parameter to retrieve")
    .arg("property_name", "Name of the property to load into");

    addOperation("storeProperty", &Params::storeProperty, this, RTT::ClientThread)
    .doc("Stores a property into a node parameter server")
    .arg("property_name", "Name of the property to store")
    .arg("parameter_name", "Name of the parameter to store into");
  }
}

Params::~Params() = default;

bool Params::check_ros2_node_in_component()
{
  return getOwner()->provides()->hasService("Node");
}

bool Params::check_ros2_node_in_global()
{
  return RTT::internal::GlobalService::Instance()->hasService("ros");
}

rclcpp::ParameterValue Params::getParameter(const std::string& name)
{
  RTT::Logger::In in(getName());

  RTT::log(RTT::Debug) <<
    "Retrieving the parameter \"" << name << "\"" <<
    RTT::endlog();

  rclcpp::ParameterValue parameter_value;

  rclcpp::Node::SharedPtr rosnode;
  if (nullptr != (rosnode = rtt_ros2_node::getNode(getOwner()))) {
    rosnode->get_parameter(name, parameter_value);
  } else {
    RTT::log(RTT::Error) <<
      "No ROS node service from package rtt_ros2_node loaded into this "
      "component or as a global service." <<
      RTT::endlog();
  }

  return parameter_value;
}

bool Params::setParameter(const std::string& name, const rclcpp::ParameterValue& value)
{
  RTT::Logger::In in(getName());

  RTT::log(RTT::Debug) <<
    "Setting the parameter \"" << name << "\"" <<
    RTT::endlog();

  rclcpp::Node::SharedPtr rosnode;
  if (nullptr != (rosnode = rtt_ros2_node::getNode(getOwner()))) {
    if (!rosnode->has_parameter(name)) {
      // Create new parameter
      RTT::log(RTT::Warning) <<
        "The parameter did not exist" <<
        RTT::endlog();
      // rosnode->declare_parameter(name);
    }

    // Update the parameter
    try {
      const auto result = rosnode->set_parameter(rclcpp::Parameter(name, value));
      if (!result.successful) {
        RTT::log(RTT::Error) <<
          "Failed to set ROS parameter " << name << ": " <<
          result.reason << RTT::endlog();
        return false;
      }
    } catch (rclcpp::exceptions::ParameterNotDeclaredException & e) {
      RTT::log(RTT::Error) <<
        "Failed to create parameter " << name << ": " <<
        e.what() << RTT::endlog();
        return false;
    }
  } else {
    RTT::log(RTT::Error) <<
      "No ROS node service from package rtt_ros2_node loaded into this "
      "component or as a global service." <<
      RTT::endlog();
    return false;
  }
  return true;
}

bool Params::loadProperty(const std::string& property_name, const std::string& param_name)
{
  RTT::Logger::In in(getName());

  rclcpp::ParameterValue paramvalue = getParameter(param_name);
  if (rclcpp::PARAMETER_NOT_SET == paramvalue.get_type()) {
    RTT::log(RTT::Error) <<
      "The parameter " << param_name << " could not be retrieved" <<
      RTT::endlog();
    return false;
  }

  if (nullptr == getOwner()) {
    // Case in which the service is GlobalService
    RTT::log(RTT::Error) << "[" << getName() << "] No owner found, properties "
    "cannot be loaded in GlobalService" << RTT::endlog();
    return false;
  }
  auto prop = getOwner()->properties()->getProperty(property_name);
  if (nullptr != prop) {
    RTT::log(RTT::Debug) <<
      getOwner()->provides()->getName() <<
      " has a property " << property_name << RTT::endlog();
  } else {
    RTT::log(RTT::Info) <<
      getOwner()->provides()->getName() <<
      " has NO property " << property_name << RTT::endlog();
    for (const auto property : getOwner()->properties()->getProperties() ) {
      RTT::log(RTT::Debug) <<
        "Available property: " << property->getName() <<
        RTT::endlog();
    }
  }

  if (nullptr == prop) {
    // Try to find it among orphan parameters (previously loaded, without a component interface)
    if (orphan_properties_.find(property_name) == orphan_properties_.end()) {
      // When totally new, then create an orphan property, i.e. owned by the ros2-params service
      orphan_properties_[property_name] = paramvalue;
      this->addProperty(property_name, orphan_properties_[property_name])
      .doc("Property loaded from ROS2 param: " + param_name);
      prop = getProperty(property_name);
    } else {
      orphan_properties_[property_name] = paramvalue;
      prop = getProperty(property_name);
    }
    return true;
  } else {
    // The property exists on the Owner's interface
    RTT::log(RTT::Debug) <<
      "The property " << property_name << " existed" <<
      RTT::endlog();

    RTT::Property<rclcpp::ParameterValue> prop_paramvalue = prop;
    const auto property_ds = prop->getDataSource();
    RTT::internal::ReferenceDataSource<rclcpp::ParameterValue> param_rds(paramvalue);
    try {
      if (property_ds->update(&param_rds)) {
        // conversion successful
        RTT::log(RTT::Debug) <<
          "Property " <<
          getOwner()->provides()->getName() << "." << property_name <<
          " loaded successfully from ROS parameter " << param_name << RTT::endlog();
        return true;
      } else {
        // conversion failed
        RTT::log(RTT::Warning) <<
          "Property " <<
          getOwner()->provides()->getName() << "." << property_name <<
          " failed to convert from ROS parameter " << param_name << RTT::endlog();
        return false;
      }
    } catch (const RTT::internal::bad_assignment& /*ba*/) {
      RTT::log(RTT::Error) <<
        "Failed to convert  " <<
        getOwner()->provides()->getName() << "." << property_name <<
        " from ROS parameter " <<
        param_name << RTT::endlog();
      return false;
    }
    return false;
  }
}

bool Params::storeProperty(const std::string& property_name, const std::string& param_name)
{
  RTT::Logger::In in(getName());

  const RTT::base::PropertyBase * prop = getOwner()->properties()->getProperty(property_name);
  if (nullptr != prop) {
    RTT::log(RTT::Debug) <<
      getOwner()->provides()->getName() <<
      " has a property " << property_name << RTT::endlog();
  } else {
    RTT::log(RTT::Error) <<
      getOwner()->provides()->getName() <<
      " has NO property " << property_name << RTT::endlog();
    for (const auto property : getOwner()->properties()->getProperties() ) {
      RTT::log(RTT::Debug) <<
        "Available property: " << property->getName() <<
        RTT::endlog();
    }
    return false;
  }

  RTT::base::DataSourceBase::shared_ptr property_bds = prop->getDataSource();
  RTT::internal::ValueDataSource<rclcpp::ParameterValue> param_vds;

  // TODO(spdintermodalics): check whether the conversion was successful!
  param_vds.update(property_bds.get());

  return setParameter(param_name, param_vds.get());
}

}  // namespace rtt_ros2_params
