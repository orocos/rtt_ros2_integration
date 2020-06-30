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
#include "rtt_ros2_node/rtt_ros2_node.hpp"
#include "rtt_ros2_rclcpp_typekit/ros2_parameter_value_type.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "rtt/internal/GlobalService.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

namespace rtt_ros2_params {

Params::Params(RTT::TaskContext *owner)
  : RTT::Service("Params", owner) {
  RTT::log(RTT::Info) << "[" << getName() << "] Parameter service instantiated! in " << getName() << RTT::endlog();

  RTT::log(RTT::Debug) << "[" << getName() << "] Name of the owner: " << owner->getName() << getName() << RTT::endlog();
  const auto check = owner->provides()->hasService("ros");
  RTT::log(RTT::Debug) << "[" << getName() << "] ROS2 exists: " << check << RTT::endlog();

  this->doc("RTT Service for synchronizing ROS2 parameters with the properties of a corresponding RTT component");

  addOperation("getParameter", &Params::getParameter, this, RTT::ClientThread)
    .doc("Gets a parameter from the node parameter server")
    .arg("name", "Name of the parameter to retrieve");
    //.arg("namespace", "Node scope to retrieve the parameter from, e.g.: \"\" (locally), \"~\" (private) \"/\" (absolute)");

  addOperation("setParameter", &Params::setParameter, this, RTT::ClientThread)
    .doc("Sets a parameter to the node parameter server")
    .arg("name", "Name of the parameter to retrieve")
    .arg("value", "The value of the parameter to set");

  addOperation("loadProperty", &Params::loadProperty, this, RTT::ClientThread)
    .doc("Loads a parameter from the node parameter server into a Property")
    .arg("parameter_name", "Name of the parameter to retrieve")
    .arg("property_name", "Name of the property to load into");

  addOperation("storeProperty", &Params::storeProperty, this, RTT::ClientThread)
    .doc("Stores a property into a node parameter server")
    .arg("property_name", "Name of the property to store")
    .arg("parameter_name", "Name of the parameter to store into");
}

Params::~Params() {

}

bool Params::check_ros2_node_in_component() {
  return getOwner()->provides()->hasService("Node");
}

bool Params::check_ros2_node_in_global() {
  return RTT::internal::GlobalService::Instance()->hasService("ros");
}

rclcpp::ParameterValue Params::getParameter(const std::string param_name) {
  RTT::log(RTT::Debug) << "[" << getName() << "] Retrieving the parameter \"" << param_name << "\"" << RTT::endlog();

  rclcpp::ParameterValue paramvalue;

  rclcpp::Node::SharedPtr rosnode;
  if (get_ros2_node(rosnode)) {
    rosnode->get_parameter(param_name, paramvalue);
  } else {
    RTT::log(RTT::Error) << "[" << getName() << "] The ROS2 node doesn't exist, no parameter can be retrieved. Import rtt_ros2_node first." << RTT::endlog();
  }

  return paramvalue;
}

bool Params::setParameter(const std::string param_name, const rclcpp::ParameterValue paramvalue) {
  RTT::log(RTT::Debug) << "[" << getName() << "] Setting the parameter \"" << param_name << "\"" << RTT::endlog();
  
  rclcpp::Node::SharedPtr rosnode;
  if (get_ros2_node(rosnode)) {
    rcl_interfaces::msg::SetParametersResult ret;
    if (rosnode->has_parameter(param_name)) {
      // Update the parameter
      try {
        ret = rosnode->set_parameter(rclcpp::Parameter(param_name, paramvalue));
      } catch (std::exception e) {
        RTT::log(RTT::Error) << "[" << getName() << "] The parameter failed to be set, reason: " << e.what() << RTT::endlog();
      }
    } else {
      // Create new parameter
      // rosnode->set_parameter_if_not_set(param_name, paramvalue); // This is obsolete, found in API b3
      RTT::log(RTT::Warning) << "[" << getName() << "] The parameter did not exist, creating one" << RTT::endlog();
      try {
        rosnode->declare_parameter(param_name);
        ret = rosnode->set_parameter(rclcpp::Parameter(param_name, paramvalue));
      } catch (std::exception e) {
        RTT::log(RTT::Error) << "[" << getName() << "] The parameter failed to be set, reason: " << e.what() << RTT::endlog();
      }
    }
    if (!ret.successful) {
      RTT::log(RTT::Error) << "[" << getName() << "] The parameter " << param_name << " couldn't be set with reason: " << ret.reason << RTT::endlog();
      return false;
    }
  } else {
    RTT::log(RTT::Error) << "[" << getName() << "] The ROS2 node doesn't exist, no parameter can be set. Import rtt_ros2_node first and use the \"ros2_node\" service." << RTT::endlog();
    return false;
  }
  return true;
}

bool Params::loadProperty(const std::string param_name, const std::string property_name) {

  rclcpp::ParameterValue paramvalue = getParameter(param_name);
  if (rclcpp::PARAMETER_NOT_SET == paramvalue.get_type()) {
    RTT::log(RTT::Error) << "[" << getName() << "] The parameter \"" << param_name << "\" couldn't be retrieved" << RTT::endlog();
    return false;
  }

  if (nullptr != getOwner()->properties()->getProperty(property_name)) {
    RTT::log(RTT::Debug) << "[" << getName() << "] " << getOwner()->provides()->getName() << " has a property " << property_name << RTT::endlog();
  } else {
    RTT::log(RTT::Info) << "[" << getName() << "] " << getOwner()->provides()->getName() << " has NO property " << property_name << RTT::endlog();
    for (const auto property : getOwner()->properties()->getProperties() ) {
      RTT::log(RTT::Debug) << "[" << getName() << "] Available property: " << property->getName() << RTT::endlog();
    }
  }

  auto prop = getOwner()->properties()->getProperty(property_name);
  RTT::Property<rclcpp::ParameterValue>* prop_paramvalue = dynamic_cast<RTT::Property<rclcpp::ParameterValue>*>(prop);
  if (nullptr == prop) {
    // Try to find it among orphan parameters (previously loaded, without a component interface)
    if (orphan_params_.find(property_name) == orphan_params_.end()) {
      // When totally new, then create an orphan property, i.e. owned by the ros2_params service
      orphan_params_[property_name] = paramvalue;
      this->addProperty(property_name, orphan_params_[property_name])
        .doc("Property loaded from ROS2 param: " + param_name);
      prop = getProperty(property_name);
    } else {
      orphan_params_[property_name] = paramvalue;
      prop = getProperty(property_name);
    }
    return true;
  } else {
    // The property exists on the Owner's interface
    RTT::log(RTT::Debug) << "[" << getName() << "] The property " << property_name << " existed" << RTT::endlog();
    if (nullptr == prop_paramvalue) {
      RTT::log(RTT::Debug) << "[" << getName() << "] But " << property_name << " cannot be casted into rclcpp::ParamValue" << RTT::endlog();

      const RTT::base::DataSourceBase::shared_ptr property_ds = prop->getDataSource();
      rtt_ros2_rclcpp_typekit::ParameterValueTypeInfo param_typeinfo;
      const RTT::internal::ValueDataSource<rclcpp::ParameterValue> param_vds(paramvalue);
      RTT::internal::ReferenceDataSource<rclcpp::ParameterValue> param_rds = RTT::internal::ReferenceDataSource<rclcpp::ParameterValue>(paramvalue);
      param_rds.ref();

      try {
        if (property_ds->update(&param_rds)) {
          // conversion successful
          RTT::log(RTT::Debug) << "[" << getName() << "] Property " << getOwner()->provides()->getName() << "." << property_name << " loaded successfully from ROS2 parameter " << param_name << RTT::endlog();
          return true;
        } else {
          // conversion failed
          RTT::log(RTT::Warning) << "[" << getName() << "] Property " << getOwner()->provides()->getName() << "." << property_name << " failed to convert from ROS2 parameter " << param_name << RTT::endlog();
          return false;
        }
      } catch (std::exception e) {
        RTT::log(RTT::Error) << "The property " << getOwner()->provides()->getName() << "." << property_name << " could not be converted from ROS2 parameter " << param_name << ". Exception caught: " << e.what() << RTT::endlog();
        return false;
      }
      return false;
    } else {
      // This is the rare case in which the Property was already of type rclcpp::ParameterValue
      prop_paramvalue->set(paramvalue);
    }
    return true;
  }
  return false;
}

bool Params::storeProperty(const std::string property_name, const std::string param_name) {
  const RTT::base::PropertyBase* prop = getOwner()->properties()->getProperty(property_name);
  if (nullptr != prop) {
    RTT::log(RTT::Debug) << "[" << getName() << "] " << getOwner()->provides()->getName() << " has a property " << property_name << RTT::endlog();
  } else {
    RTT::log(RTT::Error) << "[" << getName() << "] " << getOwner()->provides()->getName() << " has NO property " << property_name << RTT::endlog();
    for (const auto property : getOwner()->properties()->getProperties() ) {
      RTT::log(RTT::Debug) << "[" << getName() << "] Available property: " << property->getName() << RTT::endlog();
    }
    return false;
  }

  rclcpp::ParameterValue paramvalue;
  RTT::base::DataSourceBase::shared_ptr property_bds = prop->getDataSource();
  RTT::internal::ValueDataSource<rclcpp::ParameterValue> param_vds(paramvalue);
  param_vds.update(&(*property_bds));

  paramvalue = param_vds.get();
  return setParameter(param_name, paramvalue);
}

// Helper function to get the ROS2 node
bool Params::get_ros2_node(rclcpp::Node::SharedPtr &node_ptr) {
  // We won't need this anymore, since the node is already provided by rtt_ros2_node::getNode()
  // Only here now for debugging reasons
  if (check_ros2_node_in_component()) {
    RTT::log(RTT::Debug) << "[" << getName() << "] Using the component service for ROS2" << RTT::endlog();
  } else if (check_ros2_node_in_global()) {
    RTT::log(RTT::Debug) << "[" << getName() << "] Using the global service for ROS2" << RTT::endlog();
  } else {
    RTT::log(RTT::Warning) << "[" << getName() << "] No ROS2 node was found within Orocos, please import \"rtt_ros2_node\" and use the \"ros2_node\" service" << RTT::endlog();
  }

  node_ptr = nullptr;
  if (RTT::internal::GlobalService::Instance()->hasService("ros")) {
    node_ptr = rtt_ros2_node::getNode(getOwner());
  }
  return (nullptr != node_ptr);
}

} // namespace rtt_ros2_params