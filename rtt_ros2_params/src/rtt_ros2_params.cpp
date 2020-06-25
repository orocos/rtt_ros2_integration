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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

#include "rtt/internal/GlobalService.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

namespace rtt_ros2_params {

Params::Params(RTT::TaskContext *owner)
  : RTT::Service("Params", owner) {
  std::cout << "Parameter service instantiated! in " << getName() << std::endl;

  std::cout << "Name of the owner: " << owner->getName() << std::endl;
  // getName() returns the name of the Service, NOT the TC owner (the component it belongs to)
  // owner->getName() fails
  // owner->
  const auto check = owner->provides()->hasService("ros");
  std::cout << check << std::endl;

  this->doc("RTT Service for synchronizing ROS2 parameters with the properties of a corresponding RTT component");
  this->setValue(new RTT::Constant<int>("TheAnswer", 42));





  // base::DataSourceBase::shared_ptr ds;
  // if (!ds) {
  //   sresult << "(null)";
  //   return;
  // }
  // ds->reset();
  // ds->evaluate();
  // DataSource<RTT::PropertyBag>* dspbag = DataSource<RTT::PropertyBag>::narrow(ds.get());
  // RTT::PropertyBag bag( dspbag->get() );


  // Create a Property<> wrapper around the propertybag
  // RTT::PropertyBag *properties = service->properties();
  // RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr datasource(new RTT::internal::ReferenceDataSource<RTT::PropertyBag>(*properties));
  // RTT::Property<RTT::PropertyBag> prop(this->getOwner()->getName(),"",datasource);

  // addOperation("check_ros2_node", &Params::check_ros2_node, this, RTT::ClientThread);
  addOperation("getParameter", &Params::getParameter, this, RTT::ClientThread)
    .doc("Gets a parameter from the parameter server")
    .arg("name", "Name of the parameter to retrieve");
    //.arg("namespace", "Node scope to retrieve the parameter from, e.g.: \"\" (locally), \"~\" (private) \"/\" (absolute)");

  addOperation("loadProperty", &Params::loadProperty, this, RTT::ClientThread)
    .doc("Loads a parameter from the parameter server into a Property")
    .arg("parameter_name", "Name of the parameter to retrieve")
    .arg("property_name", "Name of the property to load into");
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
  RTT::log(RTT::Info) << "[" << getName() << "] Retrieving the parameter \"" << param_name << "\"" << RTT::endlog();

  // We won't need this anymore, since the node is already provided by rtt_ros2_node::getNode()
  // Only here now for debugging reasons
  if (check_ros2_node_in_component()) {
    RTT::log(RTT::Info) << "[" << getName() << "] Using the component service for ROS2" << RTT::endlog();
  } else if (check_ros2_node_in_global()) {
    RTT::log(RTT::Info) << "[" << getName() << "] Using the global service for ROS2" << RTT::endlog();
  } else {
    RTT::log(RTT::Warning) << "[" << getName() << "] No ROS2 node was found within Orocos, please import \"rtt_ros2_node\"" << RTT::endlog();
  }

  rclcpp::Node::SharedPtr rosnode = nullptr;
  if (RTT::internal::GlobalService::Instance()->hasService("ros")) {
    // const auto node = boost::dynamic_pointer_cast<rtt_ros2_node::Node>(RTT::internal::GlobalService::Instance()->provides()->getService("ros"));
    // RTT::log(RTT::Info) << "[" << getName() << "] Global service ROS2 named " << (nullptr != node ? node->getName() : "NULL") << RTT::endlog();
    rosnode = rtt_ros2_node::getNode(getOwner());
  }

  rclcpp::ParameterValue paramvalue;

  if (nullptr != rosnode) {
    rosnode->get_parameter(param_name, paramvalue);
  } else {
    RTT::log(RTT::Warning) << "[" << getName() << "] The ROS2 node doesn't exist, no parameter can be retrieved." << RTT::endlog();
  }

  return paramvalue;
}

bool Params::loadProperty(const std::string param_name, const std::string property_name) {

  const rclcpp::ParameterValue paramvalue = getParameter(param_name);
  if (rclcpp::PARAMETER_NOT_SET == paramvalue.get_type()) {
    RTT::log(RTT::Warning) << "[" << getName() << "] The parameter \"" << param_name << "\" couldn't be retrieved" << RTT::endlog();
    return false;
  }

  if (nullptr != getOwner()->properties()->getProperty(property_name)) {
    RTT::log(RTT::Info) << "[" << getName() << "] " << getOwner()->provides()->getName() << " has a property " << property_name << RTT::endlog();
  } else {
    RTT::log(RTT::Info) << "[" << getName() << "] " << getOwner()->provides()->getName() << " has NO property " << property_name << RTT::endlog();
    for (const auto property : getOwner()->properties()->getProperties() ) {
      RTT::log(RTT::Info) << property->getName() << RTT::endlog();
    }
  }

  // prop will be RTT::base::PropertyBase*
  // I want to convert and transfer the content of a RTT::Property<rclcpp::ParameterValue>* into RTT::base::PropertyBase*
  auto prop = getOwner()->properties()->getProperty(property_name);
  RTT::Property<rclcpp::ParameterValue>* prop_paramvalue = dynamic_cast<RTT::Property<rclcpp::ParameterValue>*>(prop);
  if (nullptr == prop) {
    // Try to find it among orphan parameters (previously loaded, without a component interface)
    if (orphan_params_.find(property_name) == orphan_params_.end()) {
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
    RTT::log(RTT::Info) << "[" << getName() << "] The property " << property_name << " existed" << RTT::endlog();
    if (nullptr == prop_paramvalue) {
      RTT::log(RTT::Info) << "[" << getName() << "] But cannot be casted into rclcpp::ParamValue" << RTT::endlog();
      return false;
    } else {
      prop_paramvalue->set(paramvalue);
    }
    return true;
  }

  return false;
}

} // namespace rtt_ros2_params