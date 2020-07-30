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

#include <map>
#include <string>
#include <vector>

#include "rcpputils/split.hpp"

#include "rtt/internal/GlobalService.hpp"
#include "rtt/plugin/ServicePlugin.hpp"
#include "rtt/Service.hpp"
#include "rtt/TaskContext.hpp"

#include "rtt_ros2_services/rtt_ros2_services_registry.hpp"
#include "rtt_ros2_services/rtt_ros2_services_proxy.hpp"

namespace rtt_ros2_services
{

/**
 * The globally loadable ROS service.
 */
class RosServiceService : public RTT::Service
{
public:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  explicit RosServiceService(RTT::TaskContext * owner)
  : Service("rosservice", owner)
  {
    if (owner) {
      this->doc(
        "RTT Service for connecting the operations of " + owner->getName() +
        " to ROS service clients and servers.");
    }

    this->addOperation("connect", &RosServiceService::connect, this)
    .doc(
      "Connects an RTT operation or operation caller to an associated ROS service server or "
      "client.")
    .arg(
      "operation_name",
      "The RTT operation name (like \"some_provided_service.another.operation\").")
    .arg("service_name", "The ROS service name (like \"/my_robot/ns/some_service\").")
    .arg("service_type", "The ROS service type (like \"std_srvs/srv/Empty\").");
    this->addOperation("disconnect", &RosServiceService::disconnect, this)
    .doc(
      "Disconnects an RTT operation or operation caller from an associated ROS service server or "
      "client.")
    .arg("service_name", "The ROS service name (like \"/my_robot/ns/some_service\").");
    this->addOperation("disconnectAll", &RosServiceService::disconnectAll, this)
    .doc(
      "Disconnects all RTT operations and operation callers from associated ROS service servers or "
      "clients.");

    // Get the global ros service registry
    rosservice_registry_ = RosServiceRegistry::Instance();
  }

  ~RosServiceService()
  {
    disconnectAll();
  }

  //! Get an RTT operation caller from a string identifier
  RTT::base::OperationCallerBaseInvoker * get_owner_operation_caller(const std::string & rtt_uri)
  {
    // Split up the service uri
    constexpr bool kSkipEmpty = true;
    std::vector<std::string> rtt_uri_tokens = rcpputils::split(rtt_uri, '.', kSkipEmpty);

    // Make sure the uri has at least one token
    if (rtt_uri_tokens.size() < 1) {
      return nullptr;
    }

    // Iterate through the tokens except for the last one (the operation name)
    boost::shared_ptr<RTT::ServiceRequester> required = this->getOwner()->requires();
    for (std::vector<std::string>::iterator it = rtt_uri_tokens.begin();
      it + 1 != rtt_uri_tokens.end();
      ++it)
    {
      if (required->requiresService(*it)) {
        required = required->requires(*it);
      } else {
        return nullptr;
      }
    }

    // Get the operation caller
    return required->getOperationCaller(rtt_uri_tokens.back());
  }

  //! Get an RTT operation from a string identifier
  RTT::OperationInterfacePart * get_owner_operation(const std::string & rtt_uri)
  {
    // Split up the service uri
    constexpr bool kSkipEmpty = true;
    std::vector<std::string> rtt_uri_tokens = rcpputils::split(rtt_uri, '.', kSkipEmpty);

    // Make sure the uri has at least one token
    if (rtt_uri_tokens.size() < 1) {
      return nullptr;
    }

    // Iterate through the tokens except for the last one (the operation name)
    RTT::Service::shared_ptr provided = this->getOwner()->provides();
    for (std::vector<std::string>::iterator it = rtt_uri_tokens.begin();
      it + 1 != rtt_uri_tokens.end();
      ++it)
    {
      if (provided->hasService(*it)) {
        provided = provided->provides(*it);
      } else {
        return nullptr;
      }
    }

    // Get the operation
    return provided->getOperation(rtt_uri_tokens.back());
  }

  /** \brief Connect an RTT operation or operation caller to a ROS service
   * server or service client.
   */
  bool connect(
    const std::string & rtt_operation_name,
    const std::string & ros_service_name,
    const std::string & ros_service_type)
  {
    // Make sure the factory for this service type exists
    if (!rosservice_registry_->hasServiceFactory(ros_service_type)) {
      RTT::log(RTT::Error) << "Unknown service type '" << ros_service_type << "'" << RTT::endlog();
      return false;
    }

    // Check if the operation is required by the owner
    RTT::base::OperationCallerBaseInvoker *
      operation_caller = this->get_owner_operation_caller(rtt_operation_name);

    if (operation_caller) {
      // Check if the client proxy already exists
      if (client_proxies_.find(ros_service_name) == client_proxies_.end()) {
        // Create a new client proxy
        client_proxies_[ros_service_name] =
          rosservice_registry_->getServiceFactory(ros_service_type)->
          create_client_proxy(ros_service_name);
      }

      // Associate an RTT operation caller with a ROS service client
      if (!client_proxies_[ros_service_name]->connect(this->getOwner(), operation_caller)) {
        RTT::log(RTT::Error) << "Could not connect OperationCaller '" << rtt_operation_name <<
          "' to ROS service client '" << ros_service_name << "'" << RTT::endlog();
        return false;
      }
      return true;
    }

    // Check if the operation is provided by the owner
    RTT::OperationInterfacePart *
      operation = this->get_owner_operation(rtt_operation_name);

    if (operation) {
      // Check if the server proxy already exists
      if (server_proxies_.find(ros_service_name) == server_proxies_.end()) {
        // Create a new server proxy
        server_proxies_[ros_service_name] =
          rosservice_registry_->getServiceFactory(ros_service_type)->
          create_server_proxy(ros_service_name);
      }

      // Associate an RTT operation with a ROS service server
      if (!server_proxies_[ros_service_name]->connect(this->getOwner(), operation)) {
        RTT::log(RTT::Error) << "Could not connect Operation '" << rtt_operation_name <<
          "' to ROS service server '" << ros_service_name << "'" << RTT::endlog();
        return false;
      }
      return true;
    }

    RTT::log(RTT::Error) <<
      "No such Operation or OperationCaller '" << rtt_operation_name <<
      "' in '" << getOwner()->getName() << "'" << RTT::endlog();
    return false;
  }

  bool disconnect(const std::string & ros_service_name)
  {
    bool found = false;

    // Cleanup ROS service or client named ros_service_name
    auto iter_s = server_proxies_.find(ros_service_name);
    if (iter_s != server_proxies_.end()) {
      server_proxies_.erase(iter_s);
      found = true;
    }

    auto iter_c = client_proxies_.find(ros_service_name);
    if (iter_c != client_proxies_.end()) {
      client_proxies_.erase(iter_c);
      found = true;
    }

    return found;
  }

  void disconnectAll()
  {
    // Cleanup registered ROS services and clients
    server_proxies_.clear();
    client_proxies_.clear();
  }

  RosServiceRegistryPtr rosservice_registry_;

  std::map<std::string, RosServiceServerProxyBasePtr> server_proxies_;
  std::map<std::string, RosServiceClientProxyBasePtr> client_proxies_;
};

}  // namespace rtt_ros2_services

ORO_SERVICE_NAMED_PLUGIN(rtt_ros2_services::RosServiceService, "rosservice")
