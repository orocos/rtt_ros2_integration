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

#ifndef OROCOS__RTT_ROS2_SERVICES__RTT_ROS2_SERVICES_PROXY_HPP_
#define OROCOS__RTT_ROS2_SERVICES__RTT_ROS2_SERVICES_PROXY_HPP_

#include <memory>
#include <map>
#include <string>
#include <type_traits>
#include <utility>

#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"

#include "rtt/internal/GlobalEngine.hpp"
#include "rtt/Operation.hpp"
#include "rtt/OperationCaller.hpp"
#include "rtt/TaskContext.hpp"

#include "rtt_ros2_node/rtt_ros2_node.hpp"

namespace rtt_ros2_services
{

//! Abstract ROS service proxy
class RosServiceProxyBase
{
public:
  explicit RosServiceProxyBase(const std::string & service_name);
  virtual ~RosServiceProxyBase();

  //! Get the name of the ROS service
  const std::string & getServiceName() const {return service_name_;}

protected:
  //! ROS service name
  std::string service_name_;
};
using RosServiceProxyBasePtr = std::unique_ptr<RosServiceProxyBase>;

//! Abstract ROS service service proxy
class RosServiceServerProxyBase : public RosServiceProxyBase
{
public:
  explicit RosServiceServerProxyBase(const std::string & service_name);

  //! Connect an RTT Operation to this ROS service server
  virtual bool connect(RTT::TaskContext * owner, RTT::OperationInterfacePart * operation) = 0;
};
using RosServiceServerProxyBasePtr = std::unique_ptr<RosServiceServerProxyBase>;

template<class ServiceT>
class RosServiceServerOperationCallerBase
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using Ptr = std::unique_ptr<RosServiceServerOperationCallerBase<ServiceT>>;
  virtual ~RosServiceServerOperationCallerBase() {}
  virtual void dispatch(
    std::shared_ptr<rmw_request_id_t>,
    std::shared_ptr<Request>,
    std::shared_ptr<Response>) const = 0;
};

template<class ServiceT, int variant = 0>
struct RosServiceServerOperationCallerWrapper
{
  using ProxyOperationCallerType = void;
};

// Default implementation of an OperationCaller that forwards ROS service calls to Orocos operations
// that have the default void(Request&, Response&) signature. You can add more variants of this
// class to add support for custom operation types.
//
// See package rtt_ros2_std_srvs for an example.
//
template<class ServiceT>
struct RosServiceServerOperationCallerWrapper<ServiceT, 0>
{
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using Signature = void (Request &, Response &);
  using ProxyOperationCallerType = RTT::OperationCaller<Signature>;
  template<typename Callable>
  static void dispatch(
    Callable & call,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<Request> request,
    std::shared_ptr<Response> response)
  {
    call(*request, *response);
  }
};

template<class ServiceT, int variant = 0>
class RosServiceServerOperationCaller : public RosServiceServerOperationCallerBase<ServiceT>
{
public:
  using Ptr = typename RosServiceServerOperationCallerBase<ServiceT>::Ptr;

  //! The wrapper type for this variant
  using Wrapper = RosServiceServerOperationCallerWrapper<ServiceT, variant>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  //! Default operation caller for a ROS service server proxy
  using ProxyOperationCallerType = typename Wrapper::ProxyOperationCallerType;
  using ProxyOperationCallerTypePtr = std::unique_ptr<ProxyOperationCallerType>;

  static Ptr connect(RTT::OperationInterfacePart * operation);

  void dispatch(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<Request> request,
    std::shared_ptr<Response> response) const override
  {
    // Check if the operation caller is ready, and then call it.
    if (!proxy_operation_caller_->ready()) {
      throw std::runtime_error("OperationCaller not ready");
    }
    Wrapper::dispatch(
      *proxy_operation_caller_,
      std::move(request_header),
      std::move(request),
      std::move(response));
  }

private:
  explicit RosServiceServerOperationCaller(std::unique_ptr<ProxyOperationCallerType> impl)
  : proxy_operation_caller_(std::move(impl)) {}

  ProxyOperationCallerTypePtr proxy_operation_caller_;
};

namespace
{

template<class ServiceT, int variant, typename Enabled = void>
struct RosServiceServerOperationCallerWrapperNextVariant
{
  using Ptr = typename RosServiceServerOperationCallerBase<ServiceT>::Ptr;
  static Ptr connect(RTT::OperationInterfacePart *) {return Ptr();}
};

template<class ServiceT, int variant>
struct RosServiceServerOperationCallerWrapperNextVariant<
  ServiceT, variant, typename std::enable_if<!std::is_void<
    typename RosServiceServerOperationCallerWrapper<ServiceT, variant + 1>
    ::ProxyOperationCallerType>::value>::type>
{
  using Ptr = typename RosServiceServerOperationCallerBase<ServiceT>::Ptr;
  static Ptr connect(RTT::OperationInterfacePart * operation)
  {
    return RosServiceServerOperationCaller<ServiceT, variant + 1>::connect(operation);
  }
};

}  // namespace

template<class ServiceT, int variant>
typename RosServiceServerOperationCaller<ServiceT, variant>::Ptr
RosServiceServerOperationCaller<ServiceT, variant>::connect(RTT::OperationInterfacePart * operation)
{
  ProxyOperationCallerTypePtr proxy_operation_caller =
    std::make_unique<ProxyOperationCallerType>(
    operation->getLocalOperation(), RTT::internal::GlobalEngine::Instance());
  if (proxy_operation_caller->ready()) {
    return Ptr(
      new RosServiceServerOperationCaller<ServiceT, variant>(
        std::move(proxy_operation_caller)));
  }
  return RosServiceServerOperationCallerWrapperNextVariant<ServiceT, variant>::connect(operation);
}

template<class ServiceT>
class RosServiceServerProxy : public RosServiceServerProxyBase
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  /** \brief Construct a ROS service server and associate it with an Orocos
   * task's required interface and operation caller.
   */
  explicit RosServiceServerProxy(const std::string & service_name)
  : RosServiceServerProxyBase(service_name)
  {}

  virtual ~RosServiceServerProxy()
  {
    // Clean-up advertised ROS services
    server_.reset();
  }

  // Construct the ROS service server
  bool connect(RTT::TaskContext * owner, RTT::OperationInterfacePart * operation) override
  {
    const auto node = rtt_ros2_node::getNode(owner);
    if (node == nullptr) {return false;}

    impl_ = RosServiceServerOperationCaller<ServiceT>::connect(operation);
    if (impl_ == nullptr) {
      return false;
    }

    server_ = node->create_service<ServiceT>(
      service_name_,
      [this](
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<Request> request,
        std::shared_ptr<Response> response) {
        impl_->dispatch(std::move(request_header), std::move(request), std::move(response));
      });

    return true;
  }

private:
  //! The underlying RTT operation caller
  std::unique_ptr<RosServiceServerOperationCallerBase<ServiceT>> impl_;

  //! The ROS service server
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
};

//! Abstract ROS service Client proxy
class RosServiceClientProxyBase : public RosServiceProxyBase
{
public:
  explicit RosServiceClientProxyBase(const std::string & service_name);

  //! Connect an operation caller with this proxy
  virtual bool connect(
    RTT::TaskContext * owner,
    RTT::base::OperationCallerBaseInvoker * operation_caller) = 0;
};
using RosServiceClientProxyBasePtr = std::unique_ptr<RosServiceClientProxyBase>;

template<class ServiceT>
class RosServiceClientProxy : public RosServiceClientProxyBase
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using Signature = void (Request &, Response &);

  explicit RosServiceClientProxy(const std::string & service_name)
  : RosServiceClientProxyBase(service_name)
  {
  }

  bool connect(
    RTT::TaskContext * owner,
    RTT::base::OperationCallerBaseInvoker * operation_caller) override
  {
    const auto node = rtt_ros2_node::getNode(owner);
    if (node == nullptr) {return false;}

    // Create ROS service client
    client_ = node->create_client<ServiceT>(service_name_);
    boost::function<Signature> func = [this](Request & request, Response & response) -> void {
        // Make sure the ROS service client exists and then call it (blocking)
        const auto client = client_;
        if (!client) {return;}

        // TODO(meyerj): std::make_shared<Request> is not real-time safe and executed in the thread
        // that calls the operation.
        auto result_future = client->async_send_request(std::make_shared<Request>(request));
        response = *(result_future.get());
      };

    return operation_caller->setImplementation(
      boost::make_shared<RTT::internal::LocalOperationCaller<Signature>>(
        std::move(func), nullptr, nullptr, RTT::ClientThread),
      owner->engine());
  }

private:
  //! The ROS service client
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

//! Abstract factory for ROS service proxy factories
class RosServiceProxyFactoryBase
{
public:
  explicit RosServiceProxyFactoryBase(const std::string & service_type);

  //! Get the ROS service type
  const std::string & getType() const {return service_type_;}

  //! Get a proxy to a ROS service client
  virtual RosServiceClientProxyBasePtr create_client_proxy(
    const std::string & service_name) const = 0;
  //! Get a proxy to a ROS service server
  virtual RosServiceServerProxyBasePtr create_server_proxy(
    const std::string & service_name) const = 0;

private:
  std::string service_type_;
};

template<class ServiceT>
class RosServiceProxyFactory : public RosServiceProxyFactoryBase
{
public:
  explicit RosServiceProxyFactory(const std::string & service_type)
  : RosServiceProxyFactoryBase(service_type) {}

  RosServiceClientProxyBasePtr create_client_proxy(const std::string & service_name) const override
  {
    return std::make_unique<RosServiceClientProxy<ServiceT>>(service_name);
  }

  RosServiceServerProxyBasePtr create_server_proxy(const std::string & service_name) const override
  {
    return std::make_unique<RosServiceServerProxy<ServiceT>>(service_name);
  }
};

}  // namespace rtt_ros2_services

#endif  // OROCOS__RTT_ROS2_SERVICES__RTT_ROS2_SERVICES_PROXY_HPP_
