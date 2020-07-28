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

#include <chrono>
#include <memory>
#include <utility>

#include "rtt/RTT.hpp"
#include "rtt/internal/LocalOperationCaller.hpp"
#include "rtt/os/startstop.h"

#include "rtt_ros2/rtt_ros2.hpp"
#include "rtt_ros2_node/rtt_ros2_node.hpp"
#include "rtt_ros2_services/rosservice.hpp"
#include "rtt_ros2_services/rtt_ros2_services_registry.hpp"

#include "test_msgs/srv/arrays.hpp"
#include "test_msgs/srv/basic_types.hpp"
#include "test_msgs/srv/empty.hpp"

#include "gtest/gtest.h"

/* True if the version of test_msgs is at least major.minor.patch */
// (from https://github.com/ros2/rmw_cyclonedds/pull/51)
#define test_msgs_VERSION_GTE(major, minor, patch) ( \
    (major < test_msgs_VERSION_MAJOR) ? true \
    : (major > test_msgs_VERSION_MAJOR) ? false \
    : (minor < test_msgs_VERSION_MINOR) ? true \
    : (minor > test_msgs_VERSION_MINOR) ? false \
    : (patch < test_msgs_VERSION_PATCH) ? true \
    : (patch > test_msgs_VERSION_PATCH) ? false \
    : true)

class TestRosServiceEnvironment
  : public ::testing::Environment
{
  void SetUp() override
  {
    EXPECT_TRUE(rtt_ros2::import("rtt_ros2_test_msgs"));
  }
};

class TestRosService
  : public RTT::TaskContext,
  public ::testing::Test
{
public:
  using Arrays = test_msgs::srv::Arrays;
  using ArraysOperationSignature = void (Arrays::Request &, Arrays::Response &);
  using BasicTypes = test_msgs::srv::BasicTypes;
  using BasicTypesOperationSignature = void (BasicTypes::Request &, BasicTypes::Response &);
  using Empty = test_msgs::srv::Empty;
  using EmptyOperationSignature0 = void (Empty::Request &, Empty::Response &);
  using EmptyOperationSignature1 = void (void);

  TestRosService()
  : RTT::TaskContext("TestRosService"),
    arrays_op("arrays"),
    basic_types_op("basic_types"),
    empty_op0("empty0"),
    empty_op1("empty1"),
    arrays_caller("arrays_caller"),
    basic_types_caller("basic_types_caller"),
    empty_caller("empty_caller")
  {
    this->provides()->addOperation(arrays_op);
    this->provides()->addOperation(basic_types_op);
    this->provides()->addOperation(empty_op0);
    this->provides()->addOperation(empty_op1);
    this->requires()->addOperationCaller(arrays_caller);
    this->requires()->addOperationCaller(basic_types_caller);
    this->requires()->addOperationCaller(empty_caller);
  }

  void SetUp()
  {
    // Create a component-local ROS node
    ASSERT_TRUE(loadService("rosnode"));
    ASSERT_TRUE(node = rtt_ros2_node::getNode(this));

    // Load service rosservice
    ASSERT_TRUE(rosservice = getProvider<rtt_ros2_services::RosService>("rosservice"));

    // Verify that the ROS service factories for test_msgs have been loaded
    auto rosservice_registry = rtt_ros2_services::RosServiceRegistry::Instance();
    ASSERT_TRUE(rosservice_registry->hasServiceFactory("test_msgs/srv/Arrays"));
    ASSERT_TRUE(rosservice_registry->hasServiceFactory("test_msgs/srv/BasicTypes"));
    ASSERT_TRUE(rosservice_registry->hasServiceFactory("test_msgs/srv/Empty"));
  }

protected:
  rtt_ros2_services::RosService::SharedPtr rosservice;
  rclcpp::Node::SharedPtr node;

  RTT::Operation<ArraysOperationSignature> arrays_op;
  RTT::Operation<BasicTypesOperationSignature> basic_types_op;
  RTT::Operation<EmptyOperationSignature0> empty_op0;
  RTT::Operation<EmptyOperationSignature1> empty_op1;

  RTT::OperationCaller<ArraysOperationSignature> arrays_caller;
  RTT::OperationCaller<BasicTypesOperationSignature> basic_types_caller;
  RTT::OperationCaller<EmptyOperationSignature0> empty_caller;
};

static constexpr auto kRequestTimeout = std::chrono::seconds(1);

TEST_F(TestRosService, ArraysOperation)
{
  bool operation_called = false;
  arrays_op.calls(
    boost::function<ArraysOperationSignature>(
      [&](Arrays::Request & request, Arrays::Response & response) -> void {
        RTT::log(RTT::Info) << "Operation 'arrays' called" << RTT::endlog();
        EXPECT_TRUE(this->engine()->isSelf());
        operation_called = true;
        response.string_values = request.string_values;
        response.string_values[1] = "SUCCESS";
      }), RTT::OwnThread);
  ASSERT_TRUE(arrays_op.ready());
  ASSERT_TRUE(rosservice->connect("arrays", "~/arrays", "test_msgs/srv/Arrays"));

  auto client = node->create_client<Arrays>("~/arrays");
  ASSERT_TRUE(client);
  auto request = std::make_shared<Arrays::Request>();
  request->string_values[0] = "TEST_REQUEST";
  auto response_future = client->async_send_request(std::move(request));
  ASSERT_EQ(response_future.wait_for(kRequestTimeout), std::future_status::ready);
  auto response = response_future.get();

  EXPECT_TRUE(operation_called);
  EXPECT_EQ(response->string_values[0], "TEST_REQUEST");
  EXPECT_EQ(response->string_values[1], "SUCCESS");
}

TEST_F(TestRosService, BasicTypesOperation)
{
  bool operation_called = false;
  basic_types_op.calls(
    boost::function<BasicTypesOperationSignature>(
      [&](BasicTypes::Request & request, BasicTypes::Response & response) -> void {
        RTT::log(RTT::Info) << "Operation 'basic_types' called" << RTT::endlog();
        EXPECT_TRUE(this->engine()->isSelf());
        operation_called = true;
        response.bool_value = (request.string_value == "TEST_REQUEST");
        response.string_value = "SUCCESS";
      }), RTT::OwnThread);
  ASSERT_TRUE(basic_types_op.ready());
  ASSERT_TRUE(rosservice->connect("basic_types", "~/basic_types", "test_msgs/srv/BasicTypes"));

  auto client = node->create_client<BasicTypes>("~/basic_types");
  ASSERT_TRUE(client);
  auto request = std::make_shared<BasicTypes::Request>();
  request->string_value = "TEST_REQUEST";
  auto response_future = client->async_send_request(std::move(request));
  ASSERT_EQ(response_future.wait_for(kRequestTimeout), std::future_status::ready);
  auto response = response_future.get();

  EXPECT_TRUE(operation_called);
  EXPECT_TRUE(response->bool_value);
  EXPECT_EQ(response->string_value, "SUCCESS");
}

TEST_F(TestRosService, EmptyOperation0)
{
  bool operation_called = false;
  empty_op0.calls(
    boost::function<EmptyOperationSignature0>(
      [&](Empty::Request & request, Empty::Response & response) -> void {
        RTT::log(RTT::Info) << "Operation 'empty' called" << RTT::endlog();
        EXPECT_TRUE(this->engine()->isSelf());
        operation_called = true;
      }), RTT::OwnThread);
  ASSERT_TRUE(empty_op0.ready());
  ASSERT_TRUE(rosservice->connect("empty0", "~/empty0", "test_msgs/srv/Empty"));

  auto client = node->create_client<Empty>("~/empty0");
  ASSERT_TRUE(client);
  auto request = std::make_shared<Empty::Request>();
  auto response_future = client->async_send_request(std::move(request));
  ASSERT_EQ(response_future.wait_for(kRequestTimeout), std::future_status::ready);
  auto response = response_future.get();

  EXPECT_TRUE(operation_called);
}

TEST_F(TestRosService, EmptyOperation1)
{
  bool operation_called = false;
  empty_op1.calls(
    boost::function<EmptyOperationSignature1>(
      [&]() -> void {
        RTT::log(RTT::Info) << "Operation 'empty' called" << RTT::endlog();
        EXPECT_TRUE(this->engine()->isSelf());
        operation_called = true;
      }), RTT::OwnThread);
  ASSERT_TRUE(empty_op1.ready());
  ASSERT_TRUE(rosservice->connect("empty1", "~/empty1", "test_msgs/srv/Empty"));

  auto client = node->create_client<Empty>("~/empty1");
  ASSERT_TRUE(client);
  auto request = std::make_shared<Empty::Request>();
  auto response_future = client->async_send_request(std::move(request));
  ASSERT_EQ(response_future.wait_for(kRequestTimeout), std::future_status::ready);
  auto response = response_future.get();

  EXPECT_TRUE(operation_called);
}

TEST_F(TestRosService, ArraysOperationCaller)
{
  bool service_called = false;
  auto service = node->create_service<Arrays>(
    "~/arrays",
    [&](const Arrays::Request::SharedPtr request, Arrays::Response::SharedPtr response) {
      RTT::log(RTT::Info) << "ROS service 'arrays' called" << RTT::endlog();
      service_called = true;
      response->string_values = request->string_values;
      response->string_values[1] = "SUCCESS";
    });
  ASSERT_TRUE(service);

  ASSERT_TRUE(rosservice->connect("arrays_caller", "~/arrays", "test_msgs/srv/Arrays"));
  ASSERT_TRUE(arrays_caller.ready());

  Arrays::Request request;
  Arrays::Response response;
  request.string_values[0] = "TEST_REQUEST";
  arrays_caller.call(request, response);

  EXPECT_TRUE(service_called);
  EXPECT_EQ(response.string_values[0], "TEST_REQUEST");
  EXPECT_EQ(response.string_values[1], "SUCCESS");
}

TEST_F(TestRosService, BasicTypesOperationCaller)
{
  bool service_called = false;
  auto service = node->create_service<BasicTypes>(
    "~/basic_types",
    [&](const BasicTypes::Request::SharedPtr request, BasicTypes::Response::SharedPtr response) {
      RTT::log(RTT::Info) << "ROS service 'basic_types' called" << RTT::endlog();
      service_called = true;
      response->bool_value = (request->string_value == "TEST_REQUEST");
      response->string_value = "SUCCESS";
    });
  ASSERT_TRUE(service);

  ASSERT_TRUE(
    rosservice->connect(
      "basic_types_caller", "~/basic_types", "test_msgs/srv/BasicTypes"));
  ASSERT_TRUE(basic_types_caller.ready());

  BasicTypes::Request request;
  BasicTypes::Response response;
  request.string_value = "TEST_REQUEST";
  basic_types_caller.call(request, response);

  EXPECT_TRUE(service_called);
  EXPECT_TRUE(response.bool_value);
  EXPECT_EQ(response.string_value, "SUCCESS");
}

TEST_F(TestRosService, EmptyOperationCaller)
{
  bool service_called = false;
  auto service = node->create_service<Empty>(
    "~/empty",
    [&](const Empty::Request::SharedPtr request, Empty::Response::SharedPtr response) {
      RTT::log(RTT::Info) << "ROS service 'empty' called" << RTT::endlog();
      service_called = true;
    });
  ASSERT_TRUE(service);

  ASSERT_TRUE(
    rosservice->connect(
      "empty_caller", "~/empty", "test_msgs/srv/Empty"));
  ASSERT_TRUE(empty_caller.ready());

  Empty::Request request;
  Empty::Response response;
  empty_caller.call(request, response);

  EXPECT_TRUE(service_called);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv) because this function
  // removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(new TestRosServiceEnvironment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
