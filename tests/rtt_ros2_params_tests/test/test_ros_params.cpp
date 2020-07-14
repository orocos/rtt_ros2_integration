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
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rtt/RTT.hpp"
#include "rtt/internal/GlobalService.hpp"
#include "rtt/os/startstop.h"

#include "rtt_ros2/rtt_ros2.hpp"
#include "rtt_ros2_node/rtt_ros2_node.hpp"
#include "rtt_ros2_params/rtt_ros2_params.hpp"

#include "gtest/gtest.h"

class TestRosParamsEnvironment
  : public ::testing::Environment
{
  void SetUp() override
  {
    EXPECT_TRUE(rtt_ros2::import("rtt_ros2_params"));
  }
};

class TestRosParams
  : public RTT::TaskContext,
  public ::testing::Test
{
public:
  TestRosParams()
  : RTT::TaskContext("TestRosParams")
  {
    this->addProperty("int_property", int_member_);
    this->addProperty("double_property", double_member_);
    this->addProperty("bool_property", bool_member_);
  }

protected:
  int int_member_ = 42;
  bool bool_member_ = true;
  double double_member_ = 3.14159;
  std::string string_member_ = std::string("answer to life and universe");
};

TEST_F(TestRosParams, TestGlobalNodeParams)
{
  // Create a global (process-wide) ROS node
  RTT::Service::shared_ptr global_ros =
    RTT::internal::GlobalService::Instance()->getService("ros");
  ASSERT_TRUE(global_ros);
  RTT::OperationCaller<bool()> create_node =
    global_ros->getOperation("create_node");
  ASSERT_TRUE(create_node.ready());
  ASSERT_TRUE(create_node());
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Create a global (process-wide) Parameter service
  // ASSERT_TRUE(RTT::internal::GlobalService::Instance()->provides("rosparam"));
  RTT::Service::shared_ptr global_params =
    global_ros->getService("Params");
  ASSERT_TRUE(nullptr != global_params);

  // Check set non-existant parameters
  RTT::OperationCaller<bool(std::string, rclcpp::ParameterValue)>
  setparam_operation = global_params->getOperation("setParameter");
  ASSERT_TRUE(setparam_operation.ready());
  EXPECT_FALSE(
    setparam_operation.call("fake_parameter", rclcpp::ParameterValue(42)));

  // Check get non-existant parameters
  RTT::OperationCaller<rclcpp::ParameterValue(std::string)>
  getparam_operation = global_params->getOperation("getParameter");
  ASSERT_TRUE(getparam_operation.ready());
  EXPECT_EQ(
    rclcpp::PARAMETER_NOT_SET,
    getparam_operation.call("fake_parameter").get_type()
  );

  // Check setting a parameter (INTEGER)
  rtt_ros2_node::getNode(this)->declare_parameter("int_parameter");
  EXPECT_TRUE(
    setparam_operation.call("int_parameter", rclcpp::ParameterValue(42)));

  // Check getting a parameter (INTEGER)
  rclcpp::ParameterValue new_int = getparam_operation.call("int_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_INTEGER, new_int.get_type());
  EXPECT_EQ(new_int.get<int>(), 42);

  // Check setting a parameter (DOUBLE)
  rtt_ros2_node::getNode(this)->declare_parameter("double_parameter");
  EXPECT_TRUE(
    setparam_operation.call("double_parameter", rclcpp::ParameterValue(3.14159)));

  // Check getting a parameter (DOUBLE)
  rclcpp::ParameterValue new_double = getparam_operation.call("double_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_DOUBLE, new_double.get_type());
  EXPECT_EQ(new_double.get<double>(), 3.14159);

  // Check setting a parameter (BOOL)
  rtt_ros2_node::getNode(this)->declare_parameter("bool_parameter");
  EXPECT_TRUE(
    setparam_operation.call("bool_parameter", rclcpp::ParameterValue(true)));

  // Check getting a parameter (BOOL)
  rclcpp::ParameterValue new_bool = getparam_operation.call("bool_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_BOOL, new_bool.get_type());
  EXPECT_EQ(new_bool.get<bool>(), true);

  // Check setting a parameter (STRING)
  rtt_ros2_node::getNode(this)->declare_parameter("string_parameter");
  EXPECT_TRUE(
    setparam_operation.call(
      "string_parameter",
      rclcpp::ParameterValue("answer to life and universe")));

  // Check getting a parameter (STRING)
  rclcpp::ParameterValue new_string = getparam_operation.call("string_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_STRING, new_string.get_type());
  EXPECT_EQ(new_string.get<std::string>().compare(std::string("answer to life and universe")), 0);
}

TEST_F(TestRosParams, TestComponentNodeParams)
{
  // Create a component-local ROS node
  ASSERT_TRUE(loadService("ros2_node"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Get local service
  ASSERT_TRUE(this->loadService("rosparam"));
  RTT::Service::shared_ptr local_params =
    this->provides("Params");
  ASSERT_TRUE(nullptr != local_params);

  // Check set non-existant parameters
  RTT::OperationCaller<bool(std::string, rclcpp::ParameterValue)>
  setparam_operation = local_params->getOperation("setParameter");
  ASSERT_TRUE(setparam_operation.ready());
  EXPECT_FALSE(
    setparam_operation.call("fake_parameter", rclcpp::ParameterValue(42)));

  // Check get non-existant parameters
  RTT::OperationCaller<rclcpp::ParameterValue(std::string)>
  getparam_operation = local_params->getOperation("getParameter");
  ASSERT_TRUE(getparam_operation.ready());
  EXPECT_EQ(
    rclcpp::PARAMETER_NOT_SET,
    getparam_operation.call("fake_parameter").get_type()
  );

  // Check setting a parameter (INTEGER)
  rtt_ros2_node::getNode(this)->declare_parameter("int_parameter");
  EXPECT_TRUE(
    setparam_operation.call("int_parameter", rclcpp::ParameterValue(41)));

  // Check getting a parameter (INTEGER)
  rclcpp::ParameterValue new_int = getparam_operation.call("int_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_INTEGER, new_int.get_type());
  EXPECT_EQ(new_int.get<int>(), 41);

  // Check setting a parameter (BOOL)
  rtt_ros2_node::getNode(this)->declare_parameter("bool_parameter");
  EXPECT_TRUE(
    setparam_operation.call("bool_parameter", rclcpp::ParameterValue(false)));

  // Check getting a parameter (BOOL)
  rclcpp::ParameterValue new_bool = getparam_operation.call("bool_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_BOOL, new_bool.get_type());
  EXPECT_EQ(new_bool.get<bool>(), false);

  // Check storing a parameter provoke failure
  RTT::OperationCaller<bool(std::string, std::string)>
  storeprop_operation = local_params->getOperation("storeProperty");
  ASSERT_TRUE(storeprop_operation.ready());
  EXPECT_FALSE(storeprop_operation.call("fake_property", "int_parameter"));
  EXPECT_FALSE(storeprop_operation.call("int_property", "fake_parameter"));

  // Check storing a parameter (INTEGER)
  EXPECT_EQ(getparam_operation.call("int_parameter").get<int>(), 41);
  EXPECT_TRUE(storeprop_operation.call("int_property", "int_parameter"));
  EXPECT_EQ(getparam_operation.call("int_parameter").get<int>(), 42);

  // Check storing a parameter (BOOL)
  EXPECT_EQ(getparam_operation.call("bool_parameter").get<bool>(), false);
  EXPECT_TRUE(storeprop_operation.call("bool_property", "bool_parameter"));
  EXPECT_EQ(getparam_operation.call("bool_parameter").get<bool>(), true);

  // Check loading a parameter common
  RTT::OperationCaller<bool(std::string, std::string)>
  loadprop_operation = local_params->getOperation("loadProperty");
  ASSERT_TRUE(loadprop_operation.ready());
  EXPECT_FALSE(loadprop_operation.call("fake_property", "fake_parameter"));
  EXPECT_TRUE(loadprop_operation.call("fake_property", "int_parameter"));
  auto params_service = boost::dynamic_pointer_cast<rtt_ros2_params::Params>(
    this->provides("Params"));
  ASSERT_TRUE(params_service);
  EXPECT_EQ(params_service->getOrphans()["fake_property"].get<int>(), 42);

  // Check loading a parameter (INTEGER)
  this->int_member_ = 41;
  setparam_operation.call("int_parameter", rclcpp::ParameterValue(42));
  EXPECT_TRUE(loadprop_operation.call("int_property", "int_parameter"));
  EXPECT_EQ(42, this->int_member_);

  // Check loading a parameter (BOOL)
  this->bool_member_ = false;
  setparam_operation.call("bool_parameter", rclcpp::ParameterValue(true));
  EXPECT_TRUE(loadprop_operation.call("bool_property", "bool_parameter"));
  EXPECT_EQ(true, this->bool_member_);

  // Check bad assignment
  rtt_ros2_node::getNode(this)->declare_parameter("string_parameter");
  EXPECT_TRUE(
    setparam_operation.call("string_parameter", rclcpp::ParameterValue("string_value")));
  EXPECT_FALSE(
    loadprop_operation.call("bool_property", "string_parameter"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv)
  // because this function removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(
    new TestRosParamsEnvironment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
