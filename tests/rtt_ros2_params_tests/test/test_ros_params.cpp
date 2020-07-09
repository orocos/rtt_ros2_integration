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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rtt/RTT.hpp"
#include "rtt/internal/GlobalService.hpp"
#include "rtt/os/startstop.h"

#include "rtt_ros2/rtt_ros2.hpp"
#include "rtt_ros2_node/rtt_ros2_node.hpp"

#include "gtest/gtest.h"

// /* True if the version of test_msgs is at least major.minor.patch */
// // (from https://github.com/ros2/rmw_cyclonedds/pull/51)
// #define test_msgs_VERSION_GTE(major, minor, patch) ( \
//     (major < test_msgs_VERSION_MAJOR) ? true \
//     : (major > test_msgs_VERSION_MAJOR) ? false \
//     : (minor < test_msgs_VERSION_MINOR) ? true \
//     : (minor > test_msgs_VERSION_MINOR) ? false \
//     : (patch < test_msgs_VERSION_PATCH) ? true \
//     : (patch > test_msgs_VERSION_PATCH) ? false \
//     : true)

// MessageInitialization is an alias for        Either
// rosidl_generator_cpp::MessageInitialization  Or
// rosidl_runtime_cpp::MessageInitialization
// depending on the ROS version.
#ifdef ROSIDL_RUNTIME_CPP__MESSAGE_INITIALIZATION_HPP_
using rosidl_runtime_cpp::MessageInitialization;
#else
using rosidl_generator_cpp::MessageInitialization;
#endif

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
    this->addProperty("bool_property", bool_member_);
  }

protected:
  void assign_properties(int an_int, bool a_boolean)
  {
    int_member_ = an_int;
    bool_member_ = a_boolean;
    ASSERT_TRUE(true);
  }

protected:
  int int_member_;
  bool bool_member_;
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
  // ASSERT_TRUE(RTT::internal::GlobalService::Instance()->provides("ros2-params"));
  RTT::Service::shared_ptr global_params =
    global_ros->getService("Params");
  ASSERT_TRUE(nullptr != global_params);

  // Test member function
  assign_properties(42, true);

  // Check set non-existant parameters
  ASSERT_TRUE(global_params->getOperation("setParameter"));
  ASSERT_FALSE(global_params->getOperation("nonExistantOperation"));
  RTT::OperationCaller<bool(std::string, rclcpp::ParameterValue)>
  setparam_operation = global_params->getOperation("setParameter");
  ASSERT_FALSE(
    setparam_operation.call("fake_parameter", rclcpp::ParameterValue(42)));

  // Check get non-existant parameters
  ASSERT_TRUE(global_params->getOperation("getParameter"));
  RTT::OperationCaller<rclcpp::ParameterValue(std::string)>
  getparam_operation = global_params->getOperation("getParameter");
  ASSERT_EQ(
    rclcpp::PARAMETER_NOT_SET,
    getparam_operation.call("fake_parameter").get_type()
  );

  // Check setting a parameter (INTEGER)
  rtt_ros2_node::getNode(this)->declare_parameter("int_parameter");
  ASSERT_TRUE(
    setparam_operation.call("int_parameter", rclcpp::ParameterValue(42)));

  // Check getting a parameter (INTEGER)
  rclcpp::ParameterValue new_int = getparam_operation.call("int_parameter");
  ASSERT_EQ(rclcpp::PARAMETER_INTEGER, new_int.get_type());
  ASSERT_EQ(new_int.get<int>(), 42);

  // Check setting a parameter (BOOL)
  rtt_ros2_node::getNode(this)->declare_parameter("bool_parameter");
  ASSERT_TRUE(
    setparam_operation.call("bool_parameter", rclcpp::ParameterValue(true)));

  // Check getting a parameter (BOOL)
  rclcpp::ParameterValue new_bool = getparam_operation.call("bool_parameter");
  ASSERT_EQ(rclcpp::PARAMETER_BOOL, new_bool.get_type());
  ASSERT_EQ(new_bool.get<bool>(), true);
}

TEST_F(TestRosParams, TestComponentNodeParams)
{
  // Create a component-local ROS node
  ASSERT_TRUE(loadService("ros2_node"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Get local service
  ASSERT_TRUE(this->loadService("ros2-params"));
  RTT::Service::shared_ptr local_params =
    this->provides("Params");
  ASSERT_TRUE(nullptr != local_params);


  // Test member function
  assign_properties(42, true);

  // Check set non-existant parameters
  ASSERT_TRUE(local_params->getOperation("setParameter"));
  ASSERT_FALSE(local_params->getOperation("nonExistantOperation"));
  RTT::OperationCaller<bool(std::string, rclcpp::ParameterValue)>
  setparam_operation = local_params->getOperation("setParameter");
  ASSERT_FALSE(
    setparam_operation.call("fake_parameter", rclcpp::ParameterValue(42)));

  // Check get non-existant parameters
  ASSERT_TRUE(local_params->getOperation("getParameter"));
  RTT::OperationCaller<rclcpp::ParameterValue(std::string)>
  getparam_operation = local_params->getOperation("getParameter");
  ASSERT_EQ(
    rclcpp::PARAMETER_NOT_SET,
    getparam_operation.call("fake_parameter").get_type()
  );

  // Check setting a parameter (INTEGER)
  rtt_ros2_node::getNode(this)->declare_parameter("int_parameter");
  ASSERT_TRUE(
    setparam_operation.call("int_parameter", rclcpp::ParameterValue(41)));

  // Check getting a parameter (INTEGER)
  rclcpp::ParameterValue new_int = getparam_operation.call("int_parameter");
  ASSERT_EQ(rclcpp::PARAMETER_INTEGER, new_int.get_type());
  ASSERT_EQ(new_int.get<int>(), 41);

  // Check setting a parameter (BOOL)
  rtt_ros2_node::getNode(this)->declare_parameter("bool_parameter");
  ASSERT_TRUE(
    setparam_operation.call("bool_parameter", rclcpp::ParameterValue(false)));

  // Check getting a parameter (BOOL)
  rclcpp::ParameterValue new_bool = getparam_operation.call("bool_parameter");
  ASSERT_EQ(rclcpp::PARAMETER_BOOL, new_bool.get_type());
  ASSERT_EQ(new_bool.get<bool>(), false);

  // Check storing a parameter provoke failure
  ASSERT_TRUE(local_params->getOperation("storeProperty"));
  RTT::OperationCaller<bool(std::string, std::string)>
  storeprop_operation = local_params->getOperation("storeProperty");
  ASSERT_FALSE(storeprop_operation.call("fake_property", "int_parameter"));
  ASSERT_FALSE(storeprop_operation.call("int_property", "fake_parameter"));

  // Check storing a parameter (INTEGER)
  ASSERT_EQ(getparam_operation.call("int_parameter").get<int>(), 41);
  ASSERT_TRUE(storeprop_operation.call("int_property", "int_parameter"));
  ASSERT_EQ(getparam_operation.call("int_parameter").get<int>(), 42);

  // Check storing a parameter (BOOL)
  ASSERT_EQ(getparam_operation.call("bool_parameter").get<bool>(), false);
  ASSERT_TRUE(storeprop_operation.call("bool_property", "bool_parameter"));
  ASSERT_EQ(getparam_operation.call("bool_parameter").get<bool>(), true);

  // Check loading a parameter common
  ASSERT_TRUE(local_params->getOperation("loadProperty"));
  RTT::OperationCaller<bool(std::string, std::string)>
  loadprop_operation = local_params->getOperation("loadProperty");
  ASSERT_FALSE(loadprop_operation.call("fake_property", "fake_parameter"));
  ASSERT_TRUE(loadprop_operation.call("fake_property", "int_parameter"));

  // TODO(spd-intermodalics): Fix the loading of existing parameters
  // which works OK in scripting, but NOK on C++

  // Check loading a parameter (INTEGER)
  this->int_member_ = 41;
  // ASSERT_TRUE(loadprop_operation.call("int_property", "int_parameter"));
  // ASSERT_EQ(42, this->int_member_);

  // Check loading a parameter (BOOL)
  this->bool_member_ = false;
  // ASSERT_TRUE(loadprop_operation.call("bool_property", "bool_parameter"));
  // ASSERT_EQ(true, this->bool_member_);
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
