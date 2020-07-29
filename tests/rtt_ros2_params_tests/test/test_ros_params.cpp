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
#include "rtt_ros2_params/rosparam.hpp"

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
    global_ros->getService("rosparam");
  ASSERT_TRUE(nullptr != global_params);

  rtt_ros2_params::RosParam rosparam(this);
  ASSERT_TRUE(rosparam.connectTo(global_params));

  // Check set non-existent parameters
  EXPECT_FALSE(
    rosparam.setParameter("fake_parameter", rclcpp::ParameterValue(42)));

  // Check get non-existent parameters
  EXPECT_EQ(
    rclcpp::PARAMETER_NOT_SET,
    rosparam.getParameter("fake_parameter").get_type()
  );

  // Check setting a parameter (INTEGER)
  rtt_ros2_node::getNode(this)->declare_parameter("int_parameter");
  EXPECT_TRUE(
    rosparam.setParameter("int_parameter", rclcpp::ParameterValue(42)));

  // Check getting a parameter (INTEGER)
  rclcpp::ParameterValue new_int = rosparam.getParameter("int_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_INTEGER, new_int.get_type());
  EXPECT_EQ(new_int.get<int>(), 42);

  // Check setting a parameter (DOUBLE)
  rtt_ros2_node::getNode(this)->declare_parameter("double_parameter");
  EXPECT_TRUE(
    rosparam.setParameter("double_parameter", rclcpp::ParameterValue(3.14159)));

  // Check getting a parameter (DOUBLE)
  rclcpp::ParameterValue new_double = rosparam.getParameter("double_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_DOUBLE, new_double.get_type());
  EXPECT_EQ(new_double.get<double>(), 3.14159);

  // Check setting a parameter (BOOL)
  rtt_ros2_node::getNode(this)->declare_parameter("bool_parameter");
  EXPECT_TRUE(
    rosparam.setParameter("bool_parameter", rclcpp::ParameterValue(true)));

  // Check getting a parameter (BOOL)
  rclcpp::ParameterValue new_bool = rosparam.getParameter("bool_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_BOOL, new_bool.get_type());
  EXPECT_EQ(new_bool.get<bool>(), true);

  // Check setting a parameter (STRING)
  rtt_ros2_node::getNode(this)->declare_parameter("string_parameter");
  EXPECT_TRUE(
    rosparam.setParameter(
      "string_parameter",
      rclcpp::ParameterValue("answer to life and universe")));

  // Check getting a parameter (STRING)
  rclcpp::ParameterValue new_string = rosparam.getParameter("string_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_STRING, new_string.get_type());
  EXPECT_EQ(new_string.get<std::string>().compare(std::string("answer to life and universe")), 0);
}

TEST_F(TestRosParams, TestComponentNodeParams)
{
  // Create a component-local ROS node
  ASSERT_TRUE(loadService("rosnode"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Get local service
  auto rosparam_ptr = this->getProvider<rtt_ros2_params::RosParam>("rosparam");
  ASSERT_TRUE(rosparam_ptr);
  auto & rosparam = *rosparam_ptr;

  // Check set non-existent parameters
  // True because rosnode is created with allow_undeclared_parameters(true)
  EXPECT_TRUE(
    rosparam.setParameter("new_fake_parameter", rclcpp::ParameterValue(42)));

  // Check get non-existent parameters
  EXPECT_EQ(
    rclcpp::PARAMETER_NOT_SET,
    rosparam.getParameter("fake_parameter").get_type()
  );

  // Check setting a parameter (INTEGER)
  rtt_ros2_node::getNode(this)->declare_parameter("int_parameter");
  EXPECT_TRUE(
    rosparam.setParameter("int_parameter", rclcpp::ParameterValue(41)));

  // Check getting a parameter (INTEGER)
  rclcpp::ParameterValue new_int = rosparam.getParameter("int_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_INTEGER, new_int.get_type());
  EXPECT_EQ(new_int.get<int>(), 41);

  // Check setting a parameter (BOOL)
  rtt_ros2_node::getNode(this)->declare_parameter("bool_parameter");
  EXPECT_TRUE(
    rosparam.setParameter("bool_parameter", rclcpp::ParameterValue(false)));

  // Check getting a parameter (BOOL)
  rclcpp::ParameterValue new_bool = rosparam.getParameter("bool_parameter");
  EXPECT_EQ(rclcpp::PARAMETER_BOOL, new_bool.get_type());
  EXPECT_EQ(new_bool.get<bool>(), false);

  // Check storing a parameter provoke failure
  EXPECT_FALSE(rosparam.storeProperty("fake_property", "int_parameter"));
  // True because rosnode is created with allow_undeclared_parameters(true)
  EXPECT_TRUE(rosparam.storeProperty("int_property", "new_parameter_autodeclared"));

  // Check storing a parameter (INTEGER)
  EXPECT_EQ(rosparam.getParameter("int_parameter").get<int>(), 41);
  EXPECT_TRUE(rosparam.storeProperty("int_property", "int_parameter"));
  EXPECT_EQ(rosparam.getParameter("int_parameter").get<int>(), 42);

  // Check storing a parameter (BOOL)
  EXPECT_EQ(rosparam.getParameter("bool_parameter").get<bool>(), false);
  EXPECT_TRUE(rosparam.storeProperty("bool_property", "bool_parameter"));
  EXPECT_EQ(rosparam.getParameter("bool_parameter").get<bool>(), true);

  // Check loading a parameter common
  EXPECT_FALSE(rosparam.loadProperty("fake_property", "fake_parameter"));
  EXPECT_FALSE(rosparam.loadProperty("fake_property", "int_parameter"));
  auto params_service = this->getProvider<rtt_ros2_params::RosParam>("rosparam");
  ASSERT_TRUE(params_service);

  // Check loading a parameter (INTEGER)
  this->int_member_ = 41;
  rosparam.setParameter("int_parameter", rclcpp::ParameterValue(42));
  EXPECT_TRUE(rosparam.loadProperty("int_property", "int_parameter"));
  EXPECT_EQ(42, this->int_member_);

  // Check loading a parameter (BOOL)
  this->bool_member_ = false;
  rosparam.setParameter("bool_parameter", rclcpp::ParameterValue(true));
  EXPECT_TRUE(rosparam.loadProperty("bool_property", "bool_parameter"));
  EXPECT_EQ(true, this->bool_member_);

  // Check bad assignment
  rtt_ros2_node::getNode(this)->declare_parameter("string_parameter");
  EXPECT_TRUE(
    rosparam.setParameter("string_parameter", rclcpp::ParameterValue("string_value")));
  EXPECT_FALSE(
    rosparam.loadProperty("bool_property", "string_parameter"));
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
