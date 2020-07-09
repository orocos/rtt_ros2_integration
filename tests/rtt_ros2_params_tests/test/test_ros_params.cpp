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

// MessageInitialization is an alias for either
// rosidl_generator_cpp::MessageInitialization or rosidl_runtime_cpp::MessageInitialization
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
    // this->addProperty("int_property", &int_member_);
    // this->addProperty("bool_property", &bool_member_);
  }

protected:
  void test_something(int an_int, bool a_boolean)
  {
    int_member_ = an_int;
    bool_member_ = a_boolean;
    ASSERT_TRUE(true);
  }

protected:
  int int_member_;
  bool bool_member_;
};

TEST_F(TestRosParams, TrivialTestGlobalNode)
{
  // Create a global (process-wide) ROS node
  RTT::Service::shared_ptr global_ros =
    RTT::internal::GlobalService::Instance()->getService("ros");
  ASSERT_TRUE(global_ros);
  RTT::OperationCaller<bool()> create_node = global_ros->getOperation("create_node");
  ASSERT_TRUE(create_node.ready());
  ASSERT_TRUE(create_node());
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Test member function
  test_something(3, 5);
}

TEST_F(TestRosParams, TrivialTestComponentNode)
{
  // Create a component-local ROS node
  ASSERT_TRUE(loadService("ros2-node"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Test member function
  test_something(3, 5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv) because this function
  // removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(new TestRosParamsEnvironment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
