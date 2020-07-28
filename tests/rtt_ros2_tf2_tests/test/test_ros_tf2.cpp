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
#include "rtt/OperationCaller.hpp"
#include "rtt/internal/GlobalService.hpp"
#include "rtt/internal/GlobalEngine.hpp"
#include "rtt/os/startstop.h"

#include "rtt_ros2/rtt_ros2.hpp"
#include "rtt_ros2_node/rtt_ros2_node.hpp"
#include "rtt_ros2_tf2/rtt_ros2_tf2.hpp"


// #include "geometry_msgs/typekit/Types.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "gtest/gtest.h"

class TestRosTf2Environment
  : public ::testing::Environment
{
  void SetUp() override
  {
    EXPECT_TRUE(rtt_ros2::import("rtt_ros2_tf2"));
  }
};

class TestRosTf2
  : public RTT::TaskContext,
  public ::testing::Test
{
public:
  TestRosTf2()
  : RTT::TaskContext("TestRosTf2")
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

TEST_F(TestRosTf2, TestGlobalNodeTf2)
{
  RTT::Logger::In in(getName());

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

  // Create a global (process-wide) TF2 service
  // ASSERT_TRUE(RTT::internal::GlobalService::Instance()->provides("tf2"));
  RTT::Service::shared_ptr global_tf2 =
    global_ros->getService("tf2");
  ASSERT_TRUE(nullptr != global_tf2);

  // Check operations
  RTT::OperationCaller<void(geometry_msgs::msg::TransformStamped)>
  broadcast_transform_operation = global_tf2->getOperation("broadcastTransform");
  ASSERT_TRUE(broadcast_transform_operation.ready());

  RTT::OperationCaller<void(std::vector<geometry_msgs::msg::TransformStamped>)>
  broadcast_transforms_operation = global_tf2->getOperation("broadcastTransforms");
  ASSERT_TRUE(broadcast_transforms_operation.ready());

  RTT::OperationCaller<void(geometry_msgs::msg::TransformStamped)>
  broadcast_static_transform_operation =
    global_tf2->getOperation("broadcastStaticTransform");
  ASSERT_TRUE(broadcast_static_transform_operation.ready());

  RTT::OperationCaller<void(std::vector<geometry_msgs::msg::TransformStamped>)>
  broadcast_static_transforms_operation =
    global_tf2->getOperation("broadcastStaticTransforms");
  ASSERT_TRUE(broadcast_static_transforms_operation.ready());

  // RTT::OperationCaller<bool(std::string, std::string)>
  // can_transform_operation =
  //   global_tf2->getOperation("canTransform");
  // ASSERT_TRUE(can_transform_operation.ready());

  RTT::OperationCaller<geometry_msgs::msg::TransformStamped(std::string, std::string)>
  lookup_transform_operation =
    global_tf2->getOperation("lookupTransform");
  ASSERT_TRUE(lookup_transform_operation.ready());

  RTT::OperationCaller<void(void)>
  clear_operation =
    global_tf2->getOperation("clear");
  ASSERT_TRUE(clear_operation.ready());

  // Check broadcastTransform / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out, test_transform_in;
    test_transform_out.child_frame_id = std::string("gbase0");
    test_transform_out.header.frame_id = std::string("world");
    test_transform_out.transform.translation.x = 2.0;
    test_transform_out.transform.rotation.y = 5.3;
    test_transform_in = lookup_transform_operation.call("world", "gbase0");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_transform_operation.call(test_transform_out);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("world", "gbase0");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
  // Check broadcastTransforms / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out_1, test_transform_out_2,
      test_transform_in;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    test_transform_out_1.child_frame_id = std::string("gbase1");
    test_transform_out_1.header.frame_id = std::string("world");
    test_transform_out_1.transform.translation.x = 2.0;
    test_transform_out_1.transform.rotation.y = 5.3;
    transforms.push_back(test_transform_out_1);
    test_transform_out_2.child_frame_id = std::string("gbase2");
    test_transform_out_2.header.frame_id = std::string("world");
    test_transform_out_2.transform.translation.x = 3.0;
    test_transform_out_2.transform.rotation.y = 2.3;
    transforms.push_back(test_transform_out_2);
    test_transform_in = lookup_transform_operation.call("gbase1", "gbase2");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_transforms_operation.call(transforms);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("gbase1", "gbase2");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
  // Check broadcastStaticTransform / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out, test_transform_in;
    test_transform_out.child_frame_id = std::string("gbase3");
    test_transform_out.header.frame_id = std::string("world");
    test_transform_out.transform.translation.x = 2.0;
    test_transform_out.transform.rotation.y = 5.3;
    test_transform_in = lookup_transform_operation.call("world", "gbase3");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_static_transform_operation.call(test_transform_out);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("world", "gbase3");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
  // Check broadcastStaticTransforms / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out_1, test_transform_out_2,
      test_transform_in;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    test_transform_out_1.child_frame_id = std::string("gbase4");
    test_transform_out_1.header.frame_id = std::string("world");
    test_transform_out_1.transform.translation.x = 2.0;
    test_transform_out_1.transform.rotation.y = 5.3;
    transforms.push_back(test_transform_out_1);
    test_transform_out_2.child_frame_id = std::string("gbase5");
    test_transform_out_2.header.frame_id = std::string("world");
    test_transform_out_2.transform.translation.x = 3.0;
    test_transform_out_2.transform.rotation.y = 2.3;
    transforms.push_back(test_transform_out_2);
    test_transform_in = lookup_transform_operation.call("gbase4", "gbase5");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_static_transforms_operation.call(transforms);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("gbase4", "gbase5");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
}

TEST_F(TestRosTf2, TestComponentNodeTf2)
{
  RTT::Logger::In in(getName());

  // Create a component-local ROS node
  ASSERT_TRUE(loadService("rosnode"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Get local service
  ASSERT_TRUE(this->loadService("tf2"));
  RTT::Service::shared_ptr local_tf2 =
    this->provides("tf2");
  ASSERT_TRUE(nullptr != local_tf2);

  // Check operations
  RTT::OperationCaller<void(geometry_msgs::msg::TransformStamped)>
  broadcast_transform_operation = local_tf2->getOperation("broadcastTransform");
  broadcast_transform_operation.setCaller(RTT::internal::GlobalEngine::Instance());
  ASSERT_TRUE(broadcast_transform_operation.ready());

  RTT::OperationCaller<void(std::vector<geometry_msgs::msg::TransformStamped>)>
  broadcast_transforms_operation = local_tf2->getOperation("broadcastTransforms");
  broadcast_transforms_operation.setCaller(RTT::internal::GlobalEngine::Instance());
  ASSERT_TRUE(broadcast_transforms_operation.ready());

  RTT::OperationCaller<void(geometry_msgs::msg::TransformStamped)>
  broadcast_static_transform_operation =
    local_tf2->getOperation("broadcastStaticTransform");
  broadcast_static_transform_operation.setCaller(RTT::internal::GlobalEngine::Instance());
  ASSERT_TRUE(broadcast_static_transform_operation.ready());

  RTT::OperationCaller<void(std::vector<geometry_msgs::msg::TransformStamped>)>
  broadcast_static_transforms_operation =
    local_tf2->getOperation("broadcastStaticTransforms");
  broadcast_static_transforms_operation.setCaller(RTT::internal::GlobalEngine::Instance());
  ASSERT_TRUE(broadcast_static_transforms_operation.ready());

  // RTT::OperationCaller<bool(std::string, std::string)>
  // can_transform_operation =
  //   global_tf2->getOperation("canTransform");
  // ASSERT_TRUE(can_transform_operation.ready());

  RTT::OperationCaller<geometry_msgs::msg::TransformStamped(std::string, std::string)>
  lookup_transform_operation =
    local_tf2->getOperation("lookupTransform");
  lookup_transform_operation.setCaller(RTT::internal::GlobalEngine::Instance());
  ASSERT_FALSE(RTT::internal::GlobalEngine::Instance()->isSelf());
  ASSERT_TRUE(lookup_transform_operation.ready());

  RTT::OperationCaller<void(void)>
  clear_operation =
    local_tf2->getOperation("clear");
  clear_operation.setCaller(RTT::internal::GlobalEngine::Instance());
  ASSERT_TRUE(clear_operation.ready());

  // Check broadcastTransform / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out, test_transform_in;
    test_transform_out.child_frame_id = std::string("base0");
    test_transform_out.header.frame_id = std::string("world");
    test_transform_out.transform.translation.x = 2.0;
    test_transform_out.transform.rotation.y = 5.3;
    test_transform_in = lookup_transform_operation.call("world", "base0");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_transform_operation.call(test_transform_out);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("world", "base0");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
  // Check broadcastTransforms / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out_1, test_transform_out_2,
      test_transform_in;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    test_transform_out_1.child_frame_id = std::string("base1");
    test_transform_out_1.header.frame_id = std::string("world");
    test_transform_out_1.transform.translation.x = 2.0;
    test_transform_out_1.transform.rotation.y = 5.3;
    transforms.push_back(test_transform_out_1);
    test_transform_out_2.child_frame_id = std::string("base2");
    test_transform_out_2.header.frame_id = std::string("world");
    test_transform_out_2.transform.translation.x = 3.0;
    test_transform_out_2.transform.rotation.y = 2.3;
    transforms.push_back(test_transform_out_2);
    test_transform_in = lookup_transform_operation.call("base1", "base2");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_transforms_operation.call(transforms);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("base1", "base2");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
  // Check broadcastStaticTransform / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out, test_transform_in;
    test_transform_out.child_frame_id = std::string("base3");
    test_transform_out.header.frame_id = std::string("world");
    test_transform_out.transform.translation.x = 2.0;
    test_transform_out.transform.rotation.y = 5.3;
    test_transform_in = lookup_transform_operation.call("world", "base3");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_static_transform_operation.call(test_transform_out);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("world", "base3");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
  // Check broadcastStaticTransforms / lookupTransform
  {
    geometry_msgs::msg::TransformStamped test_transform_out_1, test_transform_out_2,
      test_transform_in;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    test_transform_out_1.child_frame_id = std::string("base4");
    test_transform_out_1.header.frame_id = std::string("world");
    test_transform_out_1.transform.translation.x = 2.0;
    test_transform_out_1.transform.rotation.y = 5.3;
    transforms.push_back(test_transform_out_1);
    test_transform_out_2.child_frame_id = std::string("base5");
    test_transform_out_2.header.frame_id = std::string("world");
    test_transform_out_2.transform.translation.x = 3.0;
    test_transform_out_2.transform.rotation.y = 2.3;
    transforms.push_back(test_transform_out_2);
    test_transform_in = lookup_transform_operation.call("base4", "base5");
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
    EXPECT_EQ(test_transform_in, geometry_msgs::msg::TransformStamped());
    broadcast_static_transforms_operation.call(transforms);
    // RTT::log(RTT::Debug) << "Out: " <<
    // test_transform_out.transform.translation.x << RTT::endlog();
    test_transform_in = lookup_transform_operation.call("base4", "base5");
    EXPECT_NE(test_transform_in, geometry_msgs::msg::TransformStamped());
    // RTT::log(RTT::Debug) << "In: " << test_transform_in.transform.translation.x << RTT::endlog();
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv)
  // because this function removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(
    new TestRosTf2Environment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
