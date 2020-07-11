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

#include <string>

#include "rtt/RTT.hpp"
#include "rtt/OperationCaller.hpp"
#include "rtt/internal/GlobalService.hpp"
#include "rtt/os/startstop.h"

#include "rtt_ros2/rtt_ros2.hpp"
#include "rtt_ros2_topics/protocol_id.h"
#include "rtt_ros2_topics/topic.hpp"

#include "gtest/gtest.h"

// workaround for undefined reference linker errors
const int RTT::ConnPolicy::DATA;
const int RTT::ConnPolicy::BUFFER;
const int RTT::ConnPolicy::UNBUFFERED;

class TestTopicsServiceEnvironment
  : public ::testing::Environment
{
  void SetUp() override
  {
    EXPECT_TRUE(rtt_ros2::import("rtt_ros2_topics"));
  }
};

TEST(TestTopicsService, TopicsService)
{
  const std::string kTestTopic = "test_topic";
  const int kSize = 99;

  RTT::Service::shared_ptr ros =
    RTT::internal::GlobalService::Instance()->getService("ros");
  ASSERT_TRUE(ros != nullptr);
  EXPECT_EQ(ros->doc(), "ROS operations and services");

  // operation topic
  RTT::OperationCaller<RTT::ConnPolicy(std::string, bool)> topic =
    ros->getOperation("topic");
  EXPECT_TRUE(topic.ready());
  {
    RTT::ConnPolicy policy = topic(kTestTopic, false);
    EXPECT_EQ(policy.type, RTT::ConnPolicy::DATA);
    EXPECT_EQ(policy.size, 0);
    EXPECT_FALSE(policy.init);
    EXPECT_FALSE(policy.pull);
    EXPECT_EQ(policy.transport, ORO_ROS2_PROTOCOL_ID);
    EXPECT_EQ(policy.name_id, kTestTopic);
  }
  {
    RTT::ConnPolicy policy = topic(kTestTopic, true);
    EXPECT_EQ(policy.type, RTT::ConnPolicy::DATA);
    EXPECT_EQ(policy.size, 0);
    EXPECT_TRUE(policy.init);
    EXPECT_FALSE(policy.pull);
    EXPECT_EQ(policy.transport, ORO_ROS2_PROTOCOL_ID);
    EXPECT_EQ(policy.name_id, kTestTopic);
  }

  // operation topicLatched
  RTT::OperationCaller<RTT::ConnPolicy(std::string)> topicLatched =
    ros->getOperation("topicLatched");
  EXPECT_TRUE(topicLatched.ready());
  {
    RTT::ConnPolicy policy = topicLatched(kTestTopic);
    EXPECT_EQ(policy.type, RTT::ConnPolicy::DATA);
    EXPECT_EQ(policy.size, 0);
    EXPECT_TRUE(policy.init);
    EXPECT_FALSE(policy.pull);
    EXPECT_EQ(policy.transport, ORO_ROS2_PROTOCOL_ID);
    EXPECT_EQ(policy.name_id, kTestTopic);
  }

  // operation topicBuffered
  RTT::OperationCaller<RTT::ConnPolicy(std::string, int, bool)> topicBuffered =
    ros->getOperation("topicBuffered");
  EXPECT_TRUE(topicBuffered.ready());
  {
    RTT::ConnPolicy policy = topicBuffered(kTestTopic, kSize, false);
    EXPECT_EQ(policy.type, RTT::ConnPolicy::BUFFER);
    EXPECT_EQ(policy.size, kSize);
    EXPECT_FALSE(policy.init);
    EXPECT_FALSE(policy.pull);
    EXPECT_EQ(policy.transport, ORO_ROS2_PROTOCOL_ID);
    EXPECT_EQ(policy.name_id, kTestTopic);
  }
  {
    RTT::ConnPolicy policy = topicBuffered(kTestTopic, kSize, true);
    EXPECT_EQ(policy.type, RTT::ConnPolicy::BUFFER);
    EXPECT_EQ(policy.size, kSize);
    EXPECT_TRUE(policy.init);
    EXPECT_FALSE(policy.pull);
    EXPECT_EQ(policy.transport, ORO_ROS2_PROTOCOL_ID);
    EXPECT_EQ(policy.name_id, kTestTopic);
  }

  // operation topicDirect
  RTT::OperationCaller<RTT::ConnPolicy(std::string, int, bool)> topicDirect =
    ros->getOperation("topicDirect");
  EXPECT_TRUE(topicDirect.ready());
  {
    RTT::ConnPolicy policy = topicDirect(kTestTopic, kSize, false);
    EXPECT_EQ(policy.type, RTT::ConnPolicy::UNBUFFERED);
    EXPECT_EQ(policy.size, kSize);
    EXPECT_FALSE(policy.init);
    EXPECT_FALSE(policy.pull);
    EXPECT_EQ(policy.transport, ORO_ROS2_PROTOCOL_ID);
    EXPECT_EQ(policy.name_id, kTestTopic);
  }
  {
    RTT::ConnPolicy policy = topicDirect(kTestTopic, kSize, true);
    EXPECT_EQ(policy.type, RTT::ConnPolicy::UNBUFFERED);
    EXPECT_EQ(policy.size, kSize);
    EXPECT_TRUE(policy.init);
    EXPECT_FALSE(policy.pull);
    EXPECT_EQ(policy.transport, ORO_ROS2_PROTOCOL_ID);
    EXPECT_EQ(policy.name_id, kTestTopic);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv) because this function
  // removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(new TestTopicsServiceEnvironment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
