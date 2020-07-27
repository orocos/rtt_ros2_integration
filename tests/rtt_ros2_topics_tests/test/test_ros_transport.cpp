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
#include "rtt_ros2_topics/protocol_id.h"
#include "rtt_ros2_topics/topic.hpp"

#include "test_msgs/typekit/Types.hpp"

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

// MessageInitialization is an alias for either
// rosidl_generator_cpp::MessageInitialization or rosidl_runtime_cpp::MessageInitialization
// depending on the ROS version.
#ifdef ROSIDL_RUNTIME_CPP__MESSAGE_INITIALIZATION_HPP_
using rosidl_runtime_cpp::MessageInitialization;
#else
using rosidl_generator_cpp::MessageInitialization;
#endif

class TestRosTransportEnvironment
  : public ::testing::Environment
{
  void SetUp() override
  {
    EXPECT_TRUE(rtt_ros2::import("rtt_ros2_test_msgs"));
  }
};

class TestRosTransport
  : public RTT::TaskContext,
  public ::testing::Test
{
public:
  using Arrays = test_msgs::msg::Arrays;
  using BasicTypes = test_msgs::msg::BasicTypes;
  using BoundedSequences = test_msgs::msg::BoundedSequences;
  using MultiNested = test_msgs::msg::MultiNested;
  using Nested = test_msgs::msg::Nested;
  using Strings = test_msgs::msg::Strings;
  using UnboundedSequences = test_msgs::msg::UnboundedSequences;
  using WStrings = test_msgs::msg::WStrings;

  TestRosTransport()
  : RTT::TaskContext("TestRosTransport")
  {
    this->addPort("arrays_out", arrays_out_);
    this->addPort("arrays_in", arrays_in_);
    this->addPort("basic_types_out", basic_types_out_);
    this->addPort("basic_types_in", basic_types_in_);
    this->addPort("bounded_sequences_out", bounded_sequences_out_);
    this->addPort("bounded_sequences_in", bounded_sequences_in_);
    this->addPort("multi_nested_out", multi_nested_out_);
    this->addPort("multi_nested_in", multi_nested_in_);
    this->addPort("nested_out", nested_out_);
    this->addPort("nested_in", nested_in_);
    this->addPort("strings_out", strings_out_);
    this->addPort("strings_in", strings_in_);
    this->addPort("unbounded_sequences_out", unbounded_sequences_out_);
    this->addPort("unbounded_sequences_in", unbounded_sequences_in_);
    this->addPort("wstrings_out", wstrings_out_);
    this->addPort("wstrings_in", wstrings_in_);
  }

protected:
  static RTT::ConnPolicy getPolicy(int buffer_size, bool latch)
  {
    if (buffer_size == 1) {
      return rtt_ros2_topics::topic({}, latch);
    } else {
      return rtt_ros2_topics::topicBuffered({}, buffer_size, latch);
    }
  }

  void createOutputStreams(int buffer_size, bool latch)
  {
    RTT::ConnPolicy policy = getPolicy(buffer_size, latch);

    policy.name_id = "~/arrays";
    ASSERT_TRUE(arrays_out_.createStream(policy));
    policy.name_id = "~/basic_types";
    ASSERT_TRUE(basic_types_out_.createStream(policy));
    policy.name_id = "~/bounded_sequences";
    ASSERT_TRUE(bounded_sequences_out_.createStream(policy));
    policy.name_id = "~/multi_nested";
    ASSERT_TRUE(multi_nested_out_.createStream(policy));
    policy.name_id = "~/nested";
    ASSERT_TRUE(nested_out_.createStream(policy));
    policy.name_id = "~/strings";
    ASSERT_TRUE(strings_out_.createStream(policy));
    policy.name_id = "~/unbounded_sequences";
    ASSERT_TRUE(unbounded_sequences_out_.createStream(policy));
    policy.name_id = "~/wstrings";
    ASSERT_TRUE(wstrings_out_.createStream(policy));
  }

  void createInputStreams(int buffer_size, bool latch)
  {
    RTT::ConnPolicy policy = getPolicy(buffer_size, latch);

    policy.name_id = "~/arrays";
    ASSERT_TRUE(arrays_in_.createStream(policy));
    policy.name_id = "~/basic_types";
    ASSERT_TRUE(basic_types_in_.createStream(policy));
    policy.name_id = "~/bounded_sequences";
    ASSERT_TRUE(bounded_sequences_in_.createStream(policy));
    policy.name_id = "~/multi_nested";
    ASSERT_TRUE(multi_nested_in_.createStream(policy));
    policy.name_id = "~/nested";
    ASSERT_TRUE(nested_in_.createStream(policy));
    policy.name_id = "~/strings";
    ASSERT_TRUE(strings_in_.createStream(policy));
    policy.name_id = "~/unbounded_sequences";
    ASSERT_TRUE(unbounded_sequences_in_.createStream(policy));
    policy.name_id = "~/wstrings";
    ASSERT_TRUE(wstrings_in_.createStream(policy));
  }

  void writeToOutputPorts()
  {
    ASSERT_EQ(arrays_out_.write(arrays_msg_), RTT::WriteSuccess);
    ASSERT_EQ(basic_types_out_.write(basic_types_msg_), RTT::WriteSuccess);
    ASSERT_EQ(bounded_sequences_out_.write(bounded_sequences_msg_), RTT::WriteSuccess);
    ASSERT_EQ(multi_nested_out_.write(multi_nested_msg_), RTT::WriteSuccess);
    ASSERT_EQ(nested_out_.write(nested_msg_), RTT::WriteSuccess);
    ASSERT_EQ(strings_out_.write(strings_msg_), RTT::WriteSuccess);
    ASSERT_EQ(unbounded_sequences_out_.write(unbounded_sequences_msg_), RTT::WriteSuccess);
    ASSERT_EQ(wstrings_out_.write(wstrings_msg_), RTT::WriteSuccess);
  }

  template<typename T, typename TimeoutT>
  bool readPortWithTimeout(RTT::InputPort<T> & input_port, T & sample, TimeoutT timeout)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < start + timeout) {
      static constexpr bool kCopyOldData = false;
      if (input_port.read(sample, kCopyOldData) == RTT::NewData) {
        return true;
      }
      std::this_thread::sleep_for(timeout / 10);
    }
    return false;
  }

  void readFromInputPorts()
  {
    static constexpr auto kTimeout = std::chrono::seconds(1);
    {
      Arrays received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(arrays_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, arrays_msg_);
      }
    }
    {
      BasicTypes received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(basic_types_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, basic_types_msg_);
      }
    }
    {
      BoundedSequences received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(bounded_sequences_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, bounded_sequences_msg_);
      }
    }
    {
      MultiNested received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(multi_nested_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, multi_nested_msg_);
      }
    }
    {
      Nested received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(nested_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, nested_msg_);
      }
    }
    {
      Strings received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(strings_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, strings_msg_);
      }
    }
    {
      UnboundedSequences received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(unbounded_sequences_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, unbounded_sequences_msg_);
      }
    }
    {
      WStrings received{MessageInitialization::SKIP};
      bool success = false;
      EXPECT_TRUE(success = readPortWithTimeout(wstrings_in_, received, kTimeout));
      if (success) {
        EXPECT_EQ(received, wstrings_msg_);
      }
    }
  }

protected:
  Arrays arrays_msg_;
  RTT::OutputPort<Arrays> arrays_out_;
  RTT::InputPort<Arrays> arrays_in_;
  BasicTypes basic_types_msg_;
  RTT::OutputPort<BasicTypes> basic_types_out_;
  RTT::InputPort<BasicTypes> basic_types_in_;
  BoundedSequences bounded_sequences_msg_;
  RTT::OutputPort<BoundedSequences> bounded_sequences_out_;
  RTT::InputPort<BoundedSequences> bounded_sequences_in_;
  MultiNested multi_nested_msg_;
  RTT::OutputPort<MultiNested> multi_nested_out_;
  RTT::InputPort<MultiNested> multi_nested_in_;
  Nested nested_msg_;
  RTT::OutputPort<Nested> nested_out_;
  RTT::InputPort<Nested> nested_in_;
  Strings strings_msg_;
  RTT::OutputPort<Strings> strings_out_;
  RTT::InputPort<Strings> strings_in_;
  UnboundedSequences unbounded_sequences_msg_;
  RTT::OutputPort<UnboundedSequences> unbounded_sequences_out_;
  RTT::InputPort<UnboundedSequences> unbounded_sequences_in_;
  WStrings wstrings_msg_;
  RTT::OutputPort<WStrings> wstrings_out_;
  RTT::InputPort<WStrings> wstrings_in_;
};

TEST_F(TestRosTransport, PublishSubscribeGlobalNode)
{
  // Create a global (process-wide) ROS node
  RTT::Service::shared_ptr global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
  ASSERT_TRUE(global_ros);
  RTT::OperationCaller<bool()> create_node = global_ros->getOperation("create_node");
  ASSERT_TRUE(create_node.ready());
  ASSERT_TRUE(create_node());
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Create streams
  static constexpr bool kInit = false;
  createOutputStreams(1, kInit);
  createInputStreams(1, kInit);

  // Write to output ports (i.e. publish)
  writeToOutputPorts();

  // Read from input ports and check the samples
  readFromInputPorts();
}

TEST_F(TestRosTransport, PublishSubscribeComponentNode)
{
  // Create a component-local ROS node
  ASSERT_TRUE(loadService("rosnode"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Create streams
  static constexpr bool kInit = false;
  createOutputStreams(1, kInit);
  createInputStreams(1, kInit);

  // Write to output ports (i.e. publish)
  writeToOutputPorts();

  // Read from input ports and check the samples
  readFromInputPorts();
}

TEST_F(TestRosTransport, PublishSubscribeQueue)
{
  // Create a component-local ROS node
  ASSERT_TRUE(loadService("rosnode"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Create publishers
  static constexpr int kBufferSize = 10;
  static constexpr bool kInit = false;
  createOutputStreams(kBufferSize, kInit);
  createInputStreams(kBufferSize, kInit);

  // Write kBufferSize samples to an output port
  for (int32_t i = 0; i < 10; ++i) {
    basic_types_msg_.int32_value = i;
    basic_types_out_.write(basic_types_msg_);
  }

  // Read from input ports and check the samples
  static constexpr auto kTimeout = std::chrono::seconds(1);
  for (int32_t i = 0; i < 10; ++i) {
    BasicTypes received{MessageInitialization::SKIP};
    ASSERT_TRUE(readPortWithTimeout(basic_types_in_, received, kTimeout));
    EXPECT_EQ(received.int32_value, i);
  }
}

TEST_F(TestRosTransport, PublishSubscribeLatched)
{
  // Create a component-local ROS node
  ASSERT_TRUE(loadService("rosnode"));
  ASSERT_TRUE(rtt_ros2_node::getNodeService(this) != nullptr);
  ASSERT_TRUE(rtt_ros2_node::getNode(this) != nullptr);

  // Create output streams / publishers
  static constexpr bool kInit = true;
  createOutputStreams(1, kInit);

  // Write to output ports (i.e. publish)
  writeToOutputPorts();

  // Connect input streams / subscriptions
  createInputStreams(1, kInit);

  // Read from input ports and check the samples
  readFromInputPorts();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv) because this function
  // removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(new TestRosTransportEnvironment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
