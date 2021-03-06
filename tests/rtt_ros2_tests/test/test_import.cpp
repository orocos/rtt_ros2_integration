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

#include "rtt_ros2/rtt_ros2.hpp"

#include "rtt/TaskContext.hpp"
#include "rtt/os/main.h"

#include "gtest/gtest.h"

TEST(TestImport, import)
{
  EXPECT_TRUE(rtt_ros2::import("rtt_ros2_tests"));

  RTT::TaskContext tc("TestComponent");
  EXPECT_TRUE(tc.loadService("rtt_ros2_tests::TestService"));
  EXPECT_TRUE(tc.provides("TestService"));

  EXPECT_FALSE(tc.loadService("rtt_ros2_tests::UnknownService"));
}

int ORO_main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
