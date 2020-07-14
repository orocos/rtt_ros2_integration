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
#include <vector>

#include "rtt/RTT.hpp"
#include "rtt/internal/DataSources.hpp"
#include "rtt/os/startstop.h"
#include "rtt/types/TypeInfoRepository.hpp"

#include "rtt_ros2/rtt_ros2.hpp"

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

/*
  rosidl_generator_cpp::BoundedVector has been moved to
  rosidl_runtime_cpp::BoundedVector since ROS 2 Foxy Fitzroy
  (https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/#rosidl-generator-c-cpp-namespace-api-changes)
*/
#ifdef ROSIDL_RUNTIME_CPP__BOUNDED_VECTOR_HPP_
using rosidl_runtime_cpp::BoundedVector;
#else
using rosidl_generator_cpp::BoundedVector;
#endif

class TestTypekitEnvironment
  : public ::testing::Environment
{
  void SetUp() override
  {
    EXPECT_TRUE(rtt_ros2::import("rtt_ros2_test_msgs"));
  }
};

class TestTypekit
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

  TestTypekit()
  : RTT::TaskContext("TestTypekit"),
    arrays_ds(new RTT::internal::ReferenceDataSource<Arrays>(arrays_msg)),
    basic_types_ds(new RTT::internal::ReferenceDataSource<BasicTypes>(basic_types_msg)),
    bounded_sequences_ds(new RTT::internal::ReferenceDataSource<BoundedSequences>(
        bounded_sequences_msg)),
    multi_nested_ds(new RTT::internal::ReferenceDataSource<MultiNested>(multi_nested_msg)),
    nested_ds(new RTT::internal::ReferenceDataSource<Nested>(nested_msg)),
    strings_ds(new RTT::internal::ReferenceDataSource<Strings>(strings_msg)),
    unbounded_sequences_ds(new RTT::internal::ReferenceDataSource<UnboundedSequences>(
        unbounded_sequences_msg)),
    wstrings_ds(new RTT::internal::ReferenceDataSource<WStrings>(wstrings_msg))
  {
    EXPECT_TRUE(arrays_ti = arrays_ds->getTypeInfo());
    EXPECT_TRUE(basic_types_ti = basic_types_ds->getTypeInfo());
    EXPECT_TRUE(bounded_sequences_ti = bounded_sequences_ds->getTypeInfo());
    EXPECT_TRUE(multi_nested_ti = multi_nested_ds->getTypeInfo());
    EXPECT_TRUE(nested_ti = nested_ds->getTypeInfo());
    EXPECT_TRUE(strings_ti = strings_ds->getTypeInfo());
    EXPECT_TRUE(unbounded_sequences_ti = unbounded_sequences_ds->getTypeInfo());
    EXPECT_TRUE(wstrings_ti = wstrings_ds->getTypeInfo());
  }

protected:
  template<typename T>
  typename RTT::internal::AssignableDataSource<T>::shared_ptr
  testGetMember(
    RTT::base::DataSourceBase::shared_ptr dsb,
    const std::string & member, T & ref)
  {
    SCOPED_TRACE(
      ::testing::Message() << "Checking non-array member '" << member << "'...");

    using RTT::internal::AssignableDataSource;
    using RTT::types::Types;
    typename AssignableDataSource<T>::shared_ptr data =
      AssignableDataSource<T>::narrow(dsb->getMember(member).get());
    EXPECT_TRUE(data != nullptr);
    if (data == nullptr) {return {};}
    EXPECT_EQ(data->getTypeInfo(), Types()->getTypeInfo<decltype(ref)>());
    EXPECT_EQ(data->getParent(), dsb);
    EXPECT_EQ(&(data->rvalue()), &ref);
    return data;
  }

  template<typename T>
  typename RTT::internal::AssignableDataSource<std::vector<T>>::shared_ptr
  testGetMember(
    RTT::base::DataSourceBase::shared_ptr dsb,
    const std::string & member, std::vector<T> & ref)
  {
    SCOPED_TRACE(
      ::testing::Message() << "Checking unbounded dynamic array member '" << member << "'...");

    using RTT::internal::AssignableDataSource;
    using RTT::types::Types;
    typename AssignableDataSource<std::vector<T>>::shared_ptr data =
      AssignableDataSource<std::vector<T>>::narrow(dsb->getMember(member).get());
    EXPECT_TRUE(data != nullptr);
    if (data == nullptr) {return {};}
    EXPECT_EQ(data->getTypeInfo(), Types()->getTypeInfo<decltype(ref)>());
    EXPECT_EQ(data->getParent(), dsb);
    EXPECT_EQ(&(data->rvalue()), &ref);
    return data;
  }

  template<typename T, std::size_t N>
  typename RTT::internal::AssignableDataSource<RTT::types::carray<T>>::shared_ptr
  testGetMember(
    RTT::base::DataSourceBase::shared_ptr dsb,
    const std::string & member, std::array<T, N> & ref)
  {
    SCOPED_TRACE(
      ::testing::Message() << "Checking static array member '" << member << "'...");

    using RTT::internal::AssignableDataSource;
    using RTT::types::carray;
    using RTT::types::Types;
    typename AssignableDataSource<carray<T>>::shared_ptr data =
      AssignableDataSource<carray<T>>::narrow(dsb->getMember(member).get());
    EXPECT_TRUE(data != nullptr);
    if (data == nullptr) {return {};}
    EXPECT_EQ(data->getTypeInfo(), Types()->getTypeInfo<RTT::types::carray<T>>());
    EXPECT_EQ(data->getParent(), dsb);
    EXPECT_EQ(data->rvalue().address(), ref.data());
    EXPECT_EQ(data->rvalue().count(), ref.size());
    return data;
  }

  template<typename T, std::size_t N>
  typename RTT::internal::AssignableDataSource<std::vector<T>>::shared_ptr
  testGetMember(
    RTT::base::DataSourceBase::shared_ptr dsb,
    const std::string & member,
    BoundedVector<T, N> & ref)
  {
    SCOPED_TRACE(
      ::testing::Message() << "Checking bounded dynamic array member '" << member << "'...");

    using RTT::internal::AssignableDataSource;
    using RTT::types::carray;
    using RTT::types::Types;
    typename AssignableDataSource<std::vector<T>>::shared_ptr data =
      AssignableDataSource<std::vector<T>>::narrow(dsb->getMember(member).get());
    EXPECT_TRUE(data != nullptr);
    if (data == nullptr) {return {};}
    // BoundedVector<T, N> is not (yet) known to the Orocos type system.
    EXPECT_EQ(data->getTypeInfo(), Types()->getTypeInfo<std::vector<T>>());
    EXPECT_EQ(data->getParent(), dsb);
    EXPECT_EQ(&(data->rvalue()), reinterpret_cast<std::vector<T> *>(&ref));
    return data;
  }

  template<typename T>
  std::size_t testGetCapacity(RTT::internal::AssignableDataSource<T> * ds)
  {
    RTT::internal::DataSource<int>::shared_ptr capacity =
      RTT::internal::DataSource<int>::narrow(ds->getMember("capacity").get());
    EXPECT_TRUE(capacity != nullptr);
    if (capacity == nullptr) {return static_cast<std::size_t>(-1);}
    return static_cast<std::size_t>(capacity->get());
  }

  template<typename T>
  std::size_t testGetSize(RTT::internal::AssignableDataSource<T> * ds)
  {
    RTT::internal::DataSource<int>::shared_ptr size =
      RTT::internal::DataSource<int>::narrow(ds->getMember("size").get());
    EXPECT_TRUE(size != nullptr);
    if (size == nullptr) {return static_cast<std::size_t>(-1);}
    return static_cast<std::size_t>(size->get());
  }

  Arrays arrays_msg;
  RTT::internal::ReferenceDataSource<Arrays>::shared_ptr arrays_ds;
  const RTT::types::TypeInfo * arrays_ti = nullptr;
  BasicTypes basic_types_msg;
  RTT::internal::ReferenceDataSource<BasicTypes>::shared_ptr basic_types_ds;
  const RTT::types::TypeInfo * basic_types_ti = nullptr;
  BoundedSequences bounded_sequences_msg;
  RTT::internal::ReferenceDataSource<BoundedSequences>::shared_ptr bounded_sequences_ds;
  const RTT::types::TypeInfo * bounded_sequences_ti = nullptr;
  MultiNested multi_nested_msg;
  RTT::internal::ReferenceDataSource<MultiNested>::shared_ptr multi_nested_ds;
  const RTT::types::TypeInfo * multi_nested_ti = nullptr;
  Nested nested_msg;
  RTT::internal::ReferenceDataSource<Nested>::shared_ptr nested_ds;
  const RTT::types::TypeInfo * nested_ti = nullptr;
  Strings strings_msg;
  RTT::internal::ReferenceDataSource<Strings>::shared_ptr strings_ds;
  const RTT::types::TypeInfo * strings_ti = nullptr;
  UnboundedSequences unbounded_sequences_msg;
  RTT::internal::ReferenceDataSource<UnboundedSequences>::shared_ptr unbounded_sequences_ds;
  const RTT::types::TypeInfo * unbounded_sequences_ti = nullptr;
  WStrings wstrings_msg;
  RTT::internal::ReferenceDataSource<WStrings>::shared_ptr wstrings_ds;
  const RTT::types::TypeInfo * wstrings_ti = nullptr;
};

TEST_F(TestTypekit, BasicTypes)
{
  EXPECT_EQ(basic_types_ds->getMemberNames().size(), 13);

  testGetMember(basic_types_ds, "bool_value", basic_types_msg.bool_value);
  testGetMember(basic_types_ds, "byte_value", basic_types_msg.byte_value);
  testGetMember(basic_types_ds, "char_value", basic_types_msg.char_value);
  testGetMember(basic_types_ds, "float32_value", basic_types_msg.float32_value);
  testGetMember(basic_types_ds, "float64_value", basic_types_msg.float64_value);
  testGetMember(basic_types_ds, "int8_value", basic_types_msg.int8_value);
  testGetMember(basic_types_ds, "uint8_value", basic_types_msg.uint8_value);
  testGetMember(basic_types_ds, "int16_value", basic_types_msg.int16_value);
  testGetMember(basic_types_ds, "uint16_value", basic_types_msg.uint16_value);
  testGetMember(basic_types_ds, "int32_value", basic_types_msg.int32_value);
  testGetMember(basic_types_ds, "uint32_value", basic_types_msg.uint32_value);
  testGetMember(basic_types_ds, "int64_value", basic_types_msg.int64_value);
  testGetMember(basic_types_ds, "uint64_value", basic_types_msg.uint64_value);
}

TEST_F(TestTypekit, Arrays)
{
  EXPECT_EQ(arrays_ds->getMemberNames().size(), 32);

  const auto bool_values =
    testGetMember(arrays_ds, "bool_values", arrays_msg.bool_values);
  EXPECT_EQ(testGetCapacity(bool_values.get()), arrays_msg.bool_values.size());
  EXPECT_EQ(testGetSize(bool_values.get()), arrays_msg.bool_values.size());

  const auto byte_values =
    testGetMember(arrays_ds, "byte_values", arrays_msg.byte_values);
  EXPECT_EQ(testGetCapacity(byte_values.get()), arrays_msg.byte_values.size());
  EXPECT_EQ(testGetSize(byte_values.get()), arrays_msg.byte_values.size());

  const auto char_values =
    testGetMember(arrays_ds, "char_values", arrays_msg.char_values);
  EXPECT_EQ(testGetCapacity(char_values.get()), arrays_msg.char_values.size());
  EXPECT_EQ(testGetSize(char_values.get()), arrays_msg.char_values.size());

  const auto float32_values =
    testGetMember(arrays_ds, "float32_values", arrays_msg.float32_values);
  EXPECT_EQ(testGetCapacity(float32_values.get()), arrays_msg.float32_values.size());
  EXPECT_EQ(testGetSize(float32_values.get()), arrays_msg.float32_values.size());

  const auto float64_values =
    testGetMember(arrays_ds, "float64_values", arrays_msg.float64_values);
  EXPECT_EQ(testGetCapacity(float64_values.get()), arrays_msg.float64_values.size());
  EXPECT_EQ(testGetSize(float64_values.get()), arrays_msg.float64_values.size());

  const auto int8_values =
    testGetMember(arrays_ds, "int8_values", arrays_msg.int8_values);
  EXPECT_EQ(testGetCapacity(int8_values.get()), arrays_msg.int8_values.size());
  EXPECT_EQ(testGetSize(int8_values.get()), arrays_msg.int8_values.size());

  const auto uint8_values =
    testGetMember(arrays_ds, "uint8_values", arrays_msg.uint8_values);
  EXPECT_EQ(testGetCapacity(uint8_values.get()), arrays_msg.uint8_values.size());
  EXPECT_EQ(testGetSize(uint8_values.get()), arrays_msg.uint8_values.size());

  const auto int16_values =
    testGetMember(arrays_ds, "int16_values", arrays_msg.int16_values);
  EXPECT_EQ(testGetCapacity(int16_values.get()), arrays_msg.int16_values.size());
  EXPECT_EQ(testGetSize(int16_values.get()), arrays_msg.int16_values.size());

  const auto uint16_values =
    testGetMember(arrays_ds, "uint16_values", arrays_msg.uint16_values);
  EXPECT_EQ(testGetCapacity(uint16_values.get()), arrays_msg.uint16_values.size());
  EXPECT_EQ(testGetSize(uint16_values.get()), arrays_msg.uint16_values.size());

  const auto int32_values =
    testGetMember(arrays_ds, "int32_values", arrays_msg.int32_values);
  EXPECT_EQ(testGetCapacity(int32_values.get()), arrays_msg.int32_values.size());
  EXPECT_EQ(testGetSize(int32_values.get()), arrays_msg.int32_values.size());

  const auto uint32_values =
    testGetMember(arrays_ds, "uint32_values", arrays_msg.uint32_values);
  EXPECT_EQ(testGetCapacity(uint32_values.get()), arrays_msg.uint32_values.size());
  EXPECT_EQ(testGetSize(uint32_values.get()), arrays_msg.uint32_values.size());

  const auto int64_values =
    testGetMember(arrays_ds, "int64_values", arrays_msg.int64_values);
  EXPECT_EQ(testGetCapacity(int64_values.get()), arrays_msg.int64_values.size());
  EXPECT_EQ(testGetSize(int64_values.get()), arrays_msg.int64_values.size());

  const auto uint64_values =
    testGetMember(arrays_ds, "uint64_values", arrays_msg.uint64_values);
  EXPECT_EQ(testGetCapacity(uint64_values.get()), arrays_msg.uint64_values.size());
  EXPECT_EQ(testGetSize(uint64_values.get()), arrays_msg.uint64_values.size());

  const auto string_values =
    testGetMember(arrays_ds, "string_values", arrays_msg.string_values);
  EXPECT_EQ(testGetCapacity(string_values.get()), arrays_msg.string_values.size());
  EXPECT_EQ(testGetSize(string_values.get()), arrays_msg.string_values.size());

  const auto basic_types_values =
    testGetMember(arrays_ds, "basic_types_values", arrays_msg.basic_types_values);
  EXPECT_EQ(testGetCapacity(basic_types_values.get()), arrays_msg.basic_types_values.size());
  EXPECT_EQ(testGetSize(basic_types_values.get()), arrays_msg.basic_types_values.size());

  const auto constants_values =
    testGetMember(arrays_ds, "constants_values", arrays_msg.constants_values);
  EXPECT_EQ(testGetCapacity(constants_values.get()), arrays_msg.constants_values.size());
  EXPECT_EQ(testGetSize(constants_values.get()), arrays_msg.constants_values.size());

  const auto defaults_values =
    testGetMember(arrays_ds, "defaults_values", arrays_msg.defaults_values);
  EXPECT_EQ(testGetCapacity(defaults_values.get()), arrays_msg.defaults_values.size());
  EXPECT_EQ(testGetSize(defaults_values.get()), arrays_msg.defaults_values.size());

  const auto bool_values_default =
    testGetMember(arrays_ds, "bool_values_default", arrays_msg.bool_values_default);
  EXPECT_EQ(testGetCapacity(bool_values_default.get()), arrays_msg.bool_values_default.size());
  EXPECT_EQ(testGetSize(bool_values_default.get()), arrays_msg.bool_values_default.size());

  const auto byte_values_default =
    testGetMember(arrays_ds, "byte_values_default", arrays_msg.byte_values_default);
  EXPECT_EQ(testGetCapacity(byte_values_default.get()), arrays_msg.byte_values_default.size());
  EXPECT_EQ(testGetSize(byte_values_default.get()), arrays_msg.byte_values_default.size());

  const auto char_values_default =
    testGetMember(arrays_ds, "char_values_default", arrays_msg.char_values_default);
  EXPECT_EQ(testGetCapacity(char_values_default.get()), arrays_msg.char_values_default.size());
  EXPECT_EQ(testGetSize(char_values_default.get()), arrays_msg.char_values_default.size());

  const auto float32_values_default =
    testGetMember(arrays_ds, "float32_values_default", arrays_msg.float32_values_default);
  EXPECT_EQ(
    testGetCapacity(float32_values_default.get()),
    arrays_msg.float32_values_default.size());
  EXPECT_EQ(testGetSize(float32_values_default.get()), arrays_msg.float32_values_default.size());

  const auto float64_values_default =
    testGetMember(arrays_ds, "float64_values_default", arrays_msg.float64_values_default);
  EXPECT_EQ(
    testGetCapacity(float64_values_default.get()),
    arrays_msg.float64_values_default.size());
  EXPECT_EQ(testGetSize(float64_values_default.get()), arrays_msg.float64_values_default.size());

  const auto int8_values_default =
    testGetMember(arrays_ds, "int8_values_default", arrays_msg.int8_values_default);
  EXPECT_EQ(testGetCapacity(int8_values_default.get()), arrays_msg.int8_values_default.size());
  EXPECT_EQ(testGetSize(int8_values_default.get()), arrays_msg.int8_values_default.size());

  const auto uint8_values_default =
    testGetMember(arrays_ds, "uint8_values_default", arrays_msg.uint8_values_default);
  EXPECT_EQ(testGetCapacity(uint8_values_default.get()), arrays_msg.uint8_values_default.size());
  EXPECT_EQ(testGetSize(uint8_values_default.get()), arrays_msg.uint8_values_default.size());

  const auto int16_values_default =
    testGetMember(arrays_ds, "int16_values_default", arrays_msg.int16_values_default);
  EXPECT_EQ(testGetCapacity(int16_values_default.get()), arrays_msg.int16_values_default.size());
  EXPECT_EQ(testGetSize(int16_values_default.get()), arrays_msg.int16_values_default.size());

  const auto uint16_values_default =
    testGetMember(arrays_ds, "uint16_values_default", arrays_msg.uint16_values_default);
  EXPECT_EQ(testGetCapacity(uint16_values_default.get()), arrays_msg.uint16_values_default.size());
  EXPECT_EQ(testGetSize(uint16_values_default.get()), arrays_msg.uint16_values_default.size());

  const auto int32_values_default =
    testGetMember(arrays_ds, "int32_values_default", arrays_msg.int32_values_default);
  EXPECT_EQ(testGetCapacity(int32_values_default.get()), arrays_msg.int32_values_default.size());
  EXPECT_EQ(testGetSize(int32_values_default.get()), arrays_msg.int32_values_default.size());

  const auto uint32_values_default =
    testGetMember(arrays_ds, "uint32_values_default", arrays_msg.uint32_values_default);
  EXPECT_EQ(testGetCapacity(uint32_values_default.get()), arrays_msg.uint32_values_default.size());
  EXPECT_EQ(testGetSize(uint32_values_default.get()), arrays_msg.uint32_values_default.size());

  const auto int64_values_default =
    testGetMember(arrays_ds, "int64_values_default", arrays_msg.int64_values_default);
  EXPECT_EQ(testGetCapacity(int64_values_default.get()), arrays_msg.int64_values_default.size());
  EXPECT_EQ(testGetSize(int64_values_default.get()), arrays_msg.int64_values_default.size());

  const auto uint64_values_default =
    testGetMember(arrays_ds, "uint64_values_default", arrays_msg.uint64_values_default);
  EXPECT_EQ(testGetCapacity(uint64_values_default.get()), arrays_msg.uint64_values_default.size());
  EXPECT_EQ(testGetSize(uint64_values_default.get()), arrays_msg.uint64_values_default.size());

  const auto string_values_default =
    testGetMember(arrays_ds, "string_values_default", arrays_msg.string_values_default);
  EXPECT_EQ(testGetCapacity(string_values_default.get()), arrays_msg.string_values_default.size());
  EXPECT_EQ(testGetSize(string_values_default.get()), arrays_msg.string_values_default.size());

  const auto alignment_check =
    testGetMember(arrays_ds, "alignment_check", arrays_msg.alignment_check);
}

TEST_F(TestTypekit, UnboundedSequences)
{
  EXPECT_EQ(unbounded_sequences_ds->getMemberNames().size(), 32);

  const auto bool_values =
    testGetMember(unbounded_sequences_ds, "bool_values", unbounded_sequences_msg.bool_values);
  EXPECT_EQ(testGetCapacity(bool_values.get()), unbounded_sequences_msg.bool_values.capacity());
  EXPECT_EQ(testGetSize(bool_values.get()), unbounded_sequences_msg.bool_values.size());

  const auto byte_values =
    testGetMember(unbounded_sequences_ds, "byte_values", unbounded_sequences_msg.byte_values);
  EXPECT_EQ(testGetCapacity(byte_values.get()), unbounded_sequences_msg.byte_values.capacity());
  EXPECT_EQ(testGetSize(byte_values.get()), unbounded_sequences_msg.byte_values.size());

  const auto char_values =
    testGetMember(unbounded_sequences_ds, "char_values", unbounded_sequences_msg.char_values);
  EXPECT_EQ(testGetCapacity(char_values.get()), unbounded_sequences_msg.char_values.capacity());
  EXPECT_EQ(testGetSize(char_values.get()), unbounded_sequences_msg.char_values.size());

  const auto float32_values =
    testGetMember(unbounded_sequences_ds, "float32_values", unbounded_sequences_msg.float32_values);
  EXPECT_EQ(
    testGetCapacity(float32_values.get()),
    unbounded_sequences_msg.float32_values.capacity());
  EXPECT_EQ(testGetSize(float32_values.get()), unbounded_sequences_msg.float32_values.size());

  const auto float64_values =
    testGetMember(unbounded_sequences_ds, "float64_values", unbounded_sequences_msg.float64_values);
  EXPECT_EQ(
    testGetCapacity(float64_values.get()),
    unbounded_sequences_msg.float64_values.capacity());
  EXPECT_EQ(testGetSize(float64_values.get()), unbounded_sequences_msg.float64_values.size());

  const auto int8_values =
    testGetMember(unbounded_sequences_ds, "int8_values", unbounded_sequences_msg.int8_values);
  EXPECT_EQ(testGetCapacity(int8_values.get()), unbounded_sequences_msg.int8_values.capacity());
  EXPECT_EQ(testGetSize(int8_values.get()), unbounded_sequences_msg.int8_values.size());

  const auto uint8_values =
    testGetMember(unbounded_sequences_ds, "uint8_values", unbounded_sequences_msg.uint8_values);
  EXPECT_EQ(testGetCapacity(uint8_values.get()), unbounded_sequences_msg.uint8_values.capacity());
  EXPECT_EQ(testGetSize(uint8_values.get()), unbounded_sequences_msg.uint8_values.size());

  const auto int16_values =
    testGetMember(unbounded_sequences_ds, "int16_values", unbounded_sequences_msg.int16_values);
  EXPECT_EQ(testGetCapacity(int16_values.get()), unbounded_sequences_msg.int16_values.capacity());
  EXPECT_EQ(testGetSize(int16_values.get()), unbounded_sequences_msg.int16_values.size());

  const auto uint16_values =
    testGetMember(unbounded_sequences_ds, "uint16_values", unbounded_sequences_msg.uint16_values);
  EXPECT_EQ(testGetCapacity(uint16_values.get()), unbounded_sequences_msg.uint16_values.capacity());
  EXPECT_EQ(testGetSize(uint16_values.get()), unbounded_sequences_msg.uint16_values.size());

  const auto int32_values =
    testGetMember(unbounded_sequences_ds, "int32_values", unbounded_sequences_msg.int32_values);
  EXPECT_EQ(testGetCapacity(int32_values.get()), unbounded_sequences_msg.int32_values.capacity());
  EXPECT_EQ(testGetSize(int32_values.get()), unbounded_sequences_msg.int32_values.size());

  const auto uint32_values =
    testGetMember(unbounded_sequences_ds, "uint32_values", unbounded_sequences_msg.uint32_values);
  EXPECT_EQ(testGetCapacity(uint32_values.get()), unbounded_sequences_msg.uint32_values.capacity());
  EXPECT_EQ(testGetSize(uint32_values.get()), unbounded_sequences_msg.uint32_values.size());

  const auto int64_values =
    testGetMember(unbounded_sequences_ds, "int64_values", unbounded_sequences_msg.int64_values);
  EXPECT_EQ(testGetCapacity(int64_values.get()), unbounded_sequences_msg.int64_values.capacity());
  EXPECT_EQ(testGetSize(int64_values.get()), unbounded_sequences_msg.int64_values.size());

  const auto uint64_values =
    testGetMember(unbounded_sequences_ds, "uint64_values", unbounded_sequences_msg.uint64_values);
  EXPECT_EQ(testGetCapacity(uint64_values.get()), unbounded_sequences_msg.uint64_values.capacity());
  EXPECT_EQ(testGetSize(uint64_values.get()), unbounded_sequences_msg.uint64_values.size());

  const auto string_values =
    testGetMember(unbounded_sequences_ds, "string_values", unbounded_sequences_msg.string_values);
  EXPECT_EQ(testGetCapacity(string_values.get()), unbounded_sequences_msg.string_values.capacity());
  EXPECT_EQ(testGetSize(string_values.get()), unbounded_sequences_msg.string_values.size());

  const auto basic_types_values =
    testGetMember(
    unbounded_sequences_ds, "basic_types_values",
    unbounded_sequences_msg.basic_types_values);
  EXPECT_EQ(
    testGetCapacity(basic_types_values.get()),
    unbounded_sequences_msg.basic_types_values.size());
  EXPECT_EQ(
    testGetSize(basic_types_values.get()),
    unbounded_sequences_msg.basic_types_values.size());

  const auto constants_values =
    testGetMember(
    unbounded_sequences_ds, "constants_values",
    unbounded_sequences_msg.constants_values);
  EXPECT_EQ(
    testGetCapacity(constants_values.get()),
    unbounded_sequences_msg.constants_values.capacity());
  EXPECT_EQ(
    testGetSize(constants_values.get()), unbounded_sequences_msg.constants_values.size());

  const auto defaults_values =
    testGetMember(
    unbounded_sequences_ds, "defaults_values",
    unbounded_sequences_msg.defaults_values);
  EXPECT_EQ(
    testGetCapacity(defaults_values.get()),
    unbounded_sequences_msg.defaults_values.capacity());
  EXPECT_EQ(
    testGetSize(defaults_values.get()), unbounded_sequences_msg.defaults_values.size());

  const auto bool_values_default =
    testGetMember(
    unbounded_sequences_ds, "bool_values_default",
    unbounded_sequences_msg.bool_values_default);
  EXPECT_EQ(
    testGetCapacity(bool_values_default.get()),
    unbounded_sequences_msg.bool_values_default.capacity());
  EXPECT_EQ(
    testGetSize(bool_values_default.get()),
    unbounded_sequences_msg.bool_values_default.size());

  const auto byte_values_default =
    testGetMember(
    unbounded_sequences_ds, "byte_values_default",
    unbounded_sequences_msg.byte_values_default);
  EXPECT_EQ(
    testGetCapacity(byte_values_default.get()),
    unbounded_sequences_msg.byte_values_default.capacity());
  EXPECT_EQ(
    testGetSize(byte_values_default.get()),
    unbounded_sequences_msg.byte_values_default.size());

  const auto char_values_default =
    testGetMember(
    unbounded_sequences_ds, "char_values_default",
    unbounded_sequences_msg.char_values_default);
  EXPECT_EQ(
    testGetCapacity(char_values_default.get()),
    unbounded_sequences_msg.char_values_default.capacity());
  EXPECT_EQ(
    testGetSize(char_values_default.get()),
    unbounded_sequences_msg.char_values_default.size());

  const auto float32_values_default =
    testGetMember(
    unbounded_sequences_ds, "float32_values_default",
    unbounded_sequences_msg.float32_values_default);
  EXPECT_EQ(
    testGetCapacity(float32_values_default.get()),
    unbounded_sequences_msg.float32_values_default.capacity());
  EXPECT_EQ(
    testGetSize(float32_values_default.get()),
    unbounded_sequences_msg.float32_values_default.size());

  const auto float64_values_default =
    testGetMember(
    unbounded_sequences_ds, "float64_values_default",
    unbounded_sequences_msg.float64_values_default);
  EXPECT_EQ(
    testGetCapacity(float64_values_default.get()),
    unbounded_sequences_msg.float64_values_default.capacity());
  EXPECT_EQ(
    testGetSize(float64_values_default.get()),
    unbounded_sequences_msg.float64_values_default.size());

  const auto int8_values_default =
    testGetMember(
    unbounded_sequences_ds, "int8_values_default",
    unbounded_sequences_msg.int8_values_default);
  EXPECT_EQ(
    testGetCapacity(int8_values_default.get()),
    unbounded_sequences_msg.int8_values_default.capacity());
  EXPECT_EQ(
    testGetSize(int8_values_default.get()),
    unbounded_sequences_msg.int8_values_default.size());

  const auto uint8_values_default =
    testGetMember(
    unbounded_sequences_ds, "uint8_values_default",
    unbounded_sequences_msg.uint8_values_default);
  EXPECT_EQ(
    testGetCapacity(uint8_values_default.get()),
    unbounded_sequences_msg.uint8_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint8_values_default.get()),
    unbounded_sequences_msg.uint8_values_default.size());

  const auto int16_values_default =
    testGetMember(
    unbounded_sequences_ds, "int16_values_default",
    unbounded_sequences_msg.int16_values_default);
  EXPECT_EQ(
    testGetCapacity(int16_values_default.get()),
    unbounded_sequences_msg.int16_values_default.size());
  EXPECT_EQ(
    testGetSize(int16_values_default.get()),
    unbounded_sequences_msg.int16_values_default.size());

  const auto uint16_values_default =
    testGetMember(
    unbounded_sequences_ds, "uint16_values_default",
    unbounded_sequences_msg.uint16_values_default);
  EXPECT_EQ(
    testGetCapacity(uint16_values_default.get()),
    unbounded_sequences_msg.uint16_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint16_values_default.get()),
    unbounded_sequences_msg.uint16_values_default.size());

  const auto int32_values_default =
    testGetMember(
    unbounded_sequences_ds, "int32_values_default",
    unbounded_sequences_msg.int32_values_default);
  EXPECT_EQ(
    testGetCapacity(int32_values_default.get()),
    unbounded_sequences_msg.int32_values_default.capacity());
  EXPECT_EQ(
    testGetSize(int32_values_default.get()),
    unbounded_sequences_msg.int32_values_default.size());

  const auto uint32_values_default =
    testGetMember(
    unbounded_sequences_ds, "uint32_values_default",
    unbounded_sequences_msg.uint32_values_default);
  EXPECT_EQ(
    testGetCapacity(uint32_values_default.get()),
    unbounded_sequences_msg.uint32_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint32_values_default.get()),
    unbounded_sequences_msg.uint32_values_default.size());

  const auto int64_values_default =
    testGetMember(
    unbounded_sequences_ds, "int64_values_default",
    unbounded_sequences_msg.int64_values_default);
  EXPECT_EQ(
    testGetCapacity(int64_values_default.get()),
    unbounded_sequences_msg.int64_values_default.capacity());
  EXPECT_EQ(
    testGetSize(int64_values_default.get()),
    unbounded_sequences_msg.int64_values_default.size());

  const auto uint64_values_default =
    testGetMember(
    unbounded_sequences_ds, "uint64_values_default",
    unbounded_sequences_msg.uint64_values_default);
  EXPECT_EQ(
    testGetCapacity(uint64_values_default.get()),
    unbounded_sequences_msg.uint64_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint64_values_default.get()),
    unbounded_sequences_msg.uint64_values_default.size());

  const auto string_values_default =
    testGetMember(
    unbounded_sequences_ds, "string_values_default",
    unbounded_sequences_msg.string_values_default);
  EXPECT_EQ(
    testGetCapacity(string_values_default.get()),
    unbounded_sequences_msg.string_values_default.capacity());
  EXPECT_EQ(
    testGetSize(string_values_default.get()),
    unbounded_sequences_msg.string_values_default.size());

  const auto alignment_check =
    testGetMember(
    unbounded_sequences_ds, "alignment_check",
    unbounded_sequences_msg.alignment_check);
}

TEST_F(TestTypekit, BoundedSequences)
{
  EXPECT_EQ(bounded_sequences_ds->getMemberNames().size(), 32);

  const auto bool_values =
    testGetMember(bounded_sequences_ds, "bool_values", bounded_sequences_msg.bool_values);
  EXPECT_EQ(testGetCapacity(bool_values.get()), bounded_sequences_msg.bool_values.capacity());
  EXPECT_EQ(testGetSize(bool_values.get()), bounded_sequences_msg.bool_values.size());

  const auto byte_values =
    testGetMember(bounded_sequences_ds, "byte_values", bounded_sequences_msg.byte_values);
  EXPECT_EQ(testGetCapacity(byte_values.get()), bounded_sequences_msg.byte_values.capacity());
  EXPECT_EQ(testGetSize(byte_values.get()), bounded_sequences_msg.byte_values.size());

  const auto char_values =
    testGetMember(bounded_sequences_ds, "char_values", bounded_sequences_msg.char_values);
  EXPECT_EQ(testGetCapacity(char_values.get()), bounded_sequences_msg.char_values.capacity());
  EXPECT_EQ(testGetSize(char_values.get()), bounded_sequences_msg.char_values.size());

  const auto float32_values =
    testGetMember(bounded_sequences_ds, "float32_values", bounded_sequences_msg.float32_values);
  EXPECT_EQ(testGetCapacity(float32_values.get()), bounded_sequences_msg.float32_values.capacity());
  EXPECT_EQ(testGetSize(float32_values.get()), bounded_sequences_msg.float32_values.size());

  const auto float64_values =
    testGetMember(bounded_sequences_ds, "float64_values", bounded_sequences_msg.float64_values);
  EXPECT_EQ(testGetCapacity(float64_values.get()), bounded_sequences_msg.float64_values.capacity());
  EXPECT_EQ(testGetSize(float64_values.get()), bounded_sequences_msg.float64_values.size());

  const auto int8_values =
    testGetMember(bounded_sequences_ds, "int8_values", bounded_sequences_msg.int8_values);
  EXPECT_EQ(testGetCapacity(int8_values.get()), bounded_sequences_msg.int8_values.capacity());
  EXPECT_EQ(testGetSize(int8_values.get()), bounded_sequences_msg.int8_values.size());

  const auto uint8_values =
    testGetMember(bounded_sequences_ds, "uint8_values", bounded_sequences_msg.uint8_values);
  EXPECT_EQ(testGetCapacity(uint8_values.get()), bounded_sequences_msg.uint8_values.capacity());
  EXPECT_EQ(testGetSize(uint8_values.get()), bounded_sequences_msg.uint8_values.size());

  const auto int16_values =
    testGetMember(bounded_sequences_ds, "int16_values", bounded_sequences_msg.int16_values);
  EXPECT_EQ(testGetCapacity(int16_values.get()), bounded_sequences_msg.int16_values.capacity());
  EXPECT_EQ(testGetSize(int16_values.get()), bounded_sequences_msg.int16_values.size());

  const auto uint16_values =
    testGetMember(bounded_sequences_ds, "uint16_values", bounded_sequences_msg.uint16_values);
  EXPECT_EQ(testGetCapacity(uint16_values.get()), bounded_sequences_msg.uint16_values.capacity());
  EXPECT_EQ(testGetSize(uint16_values.get()), bounded_sequences_msg.uint16_values.size());

  const auto int32_values =
    testGetMember(bounded_sequences_ds, "int32_values", bounded_sequences_msg.int32_values);
  EXPECT_EQ(testGetCapacity(int32_values.get()), bounded_sequences_msg.int32_values.capacity());
  EXPECT_EQ(testGetSize(int32_values.get()), bounded_sequences_msg.int32_values.size());

  const auto uint32_values =
    testGetMember(bounded_sequences_ds, "uint32_values", bounded_sequences_msg.uint32_values);
  EXPECT_EQ(testGetCapacity(uint32_values.get()), bounded_sequences_msg.uint32_values.capacity());
  EXPECT_EQ(testGetSize(uint32_values.get()), bounded_sequences_msg.uint32_values.size());

  const auto int64_values =
    testGetMember(bounded_sequences_ds, "int64_values", bounded_sequences_msg.int64_values);
  EXPECT_EQ(testGetCapacity(int64_values.get()), bounded_sequences_msg.int64_values.capacity());
  EXPECT_EQ(testGetSize(int64_values.get()), bounded_sequences_msg.int64_values.size());

  const auto uint64_values =
    testGetMember(bounded_sequences_ds, "uint64_values", bounded_sequences_msg.uint64_values);
  EXPECT_EQ(testGetCapacity(uint64_values.get()), bounded_sequences_msg.uint64_values.capacity());
  EXPECT_EQ(testGetSize(uint64_values.get()), bounded_sequences_msg.uint64_values.size());

  const auto string_values =
    testGetMember(bounded_sequences_ds, "string_values", bounded_sequences_msg.string_values);
  EXPECT_EQ(testGetCapacity(string_values.get()), bounded_sequences_msg.string_values.capacity());
  EXPECT_EQ(testGetSize(string_values.get()), bounded_sequences_msg.string_values.size());

  const auto basic_types_values =
    testGetMember(
    bounded_sequences_ds, "basic_types_values",
    bounded_sequences_msg.basic_types_values);
  EXPECT_EQ(
    testGetCapacity(basic_types_values.get()),
    bounded_sequences_msg.basic_types_values.size());
  EXPECT_EQ(testGetSize(basic_types_values.get()), bounded_sequences_msg.basic_types_values.size());

  const auto constants_values =
    testGetMember(bounded_sequences_ds, "constants_values", bounded_sequences_msg.constants_values);
  EXPECT_EQ(
    testGetCapacity(constants_values.get()),
    bounded_sequences_msg.constants_values.capacity());
  EXPECT_EQ(testGetSize(constants_values.get()), bounded_sequences_msg.constants_values.size());

  const auto defaults_values =
    testGetMember(bounded_sequences_ds, "defaults_values", bounded_sequences_msg.defaults_values);
  EXPECT_EQ(
    testGetCapacity(defaults_values.get()),
    bounded_sequences_msg.defaults_values.capacity());
  EXPECT_EQ(testGetSize(defaults_values.get()), bounded_sequences_msg.defaults_values.size());

  const auto bool_values_default =
    testGetMember(
    bounded_sequences_ds, "bool_values_default",
    bounded_sequences_msg.bool_values_default);
  EXPECT_EQ(
    testGetCapacity(bool_values_default.get()),
    bounded_sequences_msg.bool_values_default.capacity());
  EXPECT_EQ(
    testGetSize(bool_values_default.get()),
    bounded_sequences_msg.bool_values_default.size());

  const auto byte_values_default =
    testGetMember(
    bounded_sequences_ds, "byte_values_default",
    bounded_sequences_msg.byte_values_default);
  EXPECT_EQ(
    testGetCapacity(byte_values_default.get()),
    bounded_sequences_msg.byte_values_default.capacity());
  EXPECT_EQ(
    testGetSize(byte_values_default.get()),
    bounded_sequences_msg.byte_values_default.size());

  const auto char_values_default =
    testGetMember(
    bounded_sequences_ds, "char_values_default",
    bounded_sequences_msg.char_values_default);
  EXPECT_EQ(
    testGetCapacity(char_values_default.get()),
    bounded_sequences_msg.char_values_default.capacity());
  EXPECT_EQ(
    testGetSize(char_values_default.get()),
    bounded_sequences_msg.char_values_default.size());

  const auto float32_values_default =
    testGetMember(
    bounded_sequences_ds, "float32_values_default",
    bounded_sequences_msg.float32_values_default);
  EXPECT_EQ(
    testGetCapacity(float32_values_default.get()),
    bounded_sequences_msg.float32_values_default.capacity());
  EXPECT_EQ(
    testGetSize(float32_values_default.get()),
    bounded_sequences_msg.float32_values_default.size());

  const auto float64_values_default =
    testGetMember(
    bounded_sequences_ds, "float64_values_default",
    bounded_sequences_msg.float64_values_default);
  EXPECT_EQ(
    testGetCapacity(float64_values_default.get()),
    bounded_sequences_msg.float64_values_default.capacity());
  EXPECT_EQ(
    testGetSize(float64_values_default.get()),
    bounded_sequences_msg.float64_values_default.size());

  const auto int8_values_default =
    testGetMember(
    bounded_sequences_ds, "int8_values_default",
    bounded_sequences_msg.int8_values_default);
  EXPECT_EQ(
    testGetCapacity(int8_values_default.get()),
    bounded_sequences_msg.int8_values_default.capacity());
  EXPECT_EQ(
    testGetSize(int8_values_default.get()),
    bounded_sequences_msg.int8_values_default.size());

  const auto uint8_values_default =
    testGetMember(
    bounded_sequences_ds, "uint8_values_default",
    bounded_sequences_msg.uint8_values_default);
  EXPECT_EQ(
    testGetCapacity(uint8_values_default.get()),
    bounded_sequences_msg.uint8_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint8_values_default.get()),
    bounded_sequences_msg.uint8_values_default.size());

  const auto int16_values_default =
    testGetMember(
    bounded_sequences_ds, "int16_values_default",
    bounded_sequences_msg.int16_values_default);
  EXPECT_EQ(
    testGetCapacity(int16_values_default.get()),
    bounded_sequences_msg.int16_values_default.size());
  EXPECT_EQ(
    testGetSize(int16_values_default.get()),
    bounded_sequences_msg.int16_values_default.size());

  const auto uint16_values_default =
    testGetMember(
    bounded_sequences_ds, "uint16_values_default",
    bounded_sequences_msg.uint16_values_default);
  EXPECT_EQ(
    testGetCapacity(uint16_values_default.get()),
    bounded_sequences_msg.uint16_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint16_values_default.get()),
    bounded_sequences_msg.uint16_values_default.size());

  const auto int32_values_default =
    testGetMember(
    bounded_sequences_ds, "int32_values_default",
    bounded_sequences_msg.int32_values_default);
  EXPECT_EQ(
    testGetCapacity(int32_values_default.get()),
    bounded_sequences_msg.int32_values_default.capacity());
  EXPECT_EQ(
    testGetSize(int32_values_default.get()),
    bounded_sequences_msg.int32_values_default.size());

  const auto uint32_values_default =
    testGetMember(
    bounded_sequences_ds, "uint32_values_default",
    bounded_sequences_msg.uint32_values_default);
  EXPECT_EQ(
    testGetCapacity(uint32_values_default.get()),
    bounded_sequences_msg.uint32_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint32_values_default.get()),
    bounded_sequences_msg.uint32_values_default.size());

  const auto int64_values_default =
    testGetMember(
    bounded_sequences_ds, "int64_values_default",
    bounded_sequences_msg.int64_values_default);
  EXPECT_EQ(
    testGetCapacity(int64_values_default.get()),
    bounded_sequences_msg.int64_values_default.capacity());
  EXPECT_EQ(
    testGetSize(int64_values_default.get()),
    bounded_sequences_msg.int64_values_default.size());

  const auto uint64_values_default =
    testGetMember(
    bounded_sequences_ds, "uint64_values_default",
    bounded_sequences_msg.uint64_values_default);
  EXPECT_EQ(
    testGetCapacity(uint64_values_default.get()),
    bounded_sequences_msg.uint64_values_default.capacity());
  EXPECT_EQ(
    testGetSize(uint64_values_default.get()),
    bounded_sequences_msg.uint64_values_default.size());

  const auto string_values_default =
    testGetMember(
    bounded_sequences_ds, "string_values_default",
    bounded_sequences_msg.string_values_default);
  EXPECT_EQ(
    testGetCapacity(string_values_default.get()),
    bounded_sequences_msg.string_values_default.capacity());
  EXPECT_EQ(
    testGetSize(string_values_default.get()),
    bounded_sequences_msg.string_values_default.size());

  const auto alignment_check =
    testGetMember(bounded_sequences_ds, "alignment_check", bounded_sequences_msg.alignment_check);
}

TEST_F(TestTypekit, MultiNested)
{
  EXPECT_EQ(multi_nested_ds->getMemberNames().size(), 9);

  const auto array_of_arrays =
    testGetMember(
    multi_nested_ds, "array_of_arrays",
    multi_nested_msg.array_of_arrays);
  EXPECT_EQ(
    testGetCapacity(array_of_arrays.get()),
    multi_nested_msg.array_of_arrays.size());
  EXPECT_EQ(
    testGetSize(array_of_arrays.get()),
    multi_nested_msg.array_of_arrays.size());

  const auto array_of_bounded_sequences =
    testGetMember(
    multi_nested_ds, "array_of_bounded_sequences",
    multi_nested_msg.array_of_bounded_sequences);
  EXPECT_EQ(
    testGetCapacity(array_of_bounded_sequences.get()),
    multi_nested_msg.array_of_bounded_sequences.size());
  EXPECT_EQ(
    testGetSize(array_of_bounded_sequences.get()),
    multi_nested_msg.array_of_bounded_sequences.size());

  const auto array_of_unbounded_sequences =
    testGetMember(
    multi_nested_ds, "array_of_unbounded_sequences",
    multi_nested_msg.array_of_unbounded_sequences);
  EXPECT_EQ(
    testGetCapacity(array_of_unbounded_sequences.get()),
    multi_nested_msg.array_of_unbounded_sequences.size());
  EXPECT_EQ(
    testGetSize(array_of_unbounded_sequences.get()),
    multi_nested_msg.array_of_unbounded_sequences.size());

  const auto bounded_sequence_of_arrays =
    testGetMember(
    multi_nested_ds, "bounded_sequence_of_arrays",
    multi_nested_msg.bounded_sequence_of_arrays);
  EXPECT_EQ(
    testGetCapacity(bounded_sequence_of_arrays.get()),
    multi_nested_msg.bounded_sequence_of_arrays.capacity());
  EXPECT_EQ(
    testGetSize(bounded_sequence_of_arrays.get()),
    multi_nested_msg.bounded_sequence_of_arrays.size());

  const auto bounded_sequence_of_bounded_sequences =
    testGetMember(
    multi_nested_ds, "bounded_sequence_of_bounded_sequences",
    multi_nested_msg.bounded_sequence_of_bounded_sequences);
  EXPECT_EQ(
    testGetCapacity(bounded_sequence_of_bounded_sequences.get()),
    multi_nested_msg.bounded_sequence_of_bounded_sequences.capacity());
  EXPECT_EQ(
    testGetSize(bounded_sequence_of_bounded_sequences.get()),
    multi_nested_msg.bounded_sequence_of_bounded_sequences.size());

  const auto bounded_sequence_of_unbounded_sequences =
    testGetMember(
    multi_nested_ds, "bounded_sequence_of_unbounded_sequences",
    multi_nested_msg.bounded_sequence_of_unbounded_sequences);
  EXPECT_EQ(
    testGetCapacity(bounded_sequence_of_unbounded_sequences.get()),
    multi_nested_msg.bounded_sequence_of_unbounded_sequences.capacity());
  EXPECT_EQ(
    testGetSize(bounded_sequence_of_unbounded_sequences.get()),
    multi_nested_msg.bounded_sequence_of_unbounded_sequences.size());

  const auto unbounded_sequence_of_arrays =
    testGetMember(
    multi_nested_ds, "unbounded_sequence_of_arrays",
    multi_nested_msg.unbounded_sequence_of_arrays);
  EXPECT_EQ(
    testGetCapacity(unbounded_sequence_of_arrays.get()),
    multi_nested_msg.unbounded_sequence_of_arrays.capacity());
  EXPECT_EQ(
    testGetSize(unbounded_sequence_of_arrays.get()),
    multi_nested_msg.unbounded_sequence_of_arrays.size());

  const auto unbounded_sequence_of_bounded_sequences =
    testGetMember(
    multi_nested_ds, "unbounded_sequence_of_bounded_sequences",
    multi_nested_msg.unbounded_sequence_of_bounded_sequences);
  EXPECT_EQ(
    testGetCapacity(unbounded_sequence_of_bounded_sequences.get()),
    multi_nested_msg.unbounded_sequence_of_bounded_sequences.capacity());
  EXPECT_EQ(
    testGetSize(unbounded_sequence_of_bounded_sequences.get()),
    multi_nested_msg.unbounded_sequence_of_bounded_sequences.size());

  const auto unbounded_sequence_of_unbounded_sequences =
    testGetMember(
    multi_nested_ds, "unbounded_sequence_of_unbounded_sequences",
    multi_nested_msg.unbounded_sequence_of_unbounded_sequences);
  EXPECT_EQ(
    testGetCapacity(unbounded_sequence_of_unbounded_sequences.get()),
    multi_nested_msg.unbounded_sequence_of_unbounded_sequences.capacity());
  EXPECT_EQ(
    testGetSize(unbounded_sequence_of_unbounded_sequences.get()),
    multi_nested_msg.unbounded_sequence_of_unbounded_sequences.size());
}

TEST_F(TestTypekit, Nested)
{
  EXPECT_EQ(nested_ds->getMemberNames().size(), 1);
  const auto basic_types_value =
    testGetMember(nested_ds, "basic_types_value", nested_msg.basic_types_value);
  ASSERT_TRUE(basic_types_value != nullptr);

  EXPECT_EQ(basic_types_value->getMemberNames().size(), 13);
  testGetMember(basic_types_value, "bool_value", nested_msg.basic_types_value.bool_value);
  testGetMember(basic_types_value, "byte_value", nested_msg.basic_types_value.byte_value);
  testGetMember(basic_types_value, "char_value", nested_msg.basic_types_value.char_value);
  testGetMember(basic_types_value, "float32_value", nested_msg.basic_types_value.float32_value);
  testGetMember(basic_types_value, "float64_value", nested_msg.basic_types_value.float64_value);
  testGetMember(basic_types_value, "int8_value", nested_msg.basic_types_value.int8_value);
  testGetMember(basic_types_value, "uint8_value", nested_msg.basic_types_value.uint8_value);
  testGetMember(basic_types_value, "int16_value", nested_msg.basic_types_value.int16_value);
  testGetMember(basic_types_value, "uint16_value", nested_msg.basic_types_value.uint16_value);
  testGetMember(basic_types_value, "int32_value", nested_msg.basic_types_value.int32_value);
  testGetMember(basic_types_value, "uint32_value", nested_msg.basic_types_value.uint32_value);
  testGetMember(basic_types_value, "int64_value", nested_msg.basic_types_value.int64_value);
  testGetMember(basic_types_value, "uint64_value", nested_msg.basic_types_value.uint64_value);
}

TEST_F(TestTypekit, Strings)
{
  EXPECT_EQ(strings_ds->getMemberNames().size(), 12);

  const auto string_value =
    testGetMember(
    strings_ds, "string_value",
    strings_msg.string_value);
  EXPECT_EQ(
    testGetCapacity(string_value.get()),
    strings_msg.string_value.capacity());
  EXPECT_EQ(
    testGetSize(string_value.get()),
    strings_msg.string_value.size());

  const auto string_value_default1 =
    testGetMember(
    strings_ds, "string_value_default1",
    strings_msg.string_value_default1);
  EXPECT_EQ(
    testGetCapacity(string_value_default1.get()),
    strings_msg.string_value_default1.capacity());
  EXPECT_EQ(
    testGetSize(string_value_default1.get()),
    strings_msg.string_value_default1.size());

  const auto string_value_default2 =
    testGetMember(
    strings_ds, "string_value_default2",
    strings_msg.string_value_default2);
  EXPECT_EQ(
    testGetCapacity(string_value_default2.get()),
    strings_msg.string_value_default2.capacity());
  EXPECT_EQ(
    testGetSize(string_value_default2.get()),
    strings_msg.string_value_default2.size());

  const auto string_value_default3 =
    testGetMember(
    strings_ds, "string_value_default3",
    strings_msg.string_value_default3);
  EXPECT_EQ(
    testGetCapacity(string_value_default3.get()),
    strings_msg.string_value_default3.capacity());
  EXPECT_EQ(
    testGetSize(string_value_default3.get()),
    strings_msg.string_value_default3.size());

  const auto string_value_default4 =
    testGetMember(
    strings_ds, "string_value_default4",
    strings_msg.string_value_default4);
  EXPECT_EQ(
    testGetCapacity(string_value_default4.get()),
    strings_msg.string_value_default4.capacity());
  EXPECT_EQ(
    testGetSize(string_value_default4.get()),
    strings_msg.string_value_default4.size());

  const auto string_value_default5 =
    testGetMember(
    strings_ds, "string_value_default5",
    strings_msg.string_value_default5);
  EXPECT_EQ(
    testGetCapacity(string_value_default5.get()),
    strings_msg.string_value_default5.capacity());
  EXPECT_EQ(
    testGetSize(string_value_default5.get()),
    strings_msg.string_value_default5.size());

  const auto bounded_string_value =
    testGetMember(
    strings_ds, "bounded_string_value",
    strings_msg.bounded_string_value);
  EXPECT_EQ(
    testGetCapacity(bounded_string_value.get()),
    strings_msg.bounded_string_value.capacity());
  EXPECT_EQ(
    testGetSize(bounded_string_value.get()),
    strings_msg.bounded_string_value.size());

  const auto bounded_string_value_default1 =
    testGetMember(
    strings_ds, "bounded_string_value_default1",
    strings_msg.bounded_string_value_default1);
  EXPECT_EQ(
    testGetCapacity(bounded_string_value_default1.get()),
    strings_msg.bounded_string_value_default1.capacity());
  EXPECT_EQ(
    testGetSize(bounded_string_value_default1.get()),
    strings_msg.bounded_string_value_default1.size());

  const auto bounded_string_value_default2 =
    testGetMember(
    strings_ds, "bounded_string_value_default2",
    strings_msg.bounded_string_value_default2);
  EXPECT_EQ(
    testGetCapacity(bounded_string_value_default2.get()),
    strings_msg.bounded_string_value_default2.capacity());
  EXPECT_EQ(
    testGetSize(bounded_string_value_default2.get()),
    strings_msg.bounded_string_value_default2.size());

  const auto bounded_string_value_default3 =
    testGetMember(
    strings_ds, "bounded_string_value_default3",
    strings_msg.bounded_string_value_default3);
  EXPECT_EQ(
    testGetCapacity(bounded_string_value_default3.get()),
    strings_msg.bounded_string_value_default3.capacity());
  EXPECT_EQ(
    testGetSize(bounded_string_value_default3.get()),
    strings_msg.bounded_string_value_default3.size());

  const auto bounded_string_value_default4 =
    testGetMember(
    strings_ds, "bounded_string_value_default4",
    strings_msg.bounded_string_value_default4);
  EXPECT_EQ(
    testGetCapacity(bounded_string_value_default4.get()),
    strings_msg.bounded_string_value_default4.capacity());
  EXPECT_EQ(
    testGetSize(bounded_string_value_default4.get()),
    strings_msg.bounded_string_value_default4.size());

  const auto bounded_string_value_default5 =
    testGetMember(
    strings_ds, "bounded_string_value_default5",
    strings_msg.bounded_string_value_default5);
  EXPECT_EQ(
    testGetCapacity(bounded_string_value_default5.get()),
    strings_msg.bounded_string_value_default5.capacity());
  EXPECT_EQ(
    testGetSize(bounded_string_value_default5.get()),
    strings_msg.bounded_string_value_default5.size());
}

TEST_F(TestTypekit, WStrings)
{
#if test_msgs_VERSION_GTE(0, 8, 0)
  EXPECT_EQ(wstrings_ds->getMemberNames().size(), 7);
#else
  EXPECT_EQ(wstrings_ds->getMemberNames().size(), 4);
#endif

  const auto wstring_value =
    testGetMember(
    wstrings_ds, "wstring_value",
    wstrings_msg.wstring_value);
  EXPECT_EQ(
    testGetCapacity(wstring_value.get()),
    wstrings_msg.wstring_value.capacity());
  EXPECT_EQ(
    testGetSize(wstring_value.get()),
    wstrings_msg.wstring_value.size());

  /* Fields have only been added in ROS eloquent (test_msgs >= 0.8.0) */
#if test_msgs_VERSION_GTE(0, 8, 0)
  const auto wstring_value_default1 =
    testGetMember(
    wstrings_ds, "wstring_value_default1",
    wstrings_msg.wstring_value_default1);
  EXPECT_EQ(
    testGetCapacity(wstring_value_default1.get()),
    wstrings_msg.wstring_value_default1.capacity());
  EXPECT_EQ(
    testGetSize(wstring_value_default1.get()),
    wstrings_msg.wstring_value_default1.size());

  const auto wstring_value_default2 =
    testGetMember(
    wstrings_ds, "wstring_value_default2",
    wstrings_msg.wstring_value_default2);
  EXPECT_EQ(
    testGetCapacity(wstring_value_default2.get()),
    wstrings_msg.wstring_value_default2.capacity());
  EXPECT_EQ(
    testGetSize(wstring_value_default2.get()),
    wstrings_msg.wstring_value_default2.size());

  const auto wstring_value_default3 =
    testGetMember(
    wstrings_ds, "wstring_value_default3",
    wstrings_msg.wstring_value_default3);
  EXPECT_EQ(
    testGetCapacity(wstring_value_default3.get()),
    wstrings_msg.wstring_value_default3.capacity());
  EXPECT_EQ(
    testGetSize(wstring_value_default3.get()),
    wstrings_msg.wstring_value_default3.size());
#endif  // test_msgs_VERSION_GTE(0, 8, 0)

  const auto array_of_wstrings =
    testGetMember(
    wstrings_ds, "array_of_wstrings",
    wstrings_msg.array_of_wstrings);
  EXPECT_EQ(
    testGetCapacity(array_of_wstrings.get()),
    wstrings_msg.array_of_wstrings.size());
  EXPECT_EQ(
    testGetSize(array_of_wstrings.get()),
    wstrings_msg.array_of_wstrings.size());

  const auto bounded_sequence_of_wstrings =
    testGetMember(
    wstrings_ds, "bounded_sequence_of_wstrings",
    wstrings_msg.bounded_sequence_of_wstrings);
  EXPECT_EQ(
    testGetCapacity(bounded_sequence_of_wstrings.get()),
    wstrings_msg.bounded_sequence_of_wstrings.capacity());
  EXPECT_EQ(
    testGetSize(bounded_sequence_of_wstrings.get()),
    wstrings_msg.bounded_sequence_of_wstrings.size());

  const auto unbounded_sequence_of_wstrings =
    testGetMember(
    wstrings_ds, "unbounded_sequence_of_wstrings",
    wstrings_msg.unbounded_sequence_of_wstrings);
  EXPECT_EQ(
    testGetCapacity(unbounded_sequence_of_wstrings.get()),
    wstrings_msg.unbounded_sequence_of_wstrings.capacity());
  EXPECT_EQ(
    testGetSize(unbounded_sequence_of_wstrings.get()),
    wstrings_msg.unbounded_sequence_of_wstrings.size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // __os_init() must be called after testing::InitGoogleTest(&argc, argv) because this function
  // removes Google Test flags from argc/argv.
  __os_init(argc, argv);

  const auto env = ::testing::AddGlobalTestEnvironment(new TestTypekitEnvironment);
  int ret = RUN_ALL_TESTS();

  __os_exit();
  return ret;
}
