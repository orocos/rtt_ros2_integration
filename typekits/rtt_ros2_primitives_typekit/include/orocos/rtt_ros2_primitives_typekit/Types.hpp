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

#ifndef OROCOS__RTT_ROS2_PRIMITIVES_TYPEKIT__TYPES_HPP_
#define OROCOS__RTT_ROS2_PRIMITIVES_TYPEKIT__TYPES_HPP_

#include <cstdint>
#include <string>

#include "rtt/internal/AssignCommand.hpp"
#include "rtt/internal/DataSources.hpp"
#include "rtt/Attribute.hpp"
#include "rtt/InputPort.hpp"
#include "rtt/OutputPort.hpp"
#include "rtt/Property.hpp"

// bool
extern template class RTT::internal::DataSource<bool>;
extern template class RTT::internal::AssignableDataSource<bool>;
extern template class RTT::internal::AssignCommand<bool>;
extern template class RTT::internal::ValueDataSource<bool>;
extern template class RTT::internal::ConstantDataSource<bool>;
extern template class RTT::internal::ReferenceDataSource<bool>;
extern template class RTT::OutputPort<bool>;
extern template class RTT::InputPort<bool>;
extern template class RTT::Property<bool>;
extern template class RTT::Attribute<bool>;
extern template class RTT::Constant<bool>;

// float32
extern template class RTT::internal::DataSource<float>;
extern template class RTT::internal::AssignableDataSource<float>;
extern template class RTT::internal::AssignCommand<float>;
extern template class RTT::internal::ValueDataSource<float>;
extern template class RTT::internal::ConstantDataSource<float>;
extern template class RTT::internal::ReferenceDataSource<float>;
extern template class RTT::OutputPort<float>;
extern template class RTT::InputPort<float>;
extern template class RTT::Property<float>;
extern template class RTT::Attribute<float>;
extern template class RTT::Constant<float>;

// float64
extern template class RTT::internal::DataSource<double>;
extern template class RTT::internal::AssignableDataSource<double>;
extern template class RTT::internal::AssignCommand<double>;
extern template class RTT::internal::ValueDataSource<double>;
extern template class RTT::internal::ConstantDataSource<double>;
extern template class RTT::internal::ReferenceDataSource<double>;
extern template class RTT::OutputPort<double>;
extern template class RTT::InputPort<double>;
extern template class RTT::Property<double>;
extern template class RTT::Attribute<double>;
extern template class RTT::Constant<double>;

// int8
extern template class RTT::internal::DataSource<int8_t>;
extern template class RTT::internal::AssignableDataSource<int8_t>;
extern template class RTT::internal::AssignCommand<int8_t>;
extern template class RTT::internal::ValueDataSource<int8_t>;
extern template class RTT::internal::ConstantDataSource<int8_t>;
extern template class RTT::internal::ReferenceDataSource<int8_t>;
extern template class RTT::OutputPort<int8_t>;
extern template class RTT::InputPort<int8_t>;
extern template class RTT::Property<int8_t>;
extern template class RTT::Attribute<int8_t>;
extern template class RTT::Constant<int8_t>;

// byte + char + uint8
extern template class RTT::internal::DataSource<uint8_t>;
extern template class RTT::internal::AssignableDataSource<uint8_t>;
extern template class RTT::internal::AssignCommand<uint8_t>;
extern template class RTT::internal::ValueDataSource<uint8_t>;
extern template class RTT::internal::ConstantDataSource<uint8_t>;
extern template class RTT::internal::ReferenceDataSource<uint8_t>;
extern template class RTT::OutputPort<uint8_t>;
extern template class RTT::InputPort<uint8_t>;
extern template class RTT::Property<uint8_t>;
extern template class RTT::Attribute<uint8_t>;
extern template class RTT::Constant<uint8_t>;

// int16
extern template class RTT::internal::DataSource<int16_t>;
extern template class RTT::internal::AssignableDataSource<int16_t>;
extern template class RTT::internal::AssignCommand<int16_t>;
extern template class RTT::internal::ValueDataSource<int16_t>;
extern template class RTT::internal::ConstantDataSource<int16_t>;
extern template class RTT::internal::ReferenceDataSource<int16_t>;
extern template class RTT::OutputPort<int16_t>;
extern template class RTT::InputPort<int16_t>;
extern template class RTT::Property<int16_t>;
extern template class RTT::Attribute<int16_t>;
extern template class RTT::Constant<int16_t>;

// uint16
extern template class RTT::internal::DataSource<uint16_t>;
extern template class RTT::internal::AssignableDataSource<uint16_t>;
extern template class RTT::internal::AssignCommand<uint16_t>;
extern template class RTT::internal::ValueDataSource<uint16_t>;
extern template class RTT::internal::ConstantDataSource<uint16_t>;
extern template class RTT::internal::ReferenceDataSource<uint16_t>;
extern template class RTT::OutputPort<uint16_t>;
extern template class RTT::InputPort<uint16_t>;
extern template class RTT::Property<uint16_t>;
extern template class RTT::Attribute<uint16_t>;
extern template class RTT::Constant<uint16_t>;

// int32
extern template class RTT::internal::DataSource<int32_t>;
extern template class RTT::internal::AssignableDataSource<int32_t>;
extern template class RTT::internal::AssignCommand<int32_t>;
extern template class RTT::internal::ValueDataSource<int32_t>;
extern template class RTT::internal::ConstantDataSource<int32_t>;
extern template class RTT::internal::ReferenceDataSource<int32_t>;
extern template class RTT::OutputPort<int32_t>;
extern template class RTT::InputPort<int32_t>;
extern template class RTT::Property<int32_t>;
extern template class RTT::Attribute<int32_t>;
extern template class RTT::Constant<int32_t>;

// uint32
extern template class RTT::internal::DataSource<uint32_t>;
extern template class RTT::internal::AssignableDataSource<uint32_t>;
extern template class RTT::internal::AssignCommand<uint32_t>;
extern template class RTT::internal::ValueDataSource<uint32_t>;
extern template class RTT::internal::ConstantDataSource<uint32_t>;
extern template class RTT::internal::ReferenceDataSource<uint32_t>;
extern template class RTT::OutputPort<uint32_t>;
extern template class RTT::InputPort<uint32_t>;
extern template class RTT::Property<uint32_t>;
extern template class RTT::Attribute<uint32_t>;
extern template class RTT::Constant<uint32_t>;

// int64
extern template class RTT::internal::DataSource<int64_t>;
extern template class RTT::internal::AssignableDataSource<int64_t>;
extern template class RTT::internal::AssignCommand<int64_t>;
extern template class RTT::internal::ValueDataSource<int64_t>;
extern template class RTT::internal::ConstantDataSource<int64_t>;
extern template class RTT::internal::ReferenceDataSource<int64_t>;
extern template class RTT::OutputPort<int64_t>;
extern template class RTT::InputPort<int64_t>;
extern template class RTT::Property<int64_t>;
extern template class RTT::Attribute<int64_t>;
extern template class RTT::Constant<int64_t>;

// uint64
extern template class RTT::internal::DataSource<uint64_t>;
extern template class RTT::internal::AssignableDataSource<uint64_t>;
extern template class RTT::internal::AssignCommand<uint64_t>;
extern template class RTT::internal::ValueDataSource<uint64_t>;
extern template class RTT::internal::ConstantDataSource<uint64_t>;
extern template class RTT::internal::ReferenceDataSource<uint64_t>;
extern template class RTT::OutputPort<uint64_t>;
extern template class RTT::InputPort<uint64_t>;
extern template class RTT::Property<uint64_t>;
extern template class RTT::Attribute<uint64_t>;
extern template class RTT::Constant<uint64_t>;

// string
extern template class RTT::internal::DataSource<std::string>;
extern template class RTT::internal::AssignableDataSource<std::string>;
extern template class RTT::internal::AssignCommand<std::string>;
extern template class RTT::internal::ValueDataSource<std::string>;
extern template class RTT::internal::ConstantDataSource<std::string>;
extern template class RTT::internal::ReferenceDataSource<std::string>;
extern template class RTT::OutputPort<std::string>;
extern template class RTT::InputPort<std::string>;
extern template class RTT::Property<std::string>;
extern template class RTT::Attribute<std::string>;
extern template class RTT::Constant<std::string>;

// wstring
extern template class RTT::internal::DataSource<std::u16string>;
extern template class RTT::internal::AssignableDataSource<std::u16string>;
extern template class RTT::internal::AssignCommand<std::u16string>;
extern template class RTT::internal::ValueDataSource<std::u16string>;
extern template class RTT::internal::ConstantDataSource<std::u16string>;
extern template class RTT::internal::ReferenceDataSource<std::u16string>;
extern template class RTT::OutputPort<std::u16string>;
extern template class RTT::InputPort<std::u16string>;
extern template class RTT::Property<std::u16string>;
extern template class RTT::Attribute<std::u16string>;
extern template class RTT::Constant<std::u16string>;

#endif  // OROCOS__RTT_ROS2_PRIMITIVES_TYPEKIT__TYPES_HPP_
