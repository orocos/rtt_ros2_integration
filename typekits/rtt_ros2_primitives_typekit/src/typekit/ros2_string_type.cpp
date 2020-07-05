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

#include "rtt/types/PrimitiveSequenceTypeInfo.hpp"
#include "rtt/types/CArrayTypeInfo.hpp"

#include "rtt_ros2_primitives_typekit/Types.hpp"

// string
template class RTT::internal::DataSource<std::string>;
template class RTT::internal::AssignableDataSource<std::string>;
template class RTT::internal::AssignCommand<std::string>;
template class RTT::internal::ValueDataSource<std::string>;
template class RTT::internal::ConstantDataSource<std::string>;
template class RTT::internal::ReferenceDataSource<std::string>;
template class RTT::OutputPort<std::string>;
template class RTT::InputPort<std::string>;
template class RTT::Property<std::string>;
template class RTT::Attribute<std::string>;
template class RTT::Constant<std::string>;

template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<std::string>>;
template class RTT::types::CArrayTypeInfo<RTT::types::carray<std::string>>;
