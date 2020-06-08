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

#include <vector>

#include "rtt/typekit/StdTypeInfo.hpp"
#include "rtt/types/PrimitiveSequenceTypeInfo.hpp"
#include "rtt/types/CArrayTypeInfo.hpp"

#include "rtt_ros2_primitives_typekit/Types.hpp"

// int16
template class RTT::internal::DataSource<int16_t>;
template class RTT::internal::AssignableDataSource<int16_t>;
template class RTT::internal::AssignCommand<int16_t>;
template class RTT::internal::ValueDataSource<int16_t>;
template class RTT::internal::ConstantDataSource<int16_t>;
template class RTT::internal::ReferenceDataSource<int16_t>;
template class RTT::OutputPort<int16_t>;
template class RTT::InputPort<int16_t>;
template class RTT::Property<int16_t>;
template class RTT::Attribute<int16_t>;
template class RTT::Constant<int16_t>;

template class RTT::types::StdTypeInfo<int16_t>;
template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<int16_t>>;
template class RTT::types::CArrayTypeInfo<RTT::types::carray<int16_t>>;
