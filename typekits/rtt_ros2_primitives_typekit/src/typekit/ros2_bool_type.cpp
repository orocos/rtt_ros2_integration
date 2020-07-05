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

#include "rtt/types/PrimitiveSequenceTypeInfo.hpp"
#include "rtt/types/CArrayTypeInfo.hpp"

#include "rtt_ros2_primitives_typekit/Types.hpp"

// bool
template class RTT::internal::DataSource<bool>;
template class RTT::internal::AssignableDataSource<bool>;
template class RTT::internal::AssignCommand<bool>;
template class RTT::internal::ValueDataSource<bool>;
template class RTT::internal::ConstantDataSource<bool>;
template class RTT::internal::ReferenceDataSource<bool>;
template class RTT::OutputPort<bool>;
template class RTT::InputPort<bool>;
template class RTT::Property<bool>;
template class RTT::Attribute<bool>;
template class RTT::Constant<bool>;

template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<bool>>;
template class RTT::types::CArrayTypeInfo<RTT::types::carray<bool>>;
