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

// float64
template class RTT::internal::DataSource<double>;
template class RTT::internal::AssignableDataSource<double>;
template class RTT::internal::AssignCommand<double>;
template class RTT::internal::ValueDataSource<double>;
template class RTT::internal::ConstantDataSource<double>;
template class RTT::internal::ReferenceDataSource<double>;
template class RTT::OutputPort<double>;
template class RTT::InputPort<double>;
template class RTT::Property<double>;
template class RTT::Attribute<double>;
template class RTT::Constant<double>;

template class RTT::types::StdTypeInfo<double>;
template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<double>>;
template class RTT::types::CArrayTypeInfo<RTT::types::carray<double>>;
