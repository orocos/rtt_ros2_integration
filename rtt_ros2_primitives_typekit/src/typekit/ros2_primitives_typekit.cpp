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

#include <functional>
#include <string>
#include <vector>

#include "rtt/typekit/BoolTypeInfo.hpp"
#include "rtt/typekit/StdStringTypeInfo.hpp"
#include "rtt/typekit/StdTypeInfo.hpp"
#include "rtt/types/CArrayTypeInfo.hpp"
#include "rtt/types/PrimitiveSequenceTypeInfo.hpp"
#include "rtt/types/TemplateConstructor.hpp"
#include "rtt/types/TypekitPlugin.hpp"

#include "rtt_ros2_primitives_typekit/Types.hpp"

// extern template declarations for TypeInfo generators
extern template class RTT::types::StdTypeInfo<float>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<float>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<float>>;

extern template class RTT::types::StdTypeInfo<double>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<double>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<double>>;

extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<bool>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<bool>>;

extern template class RTT::types::StdTypeInfo<int8_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<int8_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<int8_t>>;

extern template class RTT::types::StdTypeInfo<uint8_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<uint8_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<uint8_t>>;

extern template class RTT::types::StdTypeInfo<int16_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<int16_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<int16_t>>;

extern template class RTT::types::StdTypeInfo<uint16_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<uint16_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<uint16_t>>;

extern template class RTT::types::StdTypeInfo<int32_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<int32_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<int32_t>>;

extern template class RTT::types::StdTypeInfo<uint32_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<uint32_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<uint32_t>>;

extern template class RTT::types::StdTypeInfo<int64_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<int64_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<int64_t>>;

extern template class RTT::types::StdTypeInfo<uint64_t>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<uint64_t>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<uint64_t>>;

extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<std::string>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<std::string>>;

extern template class RTT::types::StdBasicStringTypeInfo<std::u16string::value_type>;
extern template class RTT::types::PrimitiveSequenceTypeInfo<std::vector<std::u16string>>;
extern template class RTT::types::CArrayTypeInfo<RTT::types::carray<std::u16string>>;

namespace rtt_ros2_idl
{

namespace
{

template<class T, class R>
R convert(T t) {return static_cast<R>(t);}

template<typename CharT>
struct string_ctor
  : public std::unary_function<int, std::basic_string<CharT>>
{
  typedef std::basic_string<CharT>(Signature)(int);
  std::basic_string<CharT> operator()(int size) const
  {
    return std::basic_string<CharT>(static_cast<std::size_t>(size), {});
  }
};

}  // namespace

class RosPrimitivesTypekitPlugin : public RTT::types::TypekitPlugin
{
public:
  bool loadTypes() override
  {
    auto ti = RTT::types::TypeInfoRepository::Instance();

    ti->addType(new RTT::types::StdTypeInfo<float>("float32"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<float>>("float32[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<float>>("float32[c]"));
    ti->addType(new RTT::types::StdTypeInfo<double>("float64"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<double>>("float64[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<double>>("float64[c]"));
    // ti->addType(new RTT::types::BoolTypeInfo());
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<bool>>("bool[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<bool>>("bool[c]"));
    ti->addType(new RTT::types::StdTypeInfo<int8_t>("int8"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<int8_t>>("int8[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<int8_t>>("int8[c]"));
    ti->addType(new RTT::types::StdTypeInfo<uint8_t>("uint8"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<uint8_t>>("uint8[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<uint8_t>>("uint8[c]"));
    ti->addType(new RTT::types::StdTypeInfo<int16_t>("int16"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<int16_t>>("int16[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<int16_t>>("int16[c]"));
    ti->addType(new RTT::types::StdTypeInfo<uint16_t>("uint16"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<uint16_t>>("uint16[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<uint16_t>>("uint16[c]"));
    ti->addType(new RTT::types::StdTypeInfo<int32_t>("int32"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<int32_t>>("int32[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<int32_t>>("int32[c]"));
    ti->addType(new RTT::types::StdTypeInfo<uint32_t>("uint32"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<uint32_t>>("uint32[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<uint32_t>>("uint32[c]"));
    ti->addType(new RTT::types::StdTypeInfo<int64_t>("int64"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<int64_t>>("int64[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<int64_t>>("int64[c]"));
    ti->addType(new RTT::types::StdTypeInfo<uint64_t>("uint64"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<uint64_t>>("uint64[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<uint64_t>>("uint64[c]"));
    // ti->addType(new RTT::types::StdStringTypeInfo());
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<std::string>>("string[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<std::string>>("string[c]"));
    ti->addType(new RTT::types::StdBasicStringTypeInfo<std::u16string::value_type>("wstring"));
    ti->addType(new RTT::types::SequenceTypeInfo<std::vector<std::u16string>>("wstring[]"));
    ti->addType(new RTT::types::CArrayTypeInfo<RTT::types::carray<std::u16string>>("wstring[c]"));

    return true;
  }

  bool loadOperators() override {return true;}
  bool loadConstructors() override
  {
    auto ti = RTT::types::TypeInfoRepository::Instance();
    using RTT::types::newConstructor;

    // x to float
    ti->type("float")->addConstructor(newConstructor(&convert<double, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<int8_t, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<int16_t, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<int32_t, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<int64_t, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<uint8_t, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<uint16_t, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<uint32_t, float>, false));
    ti->type("float")->addConstructor(newConstructor(&convert<uint64_t, float>, false));

    // x to double
    ti->type("double")->addConstructor(newConstructor(&convert<float, double>, true));
    ti->type("double")->addConstructor(newConstructor(&convert<int8_t, double>, false));
    ti->type("double")->addConstructor(newConstructor(&convert<int16_t, double>, false));
    ti->type("double")->addConstructor(newConstructor(&convert<int32_t, double>, false));
    ti->type("double")->addConstructor(newConstructor(&convert<int64_t, double>, false));
    ti->type("double")->addConstructor(newConstructor(&convert<uint8_t, double>, false));
    ti->type("double")->addConstructor(newConstructor(&convert<uint16_t, double>, false));
    ti->type("double")->addConstructor(newConstructor(&convert<uint32_t, double>, false));
    ti->type("double")->addConstructor(newConstructor(&convert<uint64_t, double>, false));

    // x to int8_t
    ti->type("int8")->addConstructor(newConstructor(&convert<int16_t, int8_t>, false));
    ti->type("int8")->addConstructor(newConstructor(&convert<int32_t, int8_t>, false));
    ti->type("int8")->addConstructor(newConstructor(&convert<int64_t, int8_t>, false));
    ti->type("int8")->addConstructor(newConstructor(&convert<uint8_t, int8_t>, false));
    ti->type("int8")->addConstructor(newConstructor(&convert<uint16_t, int8_t>, false));
    ti->type("int8")->addConstructor(newConstructor(&convert<uint32_t, int8_t>, false));
    ti->type("int8")->addConstructor(newConstructor(&convert<uint64_t, int8_t>, false));

    // x to uint8_t
    ti->type("uint8")->addConstructor(newConstructor(&convert<int8_t, uint8_t>, false));
    ti->type("uint8")->addConstructor(newConstructor(&convert<int16_t, uint8_t>, false));
    ti->type("uint8")->addConstructor(newConstructor(&convert<int32_t, uint8_t>, false));
    ti->type("uint8")->addConstructor(newConstructor(&convert<int64_t, uint8_t>, false));
    ti->type("uint8")->addConstructor(newConstructor(&convert<uint16_t, uint8_t>, false));
    ti->type("uint8")->addConstructor(newConstructor(&convert<uint32_t, uint8_t>, false));
    ti->type("uint8")->addConstructor(newConstructor(&convert<uint64_t, uint8_t>, false));

    // x to int16_t
    ti->type("int16")->addConstructor(newConstructor(&convert<uint8_t, int16_t>, true));
    ti->type("int16")->addConstructor(newConstructor(&convert<uint16_t, int16_t>, false));
    ti->type("int16")->addConstructor(newConstructor(&convert<uint32_t, int16_t>, false));
    ti->type("int16")->addConstructor(newConstructor(&convert<uint64_t, int16_t>, false));
    ti->type("int16")->addConstructor(newConstructor(&convert<int8_t, int16_t>, true));
    ti->type("int16")->addConstructor(newConstructor(&convert<int32_t, int16_t>, false));
    ti->type("int16")->addConstructor(newConstructor(&convert<int64_t, int16_t>, false));

    // x to uint16_t
    ti->type("uint16")->addConstructor(newConstructor(&convert<int8_t, uint16_t>, false));
    ti->type("uint16")->addConstructor(newConstructor(&convert<int16_t, uint16_t>, false));
    ti->type("uint16")->addConstructor(newConstructor(&convert<int32_t, uint16_t>, false));
    ti->type("uint16")->addConstructor(newConstructor(&convert<int64_t, uint16_t>, false));
    ti->type("uint16")->addConstructor(newConstructor(&convert<uint8_t, uint16_t>, true));
    ti->type("uint16")->addConstructor(newConstructor(&convert<uint32_t, uint16_t>, false));
    ti->type("uint16")->addConstructor(newConstructor(&convert<uint64_t, uint16_t>, false));

    // x to int32_t
    ti->type("int32")->addConstructor(newConstructor(&convert<uint8_t, int32_t>, true));
    ti->type("int32")->addConstructor(newConstructor(&convert<uint16_t, int32_t>, true));
    ti->type("int32")->addConstructor(newConstructor(&convert<uint32_t, int32_t>, false));
    ti->type("int32")->addConstructor(newConstructor(&convert<uint64_t, int32_t>, false));
    ti->type("int32")->addConstructor(newConstructor(&convert<int8_t, int32_t>, true));
    ti->type("int32")->addConstructor(newConstructor(&convert<int16_t, int32_t>, true));
    ti->type("int32")->addConstructor(newConstructor(&convert<int64_t, int32_t>, false));

    // x to uint32_t
    ti->type("uint32")->addConstructor(newConstructor(&convert<int8_t, uint32_t>, false));
    ti->type("uint32")->addConstructor(newConstructor(&convert<int16_t, uint32_t>, false));
    ti->type("uint32")->addConstructor(newConstructor(&convert<int32_t, uint32_t>, false));
    ti->type("uint32")->addConstructor(newConstructor(&convert<int64_t, uint32_t>, false));
    ti->type("uint32")->addConstructor(newConstructor(&convert<uint8_t, uint32_t>, true));
    ti->type("uint32")->addConstructor(newConstructor(&convert<uint16_t, uint32_t>, true));
    ti->type("uint32")->addConstructor(newConstructor(&convert<uint64_t, uint32_t>, false));

    // x to int64_t
    ti->type("int64")->addConstructor(newConstructor(&convert<uint8_t, int64_t>, true));
    ti->type("int64")->addConstructor(newConstructor(&convert<uint16_t, int64_t>, true));
    ti->type("int64")->addConstructor(newConstructor(&convert<uint32_t, int64_t>, true));
    ti->type("int64")->addConstructor(newConstructor(&convert<uint64_t, int64_t>, false));
    ti->type("int64")->addConstructor(newConstructor(&convert<int8_t, int64_t>, true));
    ti->type("int64")->addConstructor(newConstructor(&convert<int16_t, int64_t>, true));
    ti->type("int64")->addConstructor(newConstructor(&convert<int32_t, int64_t>, true));

    // x to uint64_t
    ti->type("uint64")->addConstructor(newConstructor(&convert<int8_t, uint64_t>, false));
    ti->type("uint64")->addConstructor(newConstructor(&convert<int16_t, uint64_t>, false));
    ti->type("uint64")->addConstructor(newConstructor(&convert<int32_t, uint64_t>, false));
    ti->type("uint64")->addConstructor(newConstructor(&convert<int64_t, uint64_t>, false));
    ti->type("uint64")->addConstructor(newConstructor(&convert<uint8_t, uint64_t>, true));
    ti->type("uint64")->addConstructor(newConstructor(&convert<uint16_t, uint64_t>, true));
    ti->type("uint64")->addConstructor(newConstructor(&convert<uint32_t, uint64_t>, true));

    ti->type("string")->addConstructor(newConstructor(string_ctor<std::string::value_type>()));
    ti->type("wstring")->addConstructor(newConstructor(string_ctor<std::u16string::value_type>()));

    return true;
  }

  bool loadGlobals() {return true;}

  std::string getName() override
  {
    static const std::string name = "ros2-primitives";
    return name;
  }
};

}  // namespace rtt_ros2_idl

ORO_TYPEKIT_PLUGIN(rtt_ros2_idl::RosPrimitivesTypekitPlugin)
