#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "idltest_msgs/msg/bool.hpp"
#include "idltest_msgs/msg/bool_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/bool_fixed_array.hpp"
#include "idltest_msgs/msg/bool_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/byte.hpp"
#include "idltest_msgs/msg/byte_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/byte_fixed_array.hpp"
#include "idltest_msgs/msg/byte_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/char.hpp"
#include "idltest_msgs/msg/char_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/char_fixed_array.hpp"
#include "idltest_msgs/msg/char_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/float32.hpp"
#include "idltest_msgs/msg/float32_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/float32_fixed_array.hpp"
#include "idltest_msgs/msg/float32_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/float64.hpp"
#include "idltest_msgs/msg/float64_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/float64_fixed_array.hpp"
#include "idltest_msgs/msg/float64_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/int16.hpp"
#include "idltest_msgs/msg/int16_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/int16_fixed_array.hpp"
#include "idltest_msgs/msg/int16_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/int32.hpp"
#include "idltest_msgs/msg/int32_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/int32_fixed_array.hpp"
#include "idltest_msgs/msg/int32_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/int64.hpp"
#include "idltest_msgs/msg/int64_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/int64_fixed_array.hpp"
#include "idltest_msgs/msg/int64_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/int8.hpp"
#include "idltest_msgs/msg/int8_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/int8_fixed_array.hpp"
#include "idltest_msgs/msg/int8_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/string.hpp"
#include "idltest_msgs/msg/string_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/string_fixed_array.hpp"
#include "idltest_msgs/msg/string_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int16.hpp"
#include "idltest_msgs/msg/u_int16_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int16_fixed_array.hpp"
#include "idltest_msgs/msg/u_int16_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int32.hpp"
#include "idltest_msgs/msg/u_int32_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int32_fixed_array.hpp"
#include "idltest_msgs/msg/u_int32_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int64.hpp"
#include "idltest_msgs/msg/u_int64_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int64_fixed_array.hpp"
#include "idltest_msgs/msg/u_int64_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int8.hpp"
#include "idltest_msgs/msg/u_int8_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/u_int8_fixed_array.hpp"
#include "idltest_msgs/msg/u_int8_unbounded_dyn_array.hpp"
#include "idltest_msgs/msg/w_string.hpp"
#include "idltest_msgs/msg/w_string_bounded_dyn_array.hpp"
#include "idltest_msgs/msg/w_string_fixed_array.hpp"
#include "idltest_msgs/msg/w_string_unbounded_dyn_array.hpp"

using std::placeholders::_1;


class MultiPubSub : public rclcpp::Node
{

public:
  MultiPubSub() : Node("multi_pubsub")
  {
    sub_bool_ = this->create_subscription<idltest_msgs::msg::Bool>(
      "idltest_Bool_in", 10, std::bind(&MultiPubSub::bool_callback, this, _1));
    pub_bool_ = this->create_publisher<idltest_msgs::msg::Bool>(
      "idltest_Bool_out", 10);

    sub_bool_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::BoolBoundedDynArray>(
      "idltest_BoolBoundedDynArray_in", 10, std::bind(&MultiPubSub::bool_bounded_dyn_array_callback, this, _1));
    pub_bool_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::BoolBoundedDynArray>(
      "idltest_BoolBoundedDynArray_out", 10);

    sub_bool_fixed_array_ = this->create_subscription<idltest_msgs::msg::BoolFixedArray>(
      "idltest_BoolFixedArray_in", 10, std::bind(&MultiPubSub::bool_fixed_array_callback, this, _1));
    pub_bool_fixed_array_ = this->create_publisher<idltest_msgs::msg::BoolFixedArray>(
      "idltest_BoolFixedArray_out", 10);

    sub_bool_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::BoolUnboundedDynArray>(
      "idltest_BoolUnboundedDynArray_in", 10, std::bind(&MultiPubSub::bool_unbounded_dyn_array_callback, this, _1));
    pub_bool_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::BoolUnboundedDynArray>(
      "idltest_BoolUnboundedDynArray_out", 10);

    sub_byte_ = this->create_subscription<idltest_msgs::msg::Byte>(
      "idltest_Byte_in", 10, std::bind(&MultiPubSub::byte_callback, this, _1));
    pub_byte_ = this->create_publisher<idltest_msgs::msg::Byte>(
      "idltest_Byte_out", 10);

    sub_byte_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::ByteBoundedDynArray>(
      "idltest_ByteBoundedDynArray_in", 10, std::bind(&MultiPubSub::byte_bounded_dyn_array_callback, this, _1));
    pub_byte_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::ByteBoundedDynArray>(
      "idltest_ByteBoundedDynArray_out", 10);

    sub_byte_fixed_array_ = this->create_subscription<idltest_msgs::msg::ByteFixedArray>(
      "idltest_ByteFixedArray_in", 10, std::bind(&MultiPubSub::byte_fixed_array_callback, this, _1));
    pub_byte_fixed_array_ = this->create_publisher<idltest_msgs::msg::ByteFixedArray>(
      "idltest_ByteFixedArray_out", 10);

    sub_byte_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::ByteUnboundedDynArray>(
      "idltest_ByteUnboundedDynArray_in", 10, std::bind(&MultiPubSub::byte_unbounded_dyn_array_callback, this, _1));
    pub_byte_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::ByteUnboundedDynArray>(
      "idltest_ByteUnboundedDynArray_out", 10);

    sub_char_ = this->create_subscription<idltest_msgs::msg::Char>(
      "idltest_Char_in", 10, std::bind(&MultiPubSub::char_callback, this, _1));
    pub_char_ = this->create_publisher<idltest_msgs::msg::Char>(
      "idltest_Char_out", 10);

    sub_char_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::CharBoundedDynArray>(
      "idltest_CharBoundedDynArray_in", 10, std::bind(&MultiPubSub::char_bounded_dyn_array_callback, this, _1));
    pub_char_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::CharBoundedDynArray>(
      "idltest_CharBoundedDynArray_out", 10);

    sub_char_fixed_array_ = this->create_subscription<idltest_msgs::msg::CharFixedArray>(
      "idltest_CharFixedArray_in", 10, std::bind(&MultiPubSub::char_fixed_array_callback, this, _1));
    pub_char_fixed_array_ = this->create_publisher<idltest_msgs::msg::CharFixedArray>(
      "idltest_CharFixedArray_out", 10);

    sub_char_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::CharUnboundedDynArray>(
      "idltest_CharUnboundedDynArray_in", 10, std::bind(&MultiPubSub::char_unbounded_dyn_array_callback, this, _1));
    pub_char_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::CharUnboundedDynArray>(
      "idltest_CharUnboundedDynArray_out", 10);

    sub_float32_ = this->create_subscription<idltest_msgs::msg::Float32>(
      "idltest_Float32_in", 10, std::bind(&MultiPubSub::float32_callback, this, _1));
    pub_float32_ = this->create_publisher<idltest_msgs::msg::Float32>(
      "idltest_Float32_out", 10);

    sub_float32_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Float32BoundedDynArray>(
      "idltest_Float32BoundedDynArray_in", 10, std::bind(&MultiPubSub::float32_bounded_dyn_array_callback, this, _1));
    pub_float32_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Float32BoundedDynArray>(
      "idltest_Float32BoundedDynArray_out", 10);

    sub_float32_fixed_array_ = this->create_subscription<idltest_msgs::msg::Float32FixedArray>(
      "idltest_Float32FixedArray_in", 10, std::bind(&MultiPubSub::float32_fixed_array_callback, this, _1));
    pub_float32_fixed_array_ = this->create_publisher<idltest_msgs::msg::Float32FixedArray>(
      "idltest_Float32FixedArray_out", 10);

    sub_float32_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Float32UnboundedDynArray>(
      "idltest_Float32UnboundedDynArray_in", 10, std::bind(&MultiPubSub::float32_unbounded_dyn_array_callback, this, _1));
    pub_float32_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Float32UnboundedDynArray>(
      "idltest_Float32UnboundedDynArray_out", 10);

    sub_float64_ = this->create_subscription<idltest_msgs::msg::Float64>(
      "idltest_Float64_in", 10, std::bind(&MultiPubSub::float64_callback, this, _1));
    pub_float64_ = this->create_publisher<idltest_msgs::msg::Float64>(
      "idltest_Float64_out", 10);

    sub_float64_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Float64BoundedDynArray>(
      "idltest_Float64BoundedDynArray_in", 10, std::bind(&MultiPubSub::float64_bounded_dyn_array_callback, this, _1));
    pub_float64_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Float64BoundedDynArray>(
      "idltest_Float64BoundedDynArray_out", 10);

    sub_float64_fixed_array_ = this->create_subscription<idltest_msgs::msg::Float64FixedArray>(
      "idltest_Float64FixedArray_in", 10, std::bind(&MultiPubSub::float64_fixed_array_callback, this, _1));
    pub_float64_fixed_array_ = this->create_publisher<idltest_msgs::msg::Float64FixedArray>(
      "idltest_Float64FixedArray_out", 10);

    sub_float64_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Float64UnboundedDynArray>(
      "idltest_Float64UnboundedDynArray_in", 10, std::bind(&MultiPubSub::float64_unbounded_dyn_array_callback, this, _1));
    pub_float64_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Float64UnboundedDynArray>(
      "idltest_Float64UnboundedDynArray_out", 10);

    sub_int16_ = this->create_subscription<idltest_msgs::msg::Int16>(
      "idltest_Int16_in", 10, std::bind(&MultiPubSub::int16_callback, this, _1));
    pub_int16_ = this->create_publisher<idltest_msgs::msg::Int16>(
      "idltest_Int16_out", 10);

    sub_int16_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int16BoundedDynArray>(
      "idltest_Int16BoundedDynArray_in", 10, std::bind(&MultiPubSub::int16_bounded_dyn_array_callback, this, _1));
    pub_int16_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int16BoundedDynArray>(
      "idltest_Int16BoundedDynArray_out", 10);

    sub_int16_fixed_array_ = this->create_subscription<idltest_msgs::msg::Int16FixedArray>(
      "idltest_Int16FixedArray_in", 10, std::bind(&MultiPubSub::int16_fixed_array_callback, this, _1));
    pub_int16_fixed_array_ = this->create_publisher<idltest_msgs::msg::Int16FixedArray>(
      "idltest_Int16FixedArray_out", 10);

    sub_int16_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int16UnboundedDynArray>(
      "idltest_Int16UnboundedDynArray_in", 10, std::bind(&MultiPubSub::int16_unbounded_dyn_array_callback, this, _1));
    pub_int16_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int16UnboundedDynArray>(
      "idltest_Int16UnboundedDynArray_out", 10);

    sub_int32_ = this->create_subscription<idltest_msgs::msg::Int32>(
      "idltest_Int32_in", 10, std::bind(&MultiPubSub::int32_callback, this, _1));
    pub_int32_ = this->create_publisher<idltest_msgs::msg::Int32>(
      "idltest_Int32_out", 10);

    sub_int32_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int32BoundedDynArray>(
      "idltest_Int32BoundedDynArray_in", 10, std::bind(&MultiPubSub::int32_bounded_dyn_array_callback, this, _1));
    pub_int32_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int32BoundedDynArray>(
      "idltest_Int32BoundedDynArray_out", 10);

    sub_int32_fixed_array_ = this->create_subscription<idltest_msgs::msg::Int32FixedArray>(
      "idltest_Int32FixedArray_in", 10, std::bind(&MultiPubSub::int32_fixed_array_callback, this, _1));
    pub_int32_fixed_array_ = this->create_publisher<idltest_msgs::msg::Int32FixedArray>(
      "idltest_Int32FixedArray_out", 10);

    sub_int32_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int32UnboundedDynArray>(
      "idltest_Int32UnboundedDynArray_in", 10, std::bind(&MultiPubSub::int32_unbounded_dyn_array_callback, this, _1));
    pub_int32_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int32UnboundedDynArray>(
      "idltest_Int32UnboundedDynArray_out", 10);

    sub_int64_ = this->create_subscription<idltest_msgs::msg::Int64>(
      "idltest_Int64_in", 10, std::bind(&MultiPubSub::int64_callback, this, _1));
    pub_int64_ = this->create_publisher<idltest_msgs::msg::Int64>(
      "idltest_Int64_out", 10);

    sub_int64_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int64BoundedDynArray>(
      "idltest_Int64BoundedDynArray_in", 10, std::bind(&MultiPubSub::int64_bounded_dyn_array_callback, this, _1));
    pub_int64_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int64BoundedDynArray>(
      "idltest_Int64BoundedDynArray_out", 10);

    sub_int64_fixed_array_ = this->create_subscription<idltest_msgs::msg::Int64FixedArray>(
      "idltest_Int64FixedArray_in", 10, std::bind(&MultiPubSub::int64_fixed_array_callback, this, _1));
    pub_int64_fixed_array_ = this->create_publisher<idltest_msgs::msg::Int64FixedArray>(
      "idltest_Int64FixedArray_out", 10);

    sub_int64_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int64UnboundedDynArray>(
      "idltest_Int64UnboundedDynArray_in", 10, std::bind(&MultiPubSub::int64_unbounded_dyn_array_callback, this, _1));
    pub_int64_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int64UnboundedDynArray>(
      "idltest_Int64UnboundedDynArray_out", 10);

    sub_int8_ = this->create_subscription<idltest_msgs::msg::Int8>(
      "idltest_Int8_in", 10, std::bind(&MultiPubSub::int8_callback, this, _1));
    pub_int8_ = this->create_publisher<idltest_msgs::msg::Int8>(
      "idltest_Int8_out", 10);

    sub_int8_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int8BoundedDynArray>(
      "idltest_Int8BoundedDynArray_in", 10, std::bind(&MultiPubSub::int8_bounded_dyn_array_callback, this, _1));
    pub_int8_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int8BoundedDynArray>(
      "idltest_Int8BoundedDynArray_out", 10);

    sub_int8_fixed_array_ = this->create_subscription<idltest_msgs::msg::Int8FixedArray>(
      "idltest_Int8FixedArray_in", 10, std::bind(&MultiPubSub::int8_fixed_array_callback, this, _1));
    pub_int8_fixed_array_ = this->create_publisher<idltest_msgs::msg::Int8FixedArray>(
      "idltest_Int8FixedArray_out", 10);

    sub_int8_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::Int8UnboundedDynArray>(
      "idltest_Int8UnboundedDynArray_in", 10, std::bind(&MultiPubSub::int8_unbounded_dyn_array_callback, this, _1));
    pub_int8_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::Int8UnboundedDynArray>(
      "idltest_Int8UnboundedDynArray_out", 10);

    sub_string_ = this->create_subscription<idltest_msgs::msg::String>(
      "idltest_String_in", 10, std::bind(&MultiPubSub::string_callback, this, _1));
    pub_string_ = this->create_publisher<idltest_msgs::msg::String>(
      "idltest_String_out", 10);

    sub_string_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::StringBoundedDynArray>(
      "idltest_StringBoundedDynArray_in", 10, std::bind(&MultiPubSub::string_bounded_dyn_array_callback, this, _1));
    pub_string_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::StringBoundedDynArray>(
      "idltest_StringBoundedDynArray_out", 10);

    sub_string_fixed_array_ = this->create_subscription<idltest_msgs::msg::StringFixedArray>(
      "idltest_StringFixedArray_in", 10, std::bind(&MultiPubSub::string_fixed_array_callback, this, _1));
    pub_string_fixed_array_ = this->create_publisher<idltest_msgs::msg::StringFixedArray>(
      "idltest_StringFixedArray_out", 10);

    sub_string_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::StringUnboundedDynArray>(
      "idltest_StringUnboundedDynArray_in", 10, std::bind(&MultiPubSub::string_unbounded_dyn_array_callback, this, _1));
    pub_string_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::StringUnboundedDynArray>(
      "idltest_StringUnboundedDynArray_out", 10);

    sub_u_int16_ = this->create_subscription<idltest_msgs::msg::UInt16>(
      "idltest_UInt16_in", 10, std::bind(&MultiPubSub::u_int16_callback, this, _1));
    pub_u_int16_ = this->create_publisher<idltest_msgs::msg::UInt16>(
      "idltest_UInt16_out", 10);

    sub_u_int16_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt16BoundedDynArray>(
      "idltest_UInt16BoundedDynArray_in", 10, std::bind(&MultiPubSub::u_int16_bounded_dyn_array_callback, this, _1));
    pub_u_int16_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt16BoundedDynArray>(
      "idltest_UInt16BoundedDynArray_out", 10);

    sub_u_int16_fixed_array_ = this->create_subscription<idltest_msgs::msg::UInt16FixedArray>(
      "idltest_UInt16FixedArray_in", 10, std::bind(&MultiPubSub::u_int16_fixed_array_callback, this, _1));
    pub_u_int16_fixed_array_ = this->create_publisher<idltest_msgs::msg::UInt16FixedArray>(
      "idltest_UInt16FixedArray_out", 10);

    sub_u_int16_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt16UnboundedDynArray>(
      "idltest_UInt16UnboundedDynArray_in", 10, std::bind(&MultiPubSub::u_int16_unbounded_dyn_array_callback, this, _1));
    pub_u_int16_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt16UnboundedDynArray>(
      "idltest_UInt16UnboundedDynArray_out", 10);

    sub_u_int32_ = this->create_subscription<idltest_msgs::msg::UInt32>(
      "idltest_UInt32_in", 10, std::bind(&MultiPubSub::u_int32_callback, this, _1));
    pub_u_int32_ = this->create_publisher<idltest_msgs::msg::UInt32>(
      "idltest_UInt32_out", 10);

    sub_u_int32_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt32BoundedDynArray>(
      "idltest_UInt32BoundedDynArray_in", 10, std::bind(&MultiPubSub::u_int32_bounded_dyn_array_callback, this, _1));
    pub_u_int32_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt32BoundedDynArray>(
      "idltest_UInt32BoundedDynArray_out", 10);

    sub_u_int32_fixed_array_ = this->create_subscription<idltest_msgs::msg::UInt32FixedArray>(
      "idltest_UInt32FixedArray_in", 10, std::bind(&MultiPubSub::u_int32_fixed_array_callback, this, _1));
    pub_u_int32_fixed_array_ = this->create_publisher<idltest_msgs::msg::UInt32FixedArray>(
      "idltest_UInt32FixedArray_out", 10);

    sub_u_int32_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt32UnboundedDynArray>(
      "idltest_UInt32UnboundedDynArray_in", 10, std::bind(&MultiPubSub::u_int32_unbounded_dyn_array_callback, this, _1));
    pub_u_int32_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt32UnboundedDynArray>(
      "idltest_UInt32UnboundedDynArray_out", 10);

    sub_u_int64_ = this->create_subscription<idltest_msgs::msg::UInt64>(
      "idltest_UInt64_in", 10, std::bind(&MultiPubSub::u_int64_callback, this, _1));
    pub_u_int64_ = this->create_publisher<idltest_msgs::msg::UInt64>(
      "idltest_UInt64_out", 10);

    sub_u_int64_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt64BoundedDynArray>(
      "idltest_UInt64BoundedDynArray_in", 10, std::bind(&MultiPubSub::u_int64_bounded_dyn_array_callback, this, _1));
    pub_u_int64_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt64BoundedDynArray>(
      "idltest_UInt64BoundedDynArray_out", 10);

    sub_u_int64_fixed_array_ = this->create_subscription<idltest_msgs::msg::UInt64FixedArray>(
      "idltest_UInt64FixedArray_in", 10, std::bind(&MultiPubSub::u_int64_fixed_array_callback, this, _1));
    pub_u_int64_fixed_array_ = this->create_publisher<idltest_msgs::msg::UInt64FixedArray>(
      "idltest_UInt64FixedArray_out", 10);

    sub_u_int64_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt64UnboundedDynArray>(
      "idltest_UInt64UnboundedDynArray_in", 10, std::bind(&MultiPubSub::u_int64_unbounded_dyn_array_callback, this, _1));
    pub_u_int64_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt64UnboundedDynArray>(
      "idltest_UInt64UnboundedDynArray_out", 10);

    sub_u_int8_ = this->create_subscription<idltest_msgs::msg::UInt8>(
      "idltest_UInt8_in", 10, std::bind(&MultiPubSub::u_int8_callback, this, _1));
    pub_u_int8_ = this->create_publisher<idltest_msgs::msg::UInt8>(
      "idltest_UInt8_out", 10);

    sub_u_int8_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt8BoundedDynArray>(
      "idltest_UInt8BoundedDynArray_in", 10, std::bind(&MultiPubSub::u_int8_bounded_dyn_array_callback, this, _1));
    pub_u_int8_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt8BoundedDynArray>(
      "idltest_UInt8BoundedDynArray_out", 10);

    sub_u_int8_fixed_array_ = this->create_subscription<idltest_msgs::msg::UInt8FixedArray>(
      "idltest_UInt8FixedArray_in", 10, std::bind(&MultiPubSub::u_int8_fixed_array_callback, this, _1));
    pub_u_int8_fixed_array_ = this->create_publisher<idltest_msgs::msg::UInt8FixedArray>(
      "idltest_UInt8FixedArray_out", 10);

    sub_u_int8_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::UInt8UnboundedDynArray>(
      "idltest_UInt8UnboundedDynArray_in", 10, std::bind(&MultiPubSub::u_int8_unbounded_dyn_array_callback, this, _1));
    pub_u_int8_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::UInt8UnboundedDynArray>(
      "idltest_UInt8UnboundedDynArray_out", 10);

    sub_w_string_ = this->create_subscription<idltest_msgs::msg::WString>(
      "idltest_WString_in", 10, std::bind(&MultiPubSub::w_string_callback, this, _1));
    pub_w_string_ = this->create_publisher<idltest_msgs::msg::WString>(
      "idltest_WString_out", 10);

    sub_w_string_bounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::WStringBoundedDynArray>(
      "idltest_WStringBoundedDynArray_in", 10, std::bind(&MultiPubSub::w_string_bounded_dyn_array_callback, this, _1));
    pub_w_string_bounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::WStringBoundedDynArray>(
      "idltest_WStringBoundedDynArray_out", 10);

    sub_w_string_fixed_array_ = this->create_subscription<idltest_msgs::msg::WStringFixedArray>(
      "idltest_WStringFixedArray_in", 10, std::bind(&MultiPubSub::w_string_fixed_array_callback, this, _1));
    pub_w_string_fixed_array_ = this->create_publisher<idltest_msgs::msg::WStringFixedArray>(
      "idltest_WStringFixedArray_out", 10);

    sub_w_string_unbounded_dyn_array_ = this->create_subscription<idltest_msgs::msg::WStringUnboundedDynArray>(
      "idltest_WStringUnboundedDynArray_in", 10, std::bind(&MultiPubSub::w_string_unbounded_dyn_array_callback, this, _1));
    pub_w_string_unbounded_dyn_array_ = this->create_publisher<idltest_msgs::msg::WStringUnboundedDynArray>(
      "idltest_WStringUnboundedDynArray_out", 10);

  }

private:
  void bool_callback(const idltest_msgs::msg::Bool::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Bool();
    msg2.data = msg->data;
    pub_bool_->publish(msg2);
    }

  void bool_bounded_dyn_array_callback(const idltest_msgs::msg::BoolBoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::BoolBoundedDynArray();
    msg2.data = msg->data;
    pub_bool_bounded_dyn_array_->publish(msg2);
    }

  void bool_fixed_array_callback(const idltest_msgs::msg::BoolFixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::BoolFixedArray();
    msg2.data = msg->data;
    pub_bool_fixed_array_->publish(msg2);
    }

  void bool_unbounded_dyn_array_callback(const idltest_msgs::msg::BoolUnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::BoolUnboundedDynArray();
    msg2.data = msg->data;
    pub_bool_unbounded_dyn_array_->publish(msg2);
    }

  void byte_callback(const idltest_msgs::msg::Byte::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Byte();
    msg2.data = msg->data;
    pub_byte_->publish(msg2);
    }

  void byte_bounded_dyn_array_callback(const idltest_msgs::msg::ByteBoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::ByteBoundedDynArray();
    msg2.data = msg->data;
    pub_byte_bounded_dyn_array_->publish(msg2);
    }

  void byte_fixed_array_callback(const idltest_msgs::msg::ByteFixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::ByteFixedArray();
    msg2.data = msg->data;
    pub_byte_fixed_array_->publish(msg2);
    }

  void byte_unbounded_dyn_array_callback(const idltest_msgs::msg::ByteUnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::ByteUnboundedDynArray();
    msg2.data = msg->data;
    pub_byte_unbounded_dyn_array_->publish(msg2);
    }

  void char_callback(const idltest_msgs::msg::Char::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Char();
    msg2.data = msg->data;
    pub_char_->publish(msg2);
    }

  void char_bounded_dyn_array_callback(const idltest_msgs::msg::CharBoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::CharBoundedDynArray();
    msg2.data = msg->data;
    pub_char_bounded_dyn_array_->publish(msg2);
    }

  void char_fixed_array_callback(const idltest_msgs::msg::CharFixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::CharFixedArray();
    msg2.data = msg->data;
    pub_char_fixed_array_->publish(msg2);
    }

  void char_unbounded_dyn_array_callback(const idltest_msgs::msg::CharUnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::CharUnboundedDynArray();
    msg2.data = msg->data;
    pub_char_unbounded_dyn_array_->publish(msg2);
    }

  void float32_callback(const idltest_msgs::msg::Float32::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float32();
    msg2.data = msg->data;
    pub_float32_->publish(msg2);
    }

  void float32_bounded_dyn_array_callback(const idltest_msgs::msg::Float32BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float32BoundedDynArray();
    msg2.data = msg->data;
    pub_float32_bounded_dyn_array_->publish(msg2);
    }

  void float32_fixed_array_callback(const idltest_msgs::msg::Float32FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float32FixedArray();
    msg2.data = msg->data;
    pub_float32_fixed_array_->publish(msg2);
    }

  void float32_unbounded_dyn_array_callback(const idltest_msgs::msg::Float32UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float32UnboundedDynArray();
    msg2.data = msg->data;
    pub_float32_unbounded_dyn_array_->publish(msg2);
    }

  void float64_callback(const idltest_msgs::msg::Float64::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float64();
    msg2.data = msg->data;
    pub_float64_->publish(msg2);
    }

  void float64_bounded_dyn_array_callback(const idltest_msgs::msg::Float64BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float64BoundedDynArray();
    msg2.data = msg->data;
    pub_float64_bounded_dyn_array_->publish(msg2);
    }

  void float64_fixed_array_callback(const idltest_msgs::msg::Float64FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float64FixedArray();
    msg2.data = msg->data;
    pub_float64_fixed_array_->publish(msg2);
    }

  void float64_unbounded_dyn_array_callback(const idltest_msgs::msg::Float64UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Float64UnboundedDynArray();
    msg2.data = msg->data;
    pub_float64_unbounded_dyn_array_->publish(msg2);
    }

  void int16_callback(const idltest_msgs::msg::Int16::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int16();
    msg2.data = msg->data;
    pub_int16_->publish(msg2);
    }

  void int16_bounded_dyn_array_callback(const idltest_msgs::msg::Int16BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int16BoundedDynArray();
    msg2.data = msg->data;
    pub_int16_bounded_dyn_array_->publish(msg2);
    }

  void int16_fixed_array_callback(const idltest_msgs::msg::Int16FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int16FixedArray();
    msg2.data = msg->data;
    pub_int16_fixed_array_->publish(msg2);
    }

  void int16_unbounded_dyn_array_callback(const idltest_msgs::msg::Int16UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int16UnboundedDynArray();
    msg2.data = msg->data;
    pub_int16_unbounded_dyn_array_->publish(msg2);
    }

  void int32_callback(const idltest_msgs::msg::Int32::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int32();
    msg2.data = msg->data;
    pub_int32_->publish(msg2);
    }

  void int32_bounded_dyn_array_callback(const idltest_msgs::msg::Int32BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int32BoundedDynArray();
    msg2.data = msg->data;
    pub_int32_bounded_dyn_array_->publish(msg2);
    }

  void int32_fixed_array_callback(const idltest_msgs::msg::Int32FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int32FixedArray();
    msg2.data = msg->data;
    pub_int32_fixed_array_->publish(msg2);
    }

  void int32_unbounded_dyn_array_callback(const idltest_msgs::msg::Int32UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int32UnboundedDynArray();
    msg2.data = msg->data;
    pub_int32_unbounded_dyn_array_->publish(msg2);
    }

  void int64_callback(const idltest_msgs::msg::Int64::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int64();
    msg2.data = msg->data;
    pub_int64_->publish(msg2);
    }

  void int64_bounded_dyn_array_callback(const idltest_msgs::msg::Int64BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int64BoundedDynArray();
    msg2.data = msg->data;
    pub_int64_bounded_dyn_array_->publish(msg2);
    }

  void int64_fixed_array_callback(const idltest_msgs::msg::Int64FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int64FixedArray();
    msg2.data = msg->data;
    pub_int64_fixed_array_->publish(msg2);
    }

  void int64_unbounded_dyn_array_callback(const idltest_msgs::msg::Int64UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int64UnboundedDynArray();
    msg2.data = msg->data;
    pub_int64_unbounded_dyn_array_->publish(msg2);
    }

  void int8_callback(const idltest_msgs::msg::Int8::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int8();
    msg2.data = msg->data;
    pub_int8_->publish(msg2);
    }

  void int8_bounded_dyn_array_callback(const idltest_msgs::msg::Int8BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int8BoundedDynArray();
    msg2.data = msg->data;
    pub_int8_bounded_dyn_array_->publish(msg2);
    }

  void int8_fixed_array_callback(const idltest_msgs::msg::Int8FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int8FixedArray();
    msg2.data = msg->data;
    pub_int8_fixed_array_->publish(msg2);
    }

  void int8_unbounded_dyn_array_callback(const idltest_msgs::msg::Int8UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::Int8UnboundedDynArray();
    msg2.data = msg->data;
    pub_int8_unbounded_dyn_array_->publish(msg2);
    }

  void string_callback(const idltest_msgs::msg::String::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::String();
    msg2.data = msg->data;
    pub_string_->publish(msg2);
    }

  void string_bounded_dyn_array_callback(const idltest_msgs::msg::StringBoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::StringBoundedDynArray();
    msg2.data = msg->data;
    pub_string_bounded_dyn_array_->publish(msg2);
    }

  void string_fixed_array_callback(const idltest_msgs::msg::StringFixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::StringFixedArray();
    msg2.data = msg->data;
    pub_string_fixed_array_->publish(msg2);
    }

  void string_unbounded_dyn_array_callback(const idltest_msgs::msg::StringUnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::StringUnboundedDynArray();
    msg2.data = msg->data;
    pub_string_unbounded_dyn_array_->publish(msg2);
    }

  void u_int16_callback(const idltest_msgs::msg::UInt16::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt16();
    msg2.data = msg->data;
    pub_u_int16_->publish(msg2);
    }

  void u_int16_bounded_dyn_array_callback(const idltest_msgs::msg::UInt16BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt16BoundedDynArray();
    msg2.data = msg->data;
    pub_u_int16_bounded_dyn_array_->publish(msg2);
    }

  void u_int16_fixed_array_callback(const idltest_msgs::msg::UInt16FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt16FixedArray();
    msg2.data = msg->data;
    pub_u_int16_fixed_array_->publish(msg2);
    }

  void u_int16_unbounded_dyn_array_callback(const idltest_msgs::msg::UInt16UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt16UnboundedDynArray();
    msg2.data = msg->data;
    pub_u_int16_unbounded_dyn_array_->publish(msg2);
    }

  void u_int32_callback(const idltest_msgs::msg::UInt32::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt32();
    msg2.data = msg->data;
    pub_u_int32_->publish(msg2);
    }

  void u_int32_bounded_dyn_array_callback(const idltest_msgs::msg::UInt32BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt32BoundedDynArray();
    msg2.data = msg->data;
    pub_u_int32_bounded_dyn_array_->publish(msg2);
    }

  void u_int32_fixed_array_callback(const idltest_msgs::msg::UInt32FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt32FixedArray();
    msg2.data = msg->data;
    pub_u_int32_fixed_array_->publish(msg2);
    }

  void u_int32_unbounded_dyn_array_callback(const idltest_msgs::msg::UInt32UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt32UnboundedDynArray();
    msg2.data = msg->data;
    pub_u_int32_unbounded_dyn_array_->publish(msg2);
    }

  void u_int64_callback(const idltest_msgs::msg::UInt64::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt64();
    msg2.data = msg->data;
    pub_u_int64_->publish(msg2);
    }

  void u_int64_bounded_dyn_array_callback(const idltest_msgs::msg::UInt64BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt64BoundedDynArray();
    msg2.data = msg->data;
    pub_u_int64_bounded_dyn_array_->publish(msg2);
    }

  void u_int64_fixed_array_callback(const idltest_msgs::msg::UInt64FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt64FixedArray();
    msg2.data = msg->data;
    pub_u_int64_fixed_array_->publish(msg2);
    }

  void u_int64_unbounded_dyn_array_callback(const idltest_msgs::msg::UInt64UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt64UnboundedDynArray();
    msg2.data = msg->data;
    pub_u_int64_unbounded_dyn_array_->publish(msg2);
    }

  void u_int8_callback(const idltest_msgs::msg::UInt8::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt8();
    msg2.data = msg->data;
    pub_u_int8_->publish(msg2);
    }

  void u_int8_bounded_dyn_array_callback(const idltest_msgs::msg::UInt8BoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt8BoundedDynArray();
    msg2.data = msg->data;
    pub_u_int8_bounded_dyn_array_->publish(msg2);
    }

  void u_int8_fixed_array_callback(const idltest_msgs::msg::UInt8FixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt8FixedArray();
    msg2.data = msg->data;
    pub_u_int8_fixed_array_->publish(msg2);
    }

  void u_int8_unbounded_dyn_array_callback(const idltest_msgs::msg::UInt8UnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::UInt8UnboundedDynArray();
    msg2.data = msg->data;
    pub_u_int8_unbounded_dyn_array_->publish(msg2);
    }

  void w_string_callback(const idltest_msgs::msg::WString::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::WString();
    msg2.data = msg->data;
    pub_w_string_->publish(msg2);
    }

  void w_string_bounded_dyn_array_callback(const idltest_msgs::msg::WStringBoundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::WStringBoundedDynArray();
    msg2.data = msg->data;
    pub_w_string_bounded_dyn_array_->publish(msg2);
    }

  void w_string_fixed_array_callback(const idltest_msgs::msg::WStringFixedArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::WStringFixedArray();
    msg2.data = msg->data;
    pub_w_string_fixed_array_->publish(msg2);
    }

  void w_string_unbounded_dyn_array_callback(const idltest_msgs::msg::WStringUnboundedDynArray::SharedPtr msg) {
    auto msg2 = idltest_msgs::msg::WStringUnboundedDynArray();
    msg2.data = msg->data;
    pub_w_string_unbounded_dyn_array_->publish(msg2);
    }

  rclcpp::Subscription<idltest_msgs::msg::Bool>::SharedPtr sub_bool_;
  rclcpp::Publisher<idltest_msgs::msg::Bool>::SharedPtr pub_bool_;
  rclcpp::Subscription<idltest_msgs::msg::BoolBoundedDynArray>::SharedPtr sub_bool_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::BoolBoundedDynArray>::SharedPtr pub_bool_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::BoolFixedArray>::SharedPtr sub_bool_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::BoolFixedArray>::SharedPtr pub_bool_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::BoolUnboundedDynArray>::SharedPtr sub_bool_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::BoolUnboundedDynArray>::SharedPtr pub_bool_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Byte>::SharedPtr sub_byte_;
  rclcpp::Publisher<idltest_msgs::msg::Byte>::SharedPtr pub_byte_;
  rclcpp::Subscription<idltest_msgs::msg::ByteBoundedDynArray>::SharedPtr sub_byte_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::ByteBoundedDynArray>::SharedPtr pub_byte_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::ByteFixedArray>::SharedPtr sub_byte_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::ByteFixedArray>::SharedPtr pub_byte_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::ByteUnboundedDynArray>::SharedPtr sub_byte_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::ByteUnboundedDynArray>::SharedPtr pub_byte_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Char>::SharedPtr sub_char_;
  rclcpp::Publisher<idltest_msgs::msg::Char>::SharedPtr pub_char_;
  rclcpp::Subscription<idltest_msgs::msg::CharBoundedDynArray>::SharedPtr sub_char_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::CharBoundedDynArray>::SharedPtr pub_char_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::CharFixedArray>::SharedPtr sub_char_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::CharFixedArray>::SharedPtr pub_char_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::CharUnboundedDynArray>::SharedPtr sub_char_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::CharUnboundedDynArray>::SharedPtr pub_char_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Float32>::SharedPtr sub_float32_;
  rclcpp::Publisher<idltest_msgs::msg::Float32>::SharedPtr pub_float32_;
  rclcpp::Subscription<idltest_msgs::msg::Float32BoundedDynArray>::SharedPtr sub_float32_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Float32BoundedDynArray>::SharedPtr pub_float32_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Float32FixedArray>::SharedPtr sub_float32_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::Float32FixedArray>::SharedPtr pub_float32_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::Float32UnboundedDynArray>::SharedPtr sub_float32_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Float32UnboundedDynArray>::SharedPtr pub_float32_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Float64>::SharedPtr sub_float64_;
  rclcpp::Publisher<idltest_msgs::msg::Float64>::SharedPtr pub_float64_;
  rclcpp::Subscription<idltest_msgs::msg::Float64BoundedDynArray>::SharedPtr sub_float64_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Float64BoundedDynArray>::SharedPtr pub_float64_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Float64FixedArray>::SharedPtr sub_float64_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::Float64FixedArray>::SharedPtr pub_float64_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::Float64UnboundedDynArray>::SharedPtr sub_float64_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Float64UnboundedDynArray>::SharedPtr pub_float64_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int16>::SharedPtr sub_int16_;
  rclcpp::Publisher<idltest_msgs::msg::Int16>::SharedPtr pub_int16_;
  rclcpp::Subscription<idltest_msgs::msg::Int16BoundedDynArray>::SharedPtr sub_int16_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int16BoundedDynArray>::SharedPtr pub_int16_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int16FixedArray>::SharedPtr sub_int16_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int16FixedArray>::SharedPtr pub_int16_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int16UnboundedDynArray>::SharedPtr sub_int16_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int16UnboundedDynArray>::SharedPtr pub_int16_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int32>::SharedPtr sub_int32_;
  rclcpp::Publisher<idltest_msgs::msg::Int32>::SharedPtr pub_int32_;
  rclcpp::Subscription<idltest_msgs::msg::Int32BoundedDynArray>::SharedPtr sub_int32_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int32BoundedDynArray>::SharedPtr pub_int32_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int32FixedArray>::SharedPtr sub_int32_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int32FixedArray>::SharedPtr pub_int32_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int32UnboundedDynArray>::SharedPtr sub_int32_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int32UnboundedDynArray>::SharedPtr pub_int32_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int64>::SharedPtr sub_int64_;
  rclcpp::Publisher<idltest_msgs::msg::Int64>::SharedPtr pub_int64_;
  rclcpp::Subscription<idltest_msgs::msg::Int64BoundedDynArray>::SharedPtr sub_int64_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int64BoundedDynArray>::SharedPtr pub_int64_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int64FixedArray>::SharedPtr sub_int64_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int64FixedArray>::SharedPtr pub_int64_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int64UnboundedDynArray>::SharedPtr sub_int64_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int64UnboundedDynArray>::SharedPtr pub_int64_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int8>::SharedPtr sub_int8_;
  rclcpp::Publisher<idltest_msgs::msg::Int8>::SharedPtr pub_int8_;
  rclcpp::Subscription<idltest_msgs::msg::Int8BoundedDynArray>::SharedPtr sub_int8_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int8BoundedDynArray>::SharedPtr pub_int8_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int8FixedArray>::SharedPtr sub_int8_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int8FixedArray>::SharedPtr pub_int8_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::Int8UnboundedDynArray>::SharedPtr sub_int8_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::Int8UnboundedDynArray>::SharedPtr pub_int8_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::String>::SharedPtr sub_string_;
  rclcpp::Publisher<idltest_msgs::msg::String>::SharedPtr pub_string_;
  rclcpp::Subscription<idltest_msgs::msg::StringBoundedDynArray>::SharedPtr sub_string_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::StringBoundedDynArray>::SharedPtr pub_string_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::StringFixedArray>::SharedPtr sub_string_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::StringFixedArray>::SharedPtr pub_string_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::StringUnboundedDynArray>::SharedPtr sub_string_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::StringUnboundedDynArray>::SharedPtr pub_string_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt16>::SharedPtr sub_u_int16_;
  rclcpp::Publisher<idltest_msgs::msg::UInt16>::SharedPtr pub_u_int16_;
  rclcpp::Subscription<idltest_msgs::msg::UInt16BoundedDynArray>::SharedPtr sub_u_int16_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt16BoundedDynArray>::SharedPtr pub_u_int16_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt16FixedArray>::SharedPtr sub_u_int16_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt16FixedArray>::SharedPtr pub_u_int16_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt16UnboundedDynArray>::SharedPtr sub_u_int16_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt16UnboundedDynArray>::SharedPtr pub_u_int16_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt32>::SharedPtr sub_u_int32_;
  rclcpp::Publisher<idltest_msgs::msg::UInt32>::SharedPtr pub_u_int32_;
  rclcpp::Subscription<idltest_msgs::msg::UInt32BoundedDynArray>::SharedPtr sub_u_int32_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt32BoundedDynArray>::SharedPtr pub_u_int32_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt32FixedArray>::SharedPtr sub_u_int32_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt32FixedArray>::SharedPtr pub_u_int32_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt32UnboundedDynArray>::SharedPtr sub_u_int32_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt32UnboundedDynArray>::SharedPtr pub_u_int32_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt64>::SharedPtr sub_u_int64_;
  rclcpp::Publisher<idltest_msgs::msg::UInt64>::SharedPtr pub_u_int64_;
  rclcpp::Subscription<idltest_msgs::msg::UInt64BoundedDynArray>::SharedPtr sub_u_int64_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt64BoundedDynArray>::SharedPtr pub_u_int64_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt64FixedArray>::SharedPtr sub_u_int64_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt64FixedArray>::SharedPtr pub_u_int64_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt64UnboundedDynArray>::SharedPtr sub_u_int64_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt64UnboundedDynArray>::SharedPtr pub_u_int64_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt8>::SharedPtr sub_u_int8_;
  rclcpp::Publisher<idltest_msgs::msg::UInt8>::SharedPtr pub_u_int8_;
  rclcpp::Subscription<idltest_msgs::msg::UInt8BoundedDynArray>::SharedPtr sub_u_int8_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt8BoundedDynArray>::SharedPtr pub_u_int8_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt8FixedArray>::SharedPtr sub_u_int8_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt8FixedArray>::SharedPtr pub_u_int8_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::UInt8UnboundedDynArray>::SharedPtr sub_u_int8_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::UInt8UnboundedDynArray>::SharedPtr pub_u_int8_unbounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::WString>::SharedPtr sub_w_string_;
  rclcpp::Publisher<idltest_msgs::msg::WString>::SharedPtr pub_w_string_;
  rclcpp::Subscription<idltest_msgs::msg::WStringBoundedDynArray>::SharedPtr sub_w_string_bounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::WStringBoundedDynArray>::SharedPtr pub_w_string_bounded_dyn_array_;
  rclcpp::Subscription<idltest_msgs::msg::WStringFixedArray>::SharedPtr sub_w_string_fixed_array_;
  rclcpp::Publisher<idltest_msgs::msg::WStringFixedArray>::SharedPtr pub_w_string_fixed_array_;
  rclcpp::Subscription<idltest_msgs::msg::WStringUnboundedDynArray>::SharedPtr sub_w_string_unbounded_dyn_array_;
  rclcpp::Publisher<idltest_msgs::msg::WStringUnboundedDynArray>::SharedPtr pub_w_string_unbounded_dyn_array_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MultiPubSub>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
