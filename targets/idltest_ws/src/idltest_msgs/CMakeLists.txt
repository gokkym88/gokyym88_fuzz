cmake_minimum_required(VERSION 3.5)

project(idltest_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoolBoundedDynArray.msg"
  "msg/BoolFixedArray.msg"
  "msg/Bool.msg"
  "msg/BoolUnboundedDynArray.msg"
  "msg/ByteBoundedDynArray.msg"
  "msg/ByteFixedArray.msg"
  "msg/Byte.msg"
  "msg/ByteUnboundedDynArray.msg"
  "msg/CharBoundedDynArray.msg"
  "msg/CharFixedArray.msg"
  "msg/Char.msg"
  "msg/CharUnboundedDynArray.msg"
  "msg/EmptyBoundedDynArray.msg"
  "msg/EmptyFixedArray.msg"
  "msg/Empty.msg"
  "msg/EmptyUnboundedDynArray.msg"
  "msg/Float32BoundedDynArray.msg"
  "msg/Float32FixedArray.msg"
  "msg/Float32.msg"
  "msg/Float32UnboundedDynArray.msg"
  "msg/Float64BoundedDynArray.msg"
  "msg/Float64FixedArray.msg"
  "msg/Float64.msg"
  "msg/Float64UnboundedDynArray.msg"
  "msg/Int16BoundedDynArray.msg"
  "msg/Int16FixedArray.msg"
  "msg/Int16.msg"
  "msg/Int16UnboundedDynArray.msg"
  "msg/Int32BoundedDynArray.msg"
  "msg/Int32FixedArray.msg"
  "msg/Int32.msg"
  "msg/Int32UnboundedDynArray.msg"
  "msg/Int64BoundedDynArray.msg"
  "msg/Int64FixedArray.msg"
  "msg/Int64.msg"
  "msg/Int64UnboundedDynArray.msg"
  "msg/Int8BoundedDynArray.msg"
  "msg/Int8FixedArray.msg"
  "msg/Int8.msg"
  "msg/Int8UnboundedDynArray.msg"
  "msg/StringBoundedDynArray.msg"
  "msg/StringFixedArray.msg"
  "msg/String.msg"
  "msg/StringUnboundedDynArray.msg"
  "msg/UInt16BoundedDynArray.msg"
  "msg/UInt16FixedArray.msg"
  "msg/UInt16.msg"
  "msg/UInt16UnboundedDynArray.msg"
  "msg/UInt32BoundedDynArray.msg"
  "msg/UInt32FixedArray.msg"
  "msg/UInt32.msg"
  "msg/UInt32UnboundedDynArray.msg"
  "msg/UInt64BoundedDynArray.msg"
  "msg/UInt64FixedArray.msg"
  "msg/UInt64.msg"
  "msg/UInt64UnboundedDynArray.msg"
  "msg/UInt8BoundedDynArray.msg"
  "msg/UInt8FixedArray.msg"
  "msg/UInt8.msg"
  "msg/UInt8UnboundedDynArray.msg"
  "msg/WStringBoundedDynArray.msg"
  "msg/WStringFixedArray.msg"
  "msg/WString.msg"
  "msg/WStringUnboundedDynArray.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
