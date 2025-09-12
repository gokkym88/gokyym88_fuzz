#!/usr/bin/python3


def to_alt(type_name):
    alt_str = ""
    for i, c in enumerate(type_name):
        if i == 0:
            alt_str += c.lower()

        else:
            if c.isupper():
                alt_str += "_" + c.lower()
            else:
                alt_str += c

    return alt_str


all_msgs = """\
Bool
BoolBoundedDynArray
BoolFixedArray
BoolUnboundedDynArray
Byte
ByteBoundedDynArray
ByteFixedArray
ByteUnboundedDynArray
Char
CharBoundedDynArray
CharFixedArray
CharUnboundedDynArray
Float32
Float32BoundedDynArray
Float32FixedArray
Float32UnboundedDynArray
Float64
Float64BoundedDynArray
Float64FixedArray
Float64UnboundedDynArray
Int16
Int16BoundedDynArray
Int16FixedArray
Int16UnboundedDynArray
Int32
Int32BoundedDynArray
Int32FixedArray
Int32UnboundedDynArray
Int64
Int64BoundedDynArray
Int64FixedArray
Int64UnboundedDynArray
Int8
Int8BoundedDynArray
Int8FixedArray
Int8UnboundedDynArray
String
StringBoundedDynArray
StringFixedArray
StringUnboundedDynArray
UInt16
UInt16BoundedDynArray
UInt16FixedArray
UInt16UnboundedDynArray
UInt32
UInt32BoundedDynArray
UInt32FixedArray
UInt32UnboundedDynArray
UInt64
UInt64BoundedDynArray
UInt64FixedArray
UInt64UnboundedDynArray
UInt8
UInt8BoundedDynArray
UInt8FixedArray
UInt8UnboundedDynArray
WString
WStringBoundedDynArray
WStringFixedArray
WStringUnboundedDynArray"""

list_all_msgs = all_msgs.split("\n")

for type_name in list_all_msgs:
    print(type_name, to_alt(type_name))


f = open("idltest_target.cpp", "w")

# header 1
f.write("""\
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
""")

# header 2
for type_name in list_all_msgs:
    f.write(f'#include "idltest_msgs/msg/{to_alt(type_name)}.hpp"\n')

# using
f.write("\nusing std::placeholders::_1;\n")

# class
f.write("""

class MultiPubSub : public rclcpp::Node
{
""")

# public (pub/sub definition)
f.write("""
public:
  MultiPubSub() : Node("multi_pubsub")
  {
""")

for tn in list_all_msgs:
    an = to_alt(tn)
    f.write(f'    sub_{an}_ = this->create_subscription<idltest_msgs::msg::{tn}>(\n')
    f.write(f'      "idltest_{tn}_in", 10, std::bind(&MultiPubSub::{an}_callback, this, _1));\n')
    f.write(f'    pub_{an}_ = this->create_publisher<idltest_msgs::msg::{tn}>(\n')
    f.write(f'      "idltest_{tn}_out", 10);\n\n')

f.write("  }\n\n")

# private (callbacks)
f.write("private:\n")

for tn in list_all_msgs:
    an = to_alt(tn)
    f.write(f"  void {an}_callback(const idltest_msgs::msg::{tn}::SharedPtr msg)" + " {\n")
    f.write(f"    auto msg2 = idltest_msgs::msg::{tn}();\n")
    f.write("    msg2.data = msg->data;\n")
    f.write(f"    pub_{an}_->publish(msg2);\n")
    f.write("    }\n\n")

for tn in list_all_msgs:
    an = to_alt(tn)
    f.write(f"  rclcpp::Subscription<idltest_msgs::msg::{tn}>::SharedPtr sub_{an}_;\n")
    f.write(f"  rclcpp::Publisher<idltest_msgs::msg::{tn}>::SharedPtr pub_{an}_;\n")

# end of class def
f.write("};\n")

# main
f.write("""
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MultiPubSub>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
""")
