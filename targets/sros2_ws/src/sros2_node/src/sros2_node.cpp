#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class RelayNode : public rclcpp::Node
{
public:
  RelayNode() : Node("sros2_node")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
    "sros2_input", 10, std::bind(&RelayNode::topic_callback, this, _1));
    pub_ = this->create_publisher<std_msgs::msg::String>(
    "sros2_output", 10);

    std::ofstream fp("/tmp/sros2_started");
    fp.close();
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    auto msg2 = std_msgs::msg::String();
    msg2.data = msg->data;
    pub_->publish(msg2);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelayNode>());
  rclcpp::shutdown();
  return 0;
}
