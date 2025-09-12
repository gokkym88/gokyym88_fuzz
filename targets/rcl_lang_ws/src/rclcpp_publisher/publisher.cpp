// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(char* msg)
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("aaaa", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    msg_to_pub = msg;
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    message.data = msg_to_pub;
    count_++;

    if (!called) {
      uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      std::cout << "[time1] " << us << std::endl;
      called = true;
    }

    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    // printf("Publishing: \"%s\"\n", message.data.c_str());
    publisher_->publish(message);

    if (count_ == 10) {
      uint64_t us2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      std::cout << "[time2] " << us2 << std::endl;
      exit(0);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  bool called = false;
  char* msg_to_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MinimalPublisher>(argv[1]);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
