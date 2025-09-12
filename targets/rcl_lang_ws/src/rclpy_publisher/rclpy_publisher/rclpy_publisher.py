#!/usr/bin/python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys, os, signal
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self, msg):
        super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        print("[app] will get publisher here")
        self.publisher_ = self.create_publisher(String, "aaaa", 10)
        print("[app] publisher is here")
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.called = False
        self.msg = msg

    def timer_callback(self):
        msg = String()
        # msg.data = 'Hello, world! %d' % self.i
        msg.data = self.msg

        if not self.called:
            print("[time1]", time.time())
            self.called = True

        # self.get_logger().info('Publishing: "%s"' % msg.data)

        print('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)

        self.i += 1
        if self.i == 10:
            print("[time2]", time.time())
            sys.exit(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(sys.argv[1])

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
