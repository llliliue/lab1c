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

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import random 
from sensor_msgs.msg import LaserScan
import math

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topic", "fake_scan"),
                ("pub_freq", 20),
                ("angle_min", -(2/3) * math.pi),
                ("angle_max", (2/3) * math.pi),
                ("angle_inc", math.pi/300),
                ("range_min", 1.0),
                ("range_max", 10.0),
                
            ],
        )
        self.fake_publisher = self.create_publisher(LaserScan, self.get_parameter("topic").value, 10)
        timer_period = 1.0/self.get_parameter("pub_freq").value
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i =0
    
    def timer_callback(self):
        msg = LaserScan()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = self.get_parameter("angle_min").value
        msg.angle_max = self.get_parameter("angle_max").value
        msg.angle_increment = self.get_parameter("angle_inc").value
        msg.scan_time = 1.0/self.get_parameter("pub_freq").value
        msg.range_min = self.get_parameter("range_min").value
        msg.range_max = self.get_parameter("range_max").value
        msg.ranges = [
            random.uniform(msg.range_min, msg.range_max)
            for _ in range(
               1+int((msg.angle_max-msg.angle_min)/msg.angle_increment)
            )
        ]
        self.fake_publisher.publish(msg)
        
    # def __init__(self):
    #     super().__init__('minimal_publisher')
    #     self.publisher_ = self.create_publisher(String, 'topic', 10)
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    #     self.i = 0

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)

    fake_publisher = MinimalPublisher()

    rclpy.spin(fake_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
