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
from custom_msgs.msg import OpenSpace

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('open_space_publisher')
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_name", "open_space"),
                ("sub_name", "fake_scan"),
            ],
        )
        self.open = self.create_publisher(OpenSpace, self.get_parameter("pub_name").value, 10)
        # self.dist = self.create_publisher(Float32, "open_space/distance", 10)
        # self.angle = self.create_publisher(Float32, "open_space/angle", 10)
        self.subscription = self.create_subscription(LaserScan, self.get_parameter("sub_name").value, self.listener_callback, 10)
    
    def listener_callback(self, scan):
        msg = OpenSpace()
        # dist = Float32()
        msg.distance = max(scan.ranges)
        # self.dist.publish(dist)

        # angle = Float32()
        msg.angle = scan.ranges.index(msg.distance)*scan.angle_increment+scan.angle_min
        # self.angle.publish(angle)
        self.open.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.angle)
    
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
