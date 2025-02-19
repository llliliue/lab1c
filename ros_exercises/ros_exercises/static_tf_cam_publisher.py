import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticTFCamPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_cam_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)

        left_tf = TransformStamped()
        left_tf.header.stamp = self.get_clock().now().to_msg()
        left_tf.header.frame_id = "base_link"
        left_tf.child_frame_id = "left_cam"

        # Left camera is 0.05 m to the left of base_link (i.e. +0.05 in y).
        left_tf.transform.translation.x = 0.0
        left_tf.transform.translation.y = 0.05
        left_tf.transform.translation.z = 0.0
        left_tf.transform.rotation.x = 0.0
        left_tf.transform.rotation.y = 0.0
        left_tf.transform.rotation.z = 0.0
        left_tf.transform.rotation.w = 1.0

        right_tf = TransformStamped()
        right_tf.header.stamp = self.get_clock().now().to_msg()
        right_tf.header.frame_id = "left_cam"
        right_tf.child_frame_id = "right_cam"
        # Right camera is 0.1 m to the right of left camera (insane).
        right_tf.transform.translation.x = 0.0
        right_tf.transform.translation.y = -0.1
        right_tf.transform.translation.z = 0.0
        right_tf.transform.rotation.x = 0.0
        right_tf.transform.rotation.y = 0.0
        right_tf.transform.rotation.z = 0.0
        right_tf.transform.rotation.w = 1.0

        # Broadcast both static transforms at once.
        self.static_broadcaster.sendTransform([left_tf, right_tf])

        self.create_timer(1.0, rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFCamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()