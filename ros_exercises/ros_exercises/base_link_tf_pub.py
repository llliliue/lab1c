import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations as tft

def pose_to_transform(translation, quaternion):
    T = tft.quaternion_matrix(quaternion) 
    T[0:3, 3] = translation              
    return T

def transform_to_pose(T):
    translation = T[0:3, 3]
    quaternion = tft.quaternion_from_matrix(T)
    return translation, quaternion

class BaseLinkTFPublisher(Node):
    def __init__(self):
        super().__init__('base_link_tf_pub')
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.T_left_base = np.eye(4)
        self.T_left_base[1, 3] = -0.05
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'left_cam', self.get_clock().now())
        except Exception as e:
            self.get_logger().warn(f"Could not get transform from odom to left_cam: {e}")
            return

        left_trans = [trans.transform.translation.x,
                      trans.transform.translation.y,
                      trans.transform.translation.z]
        left_rot = [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w]

        T_odom_left = pose_to_transform(left_trans, left_rot)
        T_odom_base = np.dot(T_odom_left, self.T_left_base)
        translation, quaternion = transform_to_pose(T_odom_base)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link_2'
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
