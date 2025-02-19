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

class DynamicTFCamPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')
        
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.T_base_right = np.eye(4)
        self.T_base_right[1, 3] = -0.05  # -0.05m along y
        self.T_base_left = np.eye(4)
        self.T_base_left[1, 3] = 0.05  # +0.05m along y
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not transform odom->base_link: {e}")
            return

        base_trans = [trans.transform.translation.x,
                      trans.transform.translation.y,
                      trans.transform.translation.z]
        base_rot = [trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w]
        
        T_odom_base = pose_to_transform(base_trans, base_rot)
        T_odom_left = np.dot(T_odom_base, self.T_base_left)
        left_trans, left_rot = transform_to_pose(T_odom_left)

        T_odom_right = np.dot(T_odom_base, self.T_base_right)
        T_left_inv = np.linalg.inv(T_odom_left)
        T_left_right = np.dot(T_left_inv, T_odom_right)
        right_trans, right_rot = transform_to_pose(T_left_right)

        # Create and publish the left camera transform (odom -> left_cam)
        left_msg = TransformStamped()
        left_msg.header.stamp = self.get_clock().now().to_msg()
        left_msg.header.frame_id = "odom"
        left_msg.child_frame_id = "left_cam"
        left_msg.transform.translation.x = float(left_trans[0])
        left_msg.transform.translation.y = float(left_trans[1])
        left_msg.transform.translation.z = float(left_trans[2])
        left_msg.transform.rotation.x = float(left_rot[0])
        left_msg.transform.rotation.y = float(left_rot[1])
        left_msg.transform.rotation.z = float(left_rot[2])
        left_msg.transform.rotation.w = float(left_rot[3])
        self.br.sendTransform(left_msg)

        # Create and publish the right camera transform (left_cam -> right_cam)
        right_msg = TransformStamped()
        right_msg.header.stamp = self.get_clock().now().to_msg()
        right_msg.header.frame_id = "left_cam"
        right_msg.child_frame_id = "right_cam"
        right_msg.transform.translation.x = float(right_trans[0])
        right_msg.transform.translation.y = float(right_trans[1])
        right_msg.transform.translation.z = float(right_trans[2])
        right_msg.transform.rotation.x = float(right_rot[0])
        right_msg.transform.rotation.y = float(right_rot[1])
        right_msg.transform.rotation.z = float(right_rot[2])
        right_msg.transform.rotation.w = float(right_rot[3])
        self.br.sendTransform(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
