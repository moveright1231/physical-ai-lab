import math
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def fk(q1, q2, q3, q4):
    p = np.array([0.0, 0.0, 0.0595])
    p = p + rot_y(q2) @ np.array([0.024, 0.0, 0.128])
    p = p + rot_y(q2 + q3) @ np.array([0.124, 0.0, 0.0])
    p = p + rot_y(q2 + q3 + q4) @ np.array([0.126, 0.0, 0.0])
    return np.array([0.012, 0.0, 0.017]) + rot_z(q1) @ p


class FkCheck(Node):
    def __init__(self):
        super().__init__('fk_check')
        self.declare_parameter('q', [0.0, 0.0, 0.0, 0.0])
        self.q = list(
            self.get_parameter('q').get_parameter_value().double_array_value)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.publish_joints)
        self.create_timer(2.0, self.compare)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4',
                    'gripper_left_joint', 'gripper_right_joint']
        msg.position = self.q + [0.0, 0.0]
        self.pub.publish(msg)

    def compare(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'link1', 'end_effector_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}')
            return
        t = tf.transform.translation
        ref = np.array([t.x, t.y, t.z])
        mine = fk(*self.q)
        self.get_logger().info(
            f"q = {[round(v, 4) for v in self.q]}\n"
            f"  my FK : ({mine[0]:.5f}, {mine[1]:.5f}, {mine[2]:.5f})\n"
            f"  tf2   : ({ref[0]:.5f}, {ref[1]:.5f}, {ref[2]:.5f})\n"
            f"  diff  : {np.linalg.norm(mine - ref) * 1000:.4f} mm")


def main():
    rclpy.init()
    node = FkCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
