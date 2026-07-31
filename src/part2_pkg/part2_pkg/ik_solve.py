import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

BASE = np.array([0.012, 0.0, 0.017])   # link1 -> joint1
D2 = 0.0595                            # joint1 -> joint2
L1 = math.hypot(0.024, 0.128)          # joint2 -> joint3 (꺾인 링크)
PSI1 = math.atan2(0.128, 0.024)        # 그 링크가 a축에서 기울어진 각
L2 = 0.124                             # joint3 -> joint4
L3 = 0.126                             # joint4 -> end_effector


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def fk(q1, q2, q3, q4):
    p = np.array([0.0, 0.0, D2])
    p = p + rot_y(q2) @ np.array([0.024, 0.0, 0.128])
    p = p + rot_y(q2 + q3) @ np.array([L2, 0.0, 0.0])
    p = p + rot_y(q2 + q3 + q4) @ np.array([L3, 0.0, 0.0])
    return BASE + rot_z(q1) @ p


def ik(x, y, z, pitch, elbow='up'):
    """pitch: 마지막 링크가 수평면에서 이루는 각(rad). 위로 들면 +."""
    dx = x - BASE[0]
    q1 = math.atan2(y, dx)
    a = math.hypot(dx, y)          # 반경
    h = z - BASE[2] - D2           # joint2 기준 높이

    wa = a - L3 * math.cos(pitch)  # 손목 위치
    wh = h - L3 * math.sin(pitch)
    d = math.hypot(wa, wh)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None

    c = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    c = max(-1.0, min(1.0, c))
    delta = -math.acos(c) if elbow == 'up' else math.acos(c)

    u1 = math.atan2(wh, wa) - math.atan2(L2 * math.sin(delta),
                                         L1 + L2 * math.cos(delta))
    u2 = u1 + delta

    q2 = PSI1 - u1
    q3 = -u2 - q2
    q4 = -pitch + u2
    return [q1, q2, q3, q4]


class IkSolve(Node):

    def __init__(self):
        super().__init__('ik_solve')
        self.declare_parameter('target', [0.15, 0.0, 0.15])
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('elbow', 'up')
        self.declare_parameter('sweep', False)

        self.target = list(
            self.get_parameter('target')
            .get_parameter_value().double_array_value)
        pitch = math.radians(self.get_parameter('pitch_deg').value)
        elbow = self.get_parameter('elbow').value

        x, y, z = self.target
        self.q = ik(x, y, z, pitch, elbow)

        if self.q is None:
            self.get_logger().error(
                f'target ({x:.3f}, {y:.3f}, {z:.3f}) @ pitch '
                f'{math.degrees(pitch):.2f} deg -> OUT OF REACH')
        else:
            back = fk(*self.q)
            err = np.linalg.norm(back - np.array(self.target)) * 1000
            self.get_logger().info(
                f'\n  target    : ({x:.5f}, {y:.5f}, {z:.5f})'
                f'   pitch {math.degrees(pitch):.3f} deg, elbow {elbow}\n'
                f'  IK joints : {[round(v, 5) for v in self.q]}\n'
                f'  FK back   : ({back[0]:.5f}, {back[1]:.5f}, {back[2]:.5f})\n'
                f'  round-trip: {err:.6f} mm')

        if self.get_parameter('sweep').value:
            self.do_sweep(x, y, z, elbow)

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.publish_joints)
        self.create_timer(2.0, self.check_tf)

    def do_sweep(self, x, y, z, elbow):
        lines = ['pitch(deg)     q1        q2        q3        q4']
        n = 0
        for deg in range(-90, 91, 10):
            sol = ik(x, y, z, math.radians(deg), elbow)
            if sol is None:
                lines.append(f'{deg:>7}      (no solution)')
            else:
                n += 1
                lines.append(f'{deg:>7}    '
                             + '  '.join(f'{v:+.4f}' for v in sol))
        lines.append(f'-> one position, {n} solutions. '
                     f'4 DOF leaves pitch free.')
        self.get_logger().info('\n  ' + '\n  '.join(lines))

    def publish_joints(self):
        if self.q is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4',
                    'gripper_left_joint', 'gripper_right_joint']
        msg.position = list(self.q) + [0.0, 0.0]
        self.pub.publish(msg)

    def check_tf(self):
        if self.q is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                'link1', 'end_effector_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}')
            return
        t = tf.transform.translation
        err = np.linalg.norm(
            np.array([t.x, t.y, t.z]) - np.array(self.target)) * 1000
        self.get_logger().info(
            f'tf2 end_effector_link : ({t.x:.5f}, {t.y:.5f}, {t.z:.5f})'
            f'   vs target: {err:.6f} mm')


def main():
    rclpy.init()
    node = IkSolve()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
