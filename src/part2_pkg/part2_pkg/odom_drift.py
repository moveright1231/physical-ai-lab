import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

R_NOM = 0.033      # TurtleBot3 Burger 바퀴 반지름
B = 0.160          # 좌우 바퀴 간격


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class OdomDrift(Node):

    def __init__(self):
        super().__init__('odom_drift')
        self.declare_parameter('err_left', 1.0)
        self.declare_parameter('err_right', 1.0)
        self.declare_parameter('distance', 20.0)
        self.declare_parameter('wheel_rate', 30.0)   # rad/s

        self.rl = R_NOM * self.get_parameter('err_left').value
        self.rr = R_NOM * self.get_parameter('err_right').value
        self.goal = self.get_parameter('distance').value
        self.wc = self.get_parameter('wheel_rate').value

        self.tx = self.ty = self.tth = 0.0     # 진짜 위치
        self.ox = self.oy = self.oth = 0.0     # 오도메트리가 믿는 위치
        self.s_true = 0.0
        self.s_odom = 0.0
        self.next_mark = 5.0
        self.done = False

        self.bc = TransformBroadcaster(self)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f'wheel radius  left {self.rl * 1000:.4f} mm   '
            f'right {self.rr * 1000:.4f} mm   '
            f'(odometry assumes {R_NOM * 1000:.4f} mm for both)')

    def step(self, dt):
        # 진짜 로봇: 실제 바퀴 반지름으로 움직인다
        vl, vr = self.wc * self.rl, self.wc * self.rr
        v = 0.5 * (vl + vr)
        w = (vr - vl) / B
        self.tth += w * dt
        self.tx += v * math.cos(self.tth) * dt
        self.ty += v * math.sin(self.tth) * dt
        self.s_true += v * dt

        # 오도메트리: 공칭 반지름을 믿는다. 그래서 회전을 전혀 감지 못한다
        vo = self.wc * R_NOM
        self.ox += vo * math.cos(self.oth) * dt
        self.oy += vo * math.sin(self.oth) * dt
        self.s_odom += vo * dt

    def on_timer(self):
        if not self.done:
            for _ in range(50):
                if self.s_true >= self.goal:
                    self.done = True
                    self.report('FINAL')
                    break
                self.step(0.001)
                if self.s_true >= self.next_mark:
                    self.report(f'{self.next_mark:>4.0f} m')
                    self.next_mark += 5.0
        self.broadcast()

    def report(self, tag):
        gap = math.hypot(self.tx - self.ox, self.ty - self.oy)
        self.get_logger().info(
            f'[{tag}] true ({self.tx:+8.3f}, {self.ty:+8.3f}) '
            f'{math.degrees(self.tth):+7.2f} deg | '
            f'odom ({self.ox:+8.3f}, {self.oy:+8.3f}) '
            f'{math.degrees(self.oth):+7.2f} deg | '
            f'GAP {gap:7.3f} m | '
            f'dist true {self.s_true:.3f} vs odom {self.s_odom:.3f}')

    def broadcast(self):
        now = self.get_clock().now().to_msg()
        for child, x, y, th in (
                ('base_link_true', self.tx, self.ty, self.tth),
                ('base_link_odom', self.ox, self.oy, self.oth)):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = yaw_to_quat(th)
            self.bc.sendTransform(t)


def main():
    rclpy.init()
    node = OdomDrift()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
