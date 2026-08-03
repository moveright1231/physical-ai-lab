#!/usr/bin/env python3
"""ch10 실습 — /cmd_vel 로 주행 명령을 주고, 바퀴가 식대로 도는지 확인한다.

배울 것
  1. "앞으로 v, 제자리에서 w" 를 좌우 바퀴 각속도로 바꾸는 역기구학
  2. diff_drive_controller 가 정말 그 식대로 바퀴를 돌리는지 눈으로 확인

실행
  터미널 A: ros2 launch part3_pkg wheels_only.launch.py
  터미널 B: ros2 run part3_pkg drive_test
           ros2 run part3_pkg drive_test --ros-args -p v:=0.1 -p w:=0.0
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# TurtleBot3 Waffle 제원 — Part 3 컨트롤러 yaml 과 같은 값이어야 한다
WHEEL_RADIUS = 0.033       # r [m]
WHEEL_SEPARATION = 0.287   # L [m], 좌우 바퀴 사이 거리


def inverse_kinematics(v, w):
    """로봇 전체의 속도(v, w) -> 좌우 바퀴 각속도(rad/s)."""
    # [STUDENT] ----------------------------------------------------------
    # 회전할 때 안쪽 바퀴와 바깥쪽 바퀴가 그리는 원의 반지름은 L/2 만큼 다릅니다.
    # 그래서 각 바퀴가 지면에서 내야 하는 선속도는
    #     왼쪽  = v - w * L/2
    #     오른쪽 = v + w * L/2
    # 이고, 이걸 바퀴 각속도로 바꾸려면 반지름 r 로 나눕니다.
    wl = (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
    wr = (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
    # [/STUDENT] ---------------------------------------------------------
    return wl, wr


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test')
        self.declare_parameter('v', 0.0)     # 전진 속도 [m/s]
        self.declare_parameter('w', 1.0)     # 회전 속도 [rad/s]
        self.v = self.get_parameter('v').value
        self.w = self.get_parameter('w').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.on_states, 10)

        # cmd_vel_timeout 이 0.5초라 한 번만 보내면 바로 멈춘다 -> 10Hz 로 계속 보낸다
        self.create_timer(0.1, self.send)
        self.reported = False

        wl, wr = inverse_kinematics(self.v, self.w)
        self.get_logger().info(f'명령 v={self.v:+.3f} m/s, w={self.w:+.3f} rad/s')
        self.get_logger().info(f'식으로 계산한 바퀴 각속도  왼쪽 {wl:+.6f}  오른쪽 {wr:+.6f} rad/s')

    def send(self):
        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(self.w)
        self.pub.publish(msg)

    def on_states(self, msg: JointState):
        if self.reported:
            return
        table = dict(zip(msg.name, msg.velocity))    # 이름으로 꺼낸다 (ch09 참고)
        got_l = table.get('wheel_left_joint', 0.0)
        got_r = table.get('wheel_right_joint', 0.0)
        if abs(got_l) < 1e-9 and abs(got_r) < 1e-9:
            return                                    # 아직 안 움직임

        want_l, want_r = inverse_kinematics(self.v, self.w)
        self.reported = True
        self.get_logger().info('--- 실제 바퀴 각속도 ---')
        self.get_logger().info(f'  왼쪽   식 {want_l:+.6f}   실제 {got_l:+.6f}')
        self.get_logger().info(f'  오른쪽 식 {want_r:+.6f}   실제 {got_r:+.6f}')
        self.get_logger().info('Ctrl+C 로 종료하면 로봇도 멈춥니다')


def main():
    rclpy.init()
    try:
        rclpy.spin(DriveTest())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
