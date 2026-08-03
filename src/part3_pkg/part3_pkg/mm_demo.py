#!/usr/bin/env python3
"""ch11 실습 — 팔과 바퀴를 한 로봇에서 순서대로 움직인다.

배울 것
  1. 같은 로봇이어도 팔과 바퀴는 서로 다른 컨트롤러가 맡는다
  2. 그래서 명령을 보내는 토픽도 메시지 타입도 다르다
       팔   -> /arm_controller/joint_trajectory  (JointTrajectory)
       바퀴 -> /cmd_vel                          (Twist)

실행
  터미널 A: ros2 launch part3_pkg mm_bringup.launch.py
  터미널 B: ros2 run part3_pkg mm_demo
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

HOME    = [0.0,  0.000, 0.000, 0.000]   # 팔을 곧게 편 자세
PICK    = [0.0, -1.204, 1.091, 0.183]   # 손끝이 (0.150, 0, 0.150) 인 자세


class MMDemo(Node):
    def __init__(self):
        super().__init__('mm_demo')
        self.arm = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.wheels = self.create_publisher(Twist, '/cmd_vel', 10)

        # [STUDENT] ------------------------------------------------------
        # 동작 순서표: (몇 초에, 무엇을)
        # 팔과 바퀴는 별개의 컨트롤러라 서로를 기다려 주지 않습니다.
        # 겹치지 않게 시간을 벌려 두는 것이 여기서의 핵심입니다.
        self.plan = [
            (1.0, self.arm_to,    PICK),   # 팔을 접는다
            (4.0, self.drive,     0.10),   # 앞으로 간다
            (7.0, self.drive,     0.00),   # 멈춘다
            (8.0, self.arm_to,    HOME),   # 팔을 편다
        ]
        # [/STUDENT] -----------------------------------------------------

        self.step = 0
        self.t = 0.0
        self.create_timer(0.1, self.tick)

    def arm_to(self, target):
        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start.sec = 2

        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        msg.points = [point]
        self.arm.publish(msg)
        self.get_logger().info(f'팔  -> {target}')

    def drive(self, speed):
        msg = Twist()
        msg.linear.x = float(speed)
        self.wheels.publish(msg)
        self.get_logger().info(f'바퀴 -> {speed:+.2f} m/s')

    def tick(self):
        self.t += 0.1

        # 주행 중에는 cmd_vel_timeout(0.5초) 때문에 계속 보내 줘야 한다
        if 4.0 <= self.t < 7.0:
            self.drive_quiet(0.10)

        if self.step >= len(self.plan):
            return
        when, action, arg = self.plan[self.step]
        if self.t >= when:
            action(arg)
            self.step += 1
            if self.step == len(self.plan):
                self.get_logger().info('시나리오 종료 — Ctrl+C 로 빠져나가세요')

    def drive_quiet(self, speed):
        msg = Twist()
        msg.linear.x = float(speed)
        self.wheels.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(MMDemo())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
