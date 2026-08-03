#!/usr/bin/env python3
"""ch09 실습 — 팔 관절을 컨트롤러로 움직이고, 결과를 되읽는다.

배울 것
  1. joint_trajectory_controller 에 목표 관절각을 보내는 방법
  2. /joint_states 는 '순서'가 아니라 '이름'으로 읽어야 한다는 것

실행
  터미널 A: ros2 launch part3_pkg arm_only.launch.py
  터미널 B: ros2 run part3_pkg arm_pose
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

# Part 2 에서 IK 로 구한, 손끝을 (0.150, 0, 0.150) 에 놓는 관절각
TARGET = [0.0, -1.204, 1.091, 0.183]


class ArmPose(Node):
    def __init__(self):
        super().__init__('arm_pose')
        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_states, 10)
        self.sent = False
        self.done = False
        self.create_timer(1.0, self.send_once)

    def send_once(self):
        """목표 자세를 한 번만 보낸다."""
        if self.sent:
            return
        point = JointTrajectoryPoint()
        point.positions = TARGET
        point.time_from_start.sec = 2          # 2초에 걸쳐 이동

        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS           # 이름과 값의 순서가 짝을 이룬다
        msg.points = [point]

        self.pub.publish(msg)
        self.sent = True
        self.get_logger().info(f'목표 자세 전송 {TARGET}')

    def on_states(self, msg: JointState):
        if not self.sent or self.done:
            return

        # [STUDENT] ------------------------------------------------------
        # /joint_states 의 관절 순서는 보장되지 않습니다.
        # 이 로봇의 실제 출력은 [joint2, joint3, joint1, joint4, ...] 입니다.
        # 그래서 msg.position[0] 을 joint1 이라고 믿으면 joint2 값을 읽게 됩니다.
        #
        # 이름과 값을 짝지어 사전으로 만든 뒤, 이름으로 꺼내세요.
        table = dict(zip(msg.name, msg.position))
        # [/STUDENT] -----------------------------------------------------

        now = [table[j] for j in ARM_JOINTS]
        error = max(abs(a - b) for a, b in zip(now, TARGET))
        if error > 1e-3:
            return                              # 아직 이동 중

        self.done = True
        self.get_logger().info('--- 도달 ---')
        for name, want, got in zip(ARM_JOINTS, TARGET, now):
            self.get_logger().info(f'  {name}  목표 {want:+.4f}   현재 {got:+.4f}')
        self.get_logger().info(f'최대 오차 {error:.2e}')
        self.get_logger().info('손끝 위치 확인: ros2 run tf2_ros tf2_echo link1 end_effector')


def main():
    rclpy.init()
    try:
        rclpy.spin(ArmPose())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
