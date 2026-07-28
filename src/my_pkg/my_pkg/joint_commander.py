import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.declare_parameter('target', [0.0, -1.17, 0.94, 0.30])
        self.pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2.0, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return
        target = self.get_parameter('target').value
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start.sec = 2
        traj.points = [point]
        self.pub.publish(traj)
        self.get_logger().info(f'목표 전송: {target}')
        self.sent = True


def main():
    rclpy.init()
    node = JointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
