import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class DummyRobot(Node):
    def __init__(self):
        super().__init__('dummy_robot')
        self.names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.current = [0.0, 0.0, 0.0, 0.0]
        self.target = list(self.current)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(
            JointTrajectory, '/arm_controller/joint_trajectory', self.on_traj, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('더미 로봇 시작')

    def on_traj(self, msg):
        if not msg.points:
            return
        point = msg.points[-1]
        mapping = dict(zip(msg.joint_names, point.positions))
        self.target = [mapping.get(n, c) for n, c in zip(self.names, self.current)]
        self.get_logger().info(f'목표 수신: {[round(v, 2) for v in self.target]}')

    def tick(self):
        step = 0.02
        for i, (cur, tgt) in enumerate(zip(self.current, self.target)):
            diff = tgt - cur
            self.current[i] = tgt if abs(diff) <= step else cur + (step if diff > 0 else -step)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = list(self.current)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DummyRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
