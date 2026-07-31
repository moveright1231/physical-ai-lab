import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState


class JointReader(Node):
    def __init__(self):
        super().__init__('joint_reader')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_state, 10)

    def on_state(self, msg):
        angles = [f'{n}={p:.2f}' for n, p in zip(msg.name, msg.position)]
        self.get_logger().info(' | '.join(angles))


def main():
    rclpy.init()
    node = JointReader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
