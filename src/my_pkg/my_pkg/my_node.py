import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class MyNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('노드가 시작되었습니다!')


def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
