import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        self.srv = self.create_service(AddTwoInts, 'add', self.handle_add)
        self.get_logger().info('add 서비스 대기 중')

    def handle_add(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a}+{request.b}={response.sum}')
        return response


def main():
    rclpy.init()
    node = AddServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
