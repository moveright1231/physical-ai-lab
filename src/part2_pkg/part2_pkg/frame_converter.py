import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class FrameConverter(Node):
    def __init__(self):
        super().__init__('frame_converter')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        p = PointStamped()
        p.header.frame_id = 'camera_link'
        p.point.x, p.point.y, p.point.z = 0.3, 0.0, 0.5
        try:
            tf = self.tf_buffer.lookup_transform(
                'link1', 'camera_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}')
            return
        q = do_transform_point(p, tf)
        self.get_logger().info(
            f'camera_link ({p.point.x:.3f}, {p.point.y:.3f}, {p.point.z:.3f})'
            f'  ->  link1 ({q.point.x:.3f}, {q.point.y:.3f}, {q.point.z:.3f})')


def main():
    rclpy.init()
    node = FrameConverter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
