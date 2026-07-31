import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MobileFrames(Node):

    def __init__(self):
        super().__init__('mobile_frames')
        self.declare_parameter('v', 0.2)    # m/s
        self.declare_parameter('w', 0.1)    # rad/s
        self.v = self.get_parameter('v').value
        self.w = self.get_parameter('w').value

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.dt = 0.05

        self.tf_bc = TransformBroadcaster(self)
        self.static_bc = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.publish_map_to_odom()
        self.create_timer(self.dt, self.on_timer)
        self.create_timer(2.0, self.report)

    def publish_map_to_odom(self):
        # 원래는 AMCL 같은 위치추정이 계속 갱신하는 값이다.
        # 여기서는 "보정량이 이렇게 생겼다"만 보이려고 고정값으로 둔다.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.5
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = yaw_to_quat(0.3)
        self.static_bc.sendTransform(t)

    def on_timer(self):
        self.th += self.w * self.dt
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt

        now = self.get_clock().now().to_msg()
        q = yaw_to_quat(self.th)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf_bc.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

    def report(self):
        self.get_logger().info(
            f'odom->base_link  x={self.x:+.3f}  y={self.y:+.3f}  '
            f'yaw={math.degrees(self.th):+.1f} deg')


def main():
    rclpy.init()
    node = MobileFrames()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
