from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='my_pkg', executable='talker', name='talker'),
        Node(package='my_pkg', executable='listener', name='listener'),
        Node(package='my_pkg', executable='param_node', name='param_node',
             parameters=[{'speed': 1.0}]),
    ])
