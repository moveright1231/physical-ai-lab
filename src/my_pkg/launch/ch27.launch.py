from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='my_pkg', executable='dummy_robot', name='dummy_robot'),
        Node(package='my_pkg', executable='joint_reader', name='joint_reader'),
        Node(package='my_pkg', executable='joint_commander', name='joint_commander',
             parameters=[{'target': [0.0, -1.0, 0.8, 0.2]}]),
    ])
