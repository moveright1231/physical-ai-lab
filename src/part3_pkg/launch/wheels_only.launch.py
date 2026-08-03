"""ch10 실습 - 바퀴 2개만 diff_drive_controller로 띄운다.

/cmd_vel (geometry_msgs/Twist) -> 좌우 바퀴 속도 -> /odom + odom->base_footprint TF
교안 본문이 /cmd_vel 이라고 적혀 있어 리맵으로 그 이름을 그대로 살렸다.
(컨트롤러 기본 토픽은 /diff_drive_controller/cmd_vel_unstamped 이다)
"""
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('part3_pkg')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'mm_mock.urdf.xacro'])]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([pkg, 'config', 'mm_controllers.yaml'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers],
        output='both',
        remappings=[('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_drive_controller/odom', '/odom')],
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    diff_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[diff_spawner])
    )

    return LaunchDescription([control_node, rsp_node, jsb_spawner, diff_after_jsb])
