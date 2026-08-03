"""ch09 실습 - 팔 4관절만 ros2_control로 띄운다.

Gazebo 없이 mock_components/GenericSystem 으로 하드웨어를 흉내내므로
controller_manager / joint_state_broadcaster / joint_trajectory_controller 의
동작을 실제로 확인할 수 있다.
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

    # 하드웨어를 물고 컨트롤러를 굴리는 본체
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers],
        output='both',
    )

    # /joint_states -> TF (URDF를 읽어 링크 위치를 계산)
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

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    # 브로드캐스터가 먼저 떠야 /joint_states 가 살아있는 상태로 팔이 올라온다
    arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )

    return LaunchDescription([control_node, rsp_node, jsb_spawner, arm_after_jsb])
