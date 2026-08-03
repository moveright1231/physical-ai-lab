"""ch11 통합 - 팔과 바퀴를 하나의 controller_manager 아래에서 동시에 굴린다.

연습문제 10의 정답을 코드로 보인 것:
팔용 launch와 바퀴용 launch를 따로 두 번 띄우면 controller_manager 가 중복 뜨면서
같은 하드웨어를 두 프로세스가 서로 잡으려 든다. 답은 '컨트롤러를 여러 개 두는 것'이지
'controller_manager 를 여러 개 두는 것'이 아니다.
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

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    # 브로드캐스터 -> 팔 -> 바퀴 순서로 올린다 (동시에 던지면 로드 경합이 난다)
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )
    after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner, on_exit=[diff_spawner])
    )

    return LaunchDescription([control_node, rsp_node, jsb_spawner, after_jsb, after_arm])
