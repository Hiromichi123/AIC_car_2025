from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(pkg_path, '..', 'worlds', 'empty.world')
    urdf_xacro = os.path.join(pkg_path, '..', 'urdf', 'robot.urdf.xacro')

    # 发布 robot_description
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_xacro])}]
    )

    # 启动 Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', 
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            world_file,
        ],
        output='screen'
    )

    # 生成机器人实体
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mycar',
            '-x', '0.00',
            '-y', '0.00',
            '-z', '0.02'
        ],
        output='screen'
    )

    # 使用 spawner 加载控制器（正确的方法）
    load_mecanum_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller'],
        output='screen'
    )

    # 在机器人生成后延迟加载控制器
    load_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[load_mecanum_controller]
                )
            ]
        )
    )

    return LaunchDescription([
        robot_description,
        gazebo_process,
        TimerAction(period=5.0, actions=[spawn_entity_node]),
        load_controller_event
    ])